/**
 * @file chassis_arbitration_node.cpp
 * @brief 底盘控制仲裁节点实现 — 三阶段仲裁 + 状态机
 */

#include "chassis_arbitration/chassis_arbitration_node.hpp"

#include <algorithm>
#include <cmath>

namespace chassis_arbitration
{

// ═════════════════════════════════════════════════════════════════
//  构造 / 析构
// ═════════════════════════════════════════════════════════════════

ChassisArbitrationNode::ChassisArbitrationNode()
: Node("chassis_arbitration_node")
{
  using namespace std::chrono_literals;

  // ── 声明参数 ──────────────────────────────────────────────────
  this->declare_parameter("timeout_ctrl_speed_ms", 200);
  this->declare_parameter("timeout_ctrl_position_ms", 200);
  this->declare_parameter("timeout_speed_command_ms", 500);
  this->declare_parameter("timeout_ctrl_hmi_ms", 500);
  this->declare_parameter("timeout_ctrl_fence_ms", 300);
  this->declare_parameter("timeout_ctrl_mag_steer_ms", 300);
  this->declare_parameter("timeout_ctrl_mag_stop_ms", 300);
  this->declare_parameter("enable_fence", false);
  this->declare_parameter("magnetic_guide_enabled", false);
  this->declare_parameter("fence_decelerate_velocity", 0.3);
  this->declare_parameter("max_velocity", 2.0);
  this->declare_parameter("max_steering_angle", 30.0);
  this->declare_parameter("recovery_brake_step", 5);
  this->declare_parameter("arbitration_rate", 50.0);
  this->declare_parameter("state_publish_rate", 10.0);
  this->declare_parameter("l4_position_nav_threshold", 20);

  // ── 读取参数 ──────────────────────────────────────────────────
  timeout_ctrl_speed_ms_     = this->get_parameter("timeout_ctrl_speed_ms").as_int();
  timeout_ctrl_position_ms_  = this->get_parameter("timeout_ctrl_position_ms").as_int();
  timeout_speed_command_ms_  = this->get_parameter("timeout_speed_command_ms").as_int();
  timeout_ctrl_hmi_ms_       = this->get_parameter("timeout_ctrl_hmi_ms").as_int();
  timeout_ctrl_fence_ms_     = this->get_parameter("timeout_ctrl_fence_ms").as_int();
  timeout_ctrl_mag_steer_ms_ = this->get_parameter("timeout_ctrl_mag_steer_ms").as_int();
  timeout_ctrl_mag_stop_ms_  = this->get_parameter("timeout_ctrl_mag_stop_ms").as_int();
  enable_fence_              = this->get_parameter("enable_fence").as_bool();
  magnetic_guide_enabled_    = this->get_parameter("magnetic_guide_enabled").as_bool();
  fence_decelerate_velocity_ = static_cast<float>(this->get_parameter("fence_decelerate_velocity").as_double());
  max_velocity_              = static_cast<float>(this->get_parameter("max_velocity").as_double());
  max_steering_angle_        = static_cast<float>(this->get_parameter("max_steering_angle").as_double());
  brake_step_                = this->get_parameter("recovery_brake_step").as_int();
  arbitration_rate_          = this->get_parameter("arbitration_rate").as_double();
  state_publish_rate_        = this->get_parameter("state_publish_rate").as_double();
  l4_position_nav_threshold_ = this->get_parameter("l4_position_nav_threshold").as_int();

  // ── 创建发布器 ──────────────────────────────────────────────────
  auto_spd_pub_ = this->create_publisher<yhs_can_interfaces::msg::AutoSpdCtrlCmd>(
    "/auto_spd_ctrl_cmd", 10);
  arbitration_state_pub_ = this->create_publisher<data_interfaces::msg::ArbitrationState>(
    "/arbitration_state", 10);

  // ── 创建订阅器 ──────────────────────────────────────────────────
  // L4
  ctrl_speed_sub_ = this->create_subscription<data_interfaces::msg::CtrlSpeed>(
    "/ctrl_speed", 10,
    [this](const data_interfaces::msg::CtrlSpeed::SharedPtr msg) { onCtrlSpeed(msg); });
  ctrl_position_sub_ = this->create_subscription<data_interfaces::msg::CtrlSpeed>(
    "/ctrl_position", 10,
    [this](const data_interfaces::msg::CtrlSpeed::SharedPtr msg) { onCtrlPosition(msg); });

  // L3
  speed_command_sub_ = this->create_subscription<data_interfaces::msg::SpeedCommand>(
    "/speed_command", 10,
    [this](const data_interfaces::msg::SpeedCommand::SharedPtr msg) { onSpeedCommand(msg); });

  // L2
  ctrl_hmi_sub_ = this->create_subscription<data_interfaces::msg::HmiCommand>(
    "/ctrl_hmi", 10,
    [this](const data_interfaces::msg::HmiCommand::SharedPtr msg) { onCtrlHmi(msg); });

  // L1
  if (enable_fence_) {
    ctrl_fence_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/ctrl_fence", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) { onCtrlFence(msg); });
    RCLCPP_INFO(this->get_logger(), "Fence monitor ENABLED");
  } else {
    RCLCPP_WARN(this->get_logger(), "Fence monitor DISABLED — /ctrl_fence will be ignored");
  }

  // L1 — 磁导航转向（始终订阅，不依赖 magnetic_guide_enabled）
  ctrl_mag_steer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/ctrl_mag_steer", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) { onCtrlMagSteer(msg); });

  // L1 — 磁导航安全信号（仅启用磁导航时订阅）
  if (magnetic_guide_enabled_) {
    ctrl_mag_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/ctrl_mag_stop", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { onCtrlMagStop(msg); });
    RCLCPP_INFO(this->get_logger(), "Magnetic guide safety ENABLED");
  } else {
    RCLCPP_WARN(this->get_logger(),
      "Magnetic guide safety DISABLED (magnetic_guide_enabled=false) — "
      "ESTOP脱轨检测已跳过，但磁导航转向仍然生效");
  }

  // 诊断
  chassis_info_fb_sub_ = this->create_subscription<yhs_can_interfaces::msg::ChassisInfoFb>(
    "/chassis_info_fb", 10,
    [this](const yhs_can_interfaces::msg::ChassisInfoFb::SharedPtr msg) { onChassisInfoFb(msg); });

  // ── 创建定时器 ──────────────────────────────────────────────────
  auto arbitration_period = std::chrono::duration<double>(1.0 / arbitration_rate_);
  arbitration_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(arbitration_period),
    [this]() { onArbitrationTick(); });

  auto state_period = std::chrono::duration<double>(1.0 / state_publish_rate_);
  state_publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(state_period),
    [this]() { onStatePublishTick(); });

  RCLCPP_INFO(this->get_logger(),
    "ChassisArbitrationNode started. arbitration=%.1fHz, state_publish=%.1fHz",
    arbitration_rate_, state_publish_rate_);
}

// ═════════════════════════════════════════════════════════════════
//  订阅回调
// ═════════════════════════════════════════════════════════════════

void ChassisArbitrationNode::onCtrlSpeed(
  const data_interfaces::msg::CtrlSpeed::SharedPtr msg)
{
  ctrl_speed_msg_ = msg;
  ctrl_speed_stamp_ = this->now();
  watchdog_counter_ = 0;
}

void ChassisArbitrationNode::onCtrlPosition(
  const data_interfaces::msg::CtrlSpeed::SharedPtr msg)
{
  ctrl_position_msg_ = msg;
  ctrl_position_stamp_ = this->now();
  watchdog_counter_ = 0;
}

void ChassisArbitrationNode::onSpeedCommand(
  const data_interfaces::msg::SpeedCommand::SharedPtr msg)
{
  speed_command_msg_ = msg;
  speed_command_stamp_ = this->now();
  watchdog_counter_ = 0;
}

void ChassisArbitrationNode::onCtrlHmi(
  const data_interfaces::msg::HmiCommand::SharedPtr msg)
{
  ctrl_hmi_msg_ = msg;
  ctrl_hmi_stamp_ = this->now();
  watchdog_counter_ = 0;
}

void ChassisArbitrationNode::onCtrlFence(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  ctrl_fence_msg_ = msg;
  ctrl_fence_stamp_ = this->now();
  watchdog_counter_ = 0;
}

void ChassisArbitrationNode::onCtrlMagSteer(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  ctrl_mag_steer_msg_ = msg;
  ctrl_mag_steer_stamp_ = this->now();
  watchdog_counter_ = 0;
}

void ChassisArbitrationNode::onCtrlMagStop(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  ctrl_mag_stop_msg_ = msg;
  ctrl_mag_stop_stamp_ = this->now();
  watchdog_counter_ = 0;
}

void ChassisArbitrationNode::onChassisInfoFb(
  const yhs_can_interfaces::msg::ChassisInfoFb::SharedPtr msg)
{
  chassis_info_fb_msg_ = msg;
  chassis_info_fb_stamp_ = this->now();
  watchdog_counter_ = 0;
}

// ═════════════════════════════════════════════════════════════════
//  定时器回调
// ═════════════════════════════════════════════════════════════════

void ChassisArbitrationNode::onArbitrationTick()
{
  // ── 看门狗 ────────────────────────────────────────────────────
  watchdog_counter_++;

  // ── 状态机分发 ────────────────────────────────────────────────
  switch (state_) {
    case ArbitrationStateMachine::INIT:
      // 首次进入，直接切换到 IDLE
      state_ = ArbitrationStateMachine::IDLE;
      RCLCPP_INFO(this->get_logger(), "State: INIT → IDLE");
      // 首周期发布一次安全默认值
      {
        ArbitrationResult default_result;
        last_result_ = default_result;
        publishCommand(default_result);
      }
      break;

    case ArbitrationStateMachine::IDLE:
      // IDLE 状态：安全信号可能尚未到达，不触发 ESTOP
      // 发布安全默认值（brake=100），等待有效输入后切换到 ARBITRATING
      {
        ArbitrationResult idle_result = buildCascadeResult();
        applyL1PostProcess(idle_result);
        updateStateMachine(idle_result);
        last_result_ = idle_result;
        publishCommand(idle_result);
      }
      break;

    case ArbitrationStateMachine::ARBITRATING: {
      // ── 阶段一: L1 安全预检查（仅在 ARBITRATING 状态下执行）───
      // ESTOP 只应在正常运行期间因安全信号丢失/触警而进入
      if (checkAndHandleEstop()) {
        return;  // ESTOP 已发布，本周期结束
      }

      // ── 阶段二: L4 → L3 → L2 级联构建 ────────────────────────
      ArbitrationResult result = buildCascadeResult();

      // ── 阶段三: L1 安全后处理 + 边界裁剪 ──────────────────────
      applyL1PostProcess(result);

      // ── 更新状态机 ───────────────────────────────────────────
      updateStateMachine(result);

      // ── 存储并发布输出 ───────────────────────────────────────
      last_result_ = result;
      publishCommand(result);
      break;
    }

    case ArbitrationStateMachine::ESTOP:
      // 持续发布 ESTOP 命令
      publishEstopCommand();
      // 更新 last_result_ 为 ESTOP 状态
      last_result_.gear = 1;
      last_result_.velocity = 0.0f;
      last_result_.brake = 100;
      last_result_.active_level = 1;
      last_result_.active_source = "estop";
      // 检查恢复条件
      if (checkRecoverConditions()) {
        state_ = ArbitrationStateMachine::RECOVER;
        recovery_brake_ = 100;
        RCLCPP_INFO(this->get_logger(),
          "State: ESTOP → RECOVER (recovery_brake=%d)", recovery_brake_);
      }
      break;

    case ArbitrationStateMachine::RECOVER: {
      // 渐进释放刹车
      ArbitrationResult result = stepRecover();
      last_result_ = result;
      publishCommand(result);
      if (recovery_brake_ <= 0) {
        state_ = ArbitrationStateMachine::ARBITRATING;
        RCLCPP_INFO(this->get_logger(), "State: RECOVER → ARBITRATING");
      }
      break;
    }
  }
}

void ChassisArbitrationNode::onStatePublishTick()
{
  auto msg = data_interfaces::msg::ArbitrationState();
  msg.header.stamp = this->now();
  msg.header.frame_id = "chassis_arbitration";

  // 使用最近一次仲裁结果
  msg.active_level     = last_result_.active_level;
  msg.active_source    = last_result_.active_source;
  msg.ctrl_cmd_gear    = last_result_.gear;
  msg.ctrl_cmd_velocity = last_result_.velocity;
  msg.ctrl_cmd_steering = last_result_.steering;
  msg.ctrl_cmd_brake   = last_result_.brake;
  msg.l4_source        = last_result_.l4_source;

  // 超时状态
  msg.input_timeout[0] = isTimeout(ctrl_speed_stamp_, timeout_ctrl_speed_ms_);
  msg.input_timeout[1] = isTimeout(ctrl_position_stamp_, timeout_ctrl_position_ms_);
  msg.input_timeout[2] = isTimeout(speed_command_stamp_, timeout_speed_command_ms_);
  // HMI 超时仅在手动模式下有效判断
  msg.input_timeout[3] = ctrl_hmi_msg_ && ctrl_hmi_msg_->hmi_ctrl_mode == 0
    ? isTimeout(ctrl_hmi_stamp_, timeout_ctrl_hmi_ms_) : false;
  msg.input_timeout[4] = enable_fence_ && isTimeout(ctrl_fence_stamp_, timeout_ctrl_fence_ms_);
  // 磁导航超时
  msg.input_timeout[5] = isTimeout(ctrl_mag_steer_stamp_, timeout_ctrl_mag_steer_ms_);
  msg.input_timeout[6] = magnetic_guide_enabled_
    ? isTimeout(ctrl_mag_stop_stamp_, timeout_ctrl_mag_stop_ms_) : false;

  // 安全信号状态
  msg.mag_off_track = magnetic_guide_enabled_ && ctrl_mag_stop_msg_ && ctrl_mag_stop_msg_->data;
  if (enable_fence_ && ctrl_fence_msg_) {
    msg.fence_warning = (ctrl_fence_msg_->data == 1);
    msg.fence_alarm   = (ctrl_fence_msg_->data == 2);
  }
  msg.hmi_override_active = ctrl_hmi_msg_ && ctrl_hmi_msg_->hmi_ctrl_mode == 0;

  arbitration_state_pub_->publish(msg);
}

// ═════════════════════════════════════════════════════════════════
//  阶段一: L1 安全预检查
// ═════════════════════════════════════════════════════════════════

bool ChassisArbitrationNode::checkAndHandleEstop()
{
  // 磁导航安全检查仅在启用磁导航时执行
  if (magnetic_guide_enabled_) {
    // 1. /ctrl_mag_stop 超时检查（危险侧原则：曾收到后丢失=脱轨）
    //    注意：从未收到过的不在此触发（由 updateStateMachine 保证进入 ARBITRATING 前安全信号已就绪）
    if (!isNeverReceived(ctrl_mag_stop_stamp_) &&
        isTimeout(ctrl_mag_stop_stamp_, timeout_ctrl_mag_stop_ms_)) {
      RCLCPP_ERROR(this->get_logger(), "L1 ESTOP: mag_stop timeout");
      state_ = ArbitrationStateMachine::ESTOP;
      publishEstopCommand();
      return true;
    }

    // 2. /ctrl_mag_stop == 1 (脱轨)
    if (ctrl_mag_stop_msg_ && ctrl_mag_stop_msg_->data) {
      RCLCPP_ERROR(this->get_logger(), "L1 ESTOP: magnetic guide off track");
      state_ = ArbitrationStateMachine::ESTOP;
      publishEstopCommand();
      return true;
    }
  }

  // 3. /ctrl_fence 超时检查（仅围栏功能启用时）
  if (enable_fence_ &&
      !isNeverReceived(ctrl_fence_stamp_) &&
      isTimeout(ctrl_fence_stamp_, timeout_ctrl_fence_ms_)) {
    RCLCPP_ERROR(this->get_logger(), "L1 ESTOP: fence timeout");
    state_ = ArbitrationStateMachine::ESTOP;
    publishEstopCommand();
    return true;
  }

  // 4. /ctrl_fence == 2（停车告警，仅围栏功能启用时）
  if (enable_fence_ && ctrl_fence_msg_ && ctrl_fence_msg_->data == 2) {
    RCLCPP_WARN(this->get_logger(), "L1 ESTOP: fence alarm (level 2)");
    state_ = ArbitrationStateMachine::ESTOP;
    publishEstopCommand();
    return true;
  }

  return false;  // 安全，继续阶段二
}

void ChassisArbitrationNode::publishEstopCommand()
{
  auto cmd = yhs_can_interfaces::msg::AutoSpdCtrlCmd();

  cmd.ctrl_cmd_gear     = 1;   // P 档
  cmd.ctrl_cmd_velocity = 0.0f;
  cmd.ctrl_cmd_brake    = 100;

  // 转向角: 使用 mag_steer（如果有效），否则回正
  if (!isTimeout(ctrl_mag_steer_stamp_, timeout_ctrl_mag_steer_ms_)
      && ctrl_mag_steer_msg_) {
    cmd.ctrl_cmd_steering = ctrl_mag_steer_msg_->data;
  } else {
    cmd.ctrl_cmd_steering = 0.0f;
  }

  auto_spd_pub_->publish(cmd);
}

// ═════════════════════════════════════════════════════════════════
//  阶段二: L4 → L3 → L2 级联构建
// ═════════════════════════════════════════════════════════════════

ArbitrationResult ChassisArbitrationNode::buildCascadeResult()
{
  // L4 基础层
  ArbitrationResult result = buildL4Base();

  // L3 行为树档位覆盖
  applyL3Override(result);

  // L2 HMI 手动接管
  applyL2Override(result);

  return result;
}

// ── L4: 运动指令基础层 ───────────────────────────────────────────

ArbitrationResult ChassisArbitrationNode::buildL4Base()
{
  ArbitrationResult result;

  // ── 步骤1: 源选择 ──────────────────────────────────────────────
  // speed_command 非周期发送，不做超时判断，只要收到过就用其 ctrl_mode 决策
  bool use_position = false;
  if (speed_command_msg_) {
    if (speed_command_msg_->ctrl_mode >= static_cast<uint8_t>(l4_position_nav_threshold_)) {
      use_position = true;  // ctrl_mode >= 20 → /ctrl_position
    }
  }

  // 获取所选源的 CtrlSpeed
  data_interfaces::msg::CtrlSpeed::SharedPtr selected_src;
  int timeout_ms;
  const char * source_name;

  if (use_position) {
    selected_src = ctrl_position_msg_;
    timeout_ms   = timeout_ctrl_position_ms_;
    source_name  = "position_navigation";
    result.l4_source = 2;
  } else {
    selected_src = ctrl_speed_msg_;
    timeout_ms   = timeout_ctrl_speed_ms_;
    source_name  = "speed_track";
    result.l4_source = 1;
  }

  // ── 步骤2: 超时/降级处理 ──────────────────────────────────────
  rclcpp::Time stamp = use_position ? ctrl_position_stamp_ : ctrl_speed_stamp_;

  if (isNeverReceived(stamp) || isTimeout(stamp, timeout_ms) || !selected_src) {
    // 所选源不可用 → 降级值
    result.gear     = 4;    // D 档
    result.velocity = 0.0f;
    result.brake    = 100;
    result.steering = 0.0f;
    result.active_level  = 0;
    result.active_source = "none (L4 timeout)";
    result.l4_source     = 0;
    return result;
  }

  // ── 步骤3: 速度符号 → 档位 + 绝对值转换 ───────────────────────
  float raw_speed = selected_src->ctrl_speed;
  if (raw_speed >= 0.0f) {
    result.gear     = 4;  // D 档
    result.velocity = raw_speed;
  } else {
    result.gear     = 2;  // R 档
    result.velocity = std::fabs(raw_speed);
  }
  result.brake    = selected_src->ctrl_break;
  result.steering = 0.0f;  // L4 不控制转向
  result.active_level  = 4;
  result.active_source = source_name;

  return result;
}

// ── L3: 行为树档位覆盖层 ─────────────────────────────────────────

void ChassisArbitrationNode::applyL3Override(ArbitrationResult & result)
{
  // 仅当 HMI 在线且处于手动模式(0)时 L3 才旁路，由 L2 HMI 接管
  // HMI 未上线时 L3 行为树档位下发仍然生效
  if (ctrl_hmi_msg_ && ctrl_hmi_msg_->hmi_ctrl_mode == 0) {
    return;
  }

  if (!speed_command_msg_) {
    // 从未收到 speed_command → 透传 L4
    return;
  }

  uint8_t ctrl_mode   = speed_command_msg_->ctrl_mode;
  uint8_t target_gare = speed_command_msg_->target_gare;

  if (ctrl_mode == 0) {
    // ── 情况A: 刹车指令 ──
    result.brake    = 100;
    result.velocity = 0.0f;
    result.active_level  = 3;
    result.active_source = "behavior_tree";
  } else {
    // ── 情况B: 正常运动 ──
    if (target_gare == 1) {
      // P 档: 强制停车
      result.gear     = 1;
      result.velocity = 0.0f;
      result.active_level  = 3;
      result.active_source = "behavior_tree";
    } else if (target_gare == 3) {
      // N 档: 强制停车
      result.gear     = 3;
      result.velocity = 0.0f;
      result.active_level  = 3;
      result.active_source = "behavior_tree";
    } else if (target_gare == 2 || target_gare == 4) {
      // ★ R 或 D 档 → 沿用 L4 的 gear（由 L4 速度符号决定实际方向）
    } else {
      // 无效 target_gare → 保持 L4
    }
  }
}

// ── L2: HMI 手动接管层 ───────────────────────────────────────────

void ChassisArbitrationNode::applyL2Override(ArbitrationResult & result)
{
  if (!ctrl_hmi_msg_) {
    return;
  }

  if (ctrl_hmi_msg_->hmi_ctrl_mode == 0
      && isTimeout(ctrl_hmi_stamp_, timeout_ctrl_hmi_ms_)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "L2: HMI timeout in manual mode, falling through to L3");
    return;
  }

  if (ctrl_hmi_msg_->hmi_ctrl_mode == 0) {
    // ── 手动接管 ──
    result.gear     = ctrl_hmi_msg_->ctrl_gare;
    result.velocity = ctrl_hmi_msg_->ctrl_speed;
    result.brake    = ctrl_hmi_msg_->ctrl_break;
    result.active_level  = 2;
    result.active_source = "hmi_manual";
  }
}

// ═════════════════════════════════════════════════════════════════
//  阶段三: L1 安全后处理 + 边界裁剪
// ═════════════════════════════════════════════════════════════════

void ChassisArbitrationNode::applyL1PostProcess(ArbitrationResult & result)
{
  // ── 4.5.1 电子围栏减速限速 (fence = 1，仅围栏功能启用时) ──────
  if (enable_fence_
      && !isTimeout(ctrl_fence_stamp_, timeout_ctrl_fence_ms_)
      && ctrl_fence_msg_
      && ctrl_fence_msg_->data == 1) {
    result.velocity = std::min(result.velocity, fence_decelerate_velocity_);
    result.active_level = 1;
    if (result.active_source.find("fence_decelerate") == std::string::npos) {
      result.active_source += " + fence_decelerate";
    }
  }

  // ── 4.5.2 磁导航转向 (L1 级转向，始终生效) ──
  if (!isTimeout(ctrl_mag_steer_stamp_, timeout_ctrl_mag_steer_ms_)
      && ctrl_mag_steer_msg_) {
    result.steering = ctrl_mag_steer_msg_->data;
  } else if (!isNeverReceived(ctrl_mag_steer_stamp_)) {
    // 曾经收到但超时 → 回正（安全侧）
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "L1: mag_steer timeout, steering reset to 0");
    result.steering = 0.0f;
  }
  // 从未收到 mag_steer → 保留 L4/L3/L2 的 steering 值

  // ── 4.5.3 速度边界裁剪 ─────────────────────────────────────────
  // speed_command 非周期发送，不做超时判断
  if (speed_command_msg_) {
    if (result.gear == 4) {
      result.velocity = std::min(result.velocity, speed_command_msg_->max_forward_vel);
    } else if (result.gear == 2) {
      result.velocity = std::min(result.velocity, speed_command_msg_->max_backward_vel);
    }
  }

  // 全局硬限制
  result.velocity = std::min(result.velocity, max_velocity_);

  // ── 4.5.4 最终值域钳位 ─────────────────────────────────────────
  result.brake    = clampU8(result.brake, 0, 100);
  result.velocity = clamp(result.velocity, 0.0f, max_velocity_);
  result.steering = clamp(result.steering, -max_steering_angle_, max_steering_angle_);
  result.gear     = clampU8(result.gear, 1, 4);
}

// ═════════════════════════════════════════════════════════════════
//  状态机
// ═════════════════════════════════════════════════════════════════

void ChassisArbitrationNode::updateStateMachine(const ArbitrationResult & result)
{
  if (result.active_level == 0 && result.l4_source == 0) {
    // L4 双路均超时且无有效输入 → IDLE
    if (state_ != ArbitrationStateMachine::IDLE) {
      state_ = ArbitrationStateMachine::IDLE;
      RCLCPP_INFO(this->get_logger(), "State: → IDLE (no valid L4 source)");
    }
  } else {
    if (state_ == ArbitrationStateMachine::IDLE) {
      // 仅当关键安全信号已建立时才允许进入 ARBITRATING
      // fence 信号仅在 enable_fence_=true 时要求
      // mag_stop 信号仅在 magnetic_guide_enabled_=true 时要求
      bool fence_ready = !enable_fence_ || !isNeverReceived(ctrl_fence_stamp_);
      bool mag_ready = !magnetic_guide_enabled_ || !isNeverReceived(ctrl_mag_stop_stamp_);
      if (mag_ready && fence_ready) {
        state_ = ArbitrationStateMachine::ARBITRATING;
        RCLCPP_INFO(this->get_logger(), "State: IDLE → ARBITRATING");
      }
    }
  }
}

bool ChassisArbitrationNode::checkRecoverConditions()
{
  // 1. mag_stop == 0（未脱轨），磁导航禁用时自动满足
  bool mag_ok = !magnetic_guide_enabled_
                || (!isTimeout(ctrl_mag_stop_stamp_, timeout_ctrl_mag_stop_ms_)
                    && ctrl_mag_stop_msg_ && !ctrl_mag_stop_msg_->data);

  // 2. fence == 0（无危险，或围栏功能未启用时跳过此检查）
  bool fence_ok = !enable_fence_
                  || (!isTimeout(ctrl_fence_stamp_, timeout_ctrl_fence_ms_)
                      && ctrl_fence_msg_ && ctrl_fence_msg_->data == 0);

  // 3. hmi_ctrl_mode == 4（HMI 主动发送恢复指令）
  bool hmi_recover = ctrl_hmi_msg_ && ctrl_hmi_msg_->hmi_ctrl_mode == 4;

  if (mag_ok && fence_ok && hmi_recover) {
    RCLCPP_INFO(this->get_logger(),
      "ESTOP recover conditions met: mag_ok=%d, fence_ok=%d, hmi_recover=%d",
      mag_ok, fence_ok, hmi_recover);
    return true;
  }
  return false;
}

ArbitrationResult ChassisArbitrationNode::stepRecover()
{
  ArbitrationResult result;

  result.gear     = 4;  // D 档（预备状态）
  result.velocity = 0.0f;
  result.brake    = static_cast<uint8_t>(recovery_brake_);

  // 转向: 仅在启用磁导航时使用 mag_steer，否则回正
  if (magnetic_guide_enabled_
      && !isTimeout(ctrl_mag_steer_stamp_, timeout_ctrl_mag_steer_ms_)
      && ctrl_mag_steer_msg_) {
    result.steering = ctrl_mag_steer_msg_->data;
  } else {
    result.steering = 0.0f;
  }

  result.active_level  = 1;
  result.active_source = "recovering";

  // 递减刹车
  recovery_brake_ = std::max(0, recovery_brake_ - brake_step_);

  return result;
}

// ═════════════════════════════════════════════════════════════════
//  输出
// ═════════════════════════════════════════════════════════════════

void ChassisArbitrationNode::publishCommand(const ArbitrationResult & result)
{
  auto cmd = yhs_can_interfaces::msg::AutoSpdCtrlCmd();

  cmd.ctrl_cmd_gear     = result.gear;
  cmd.ctrl_cmd_velocity = result.velocity;
  cmd.ctrl_cmd_steering = result.steering;
  cmd.ctrl_cmd_brake    = result.brake;

  auto_spd_pub_->publish(cmd);
}

// ═════════════════════════════════════════════════════════════════
//  工具函数
// ═════════════════════════════════════════════════════════════════

bool ChassisArbitrationNode::isTimeout(const rclcpp::Time & stamp, int timeout_ms) const
{
  if (isNeverReceived(stamp)) {
    return true;  // 从未收到 → 视为超时
  }
  auto elapsed = this->now() - stamp;
  return elapsed.seconds() * 1000.0 > static_cast<double>(timeout_ms);
}

bool ChassisArbitrationNode::isNeverReceived(const rclcpp::Time & stamp) const
{
  return stamp.nanoseconds() == 0 && stamp.seconds() == 0;
}

float ChassisArbitrationNode::clamp(float val, float lo, float hi)
{
  return std::max(lo, std::min(val, hi));
}

uint8_t ChassisArbitrationNode::clampU8(uint8_t val, uint8_t lo, uint8_t hi)
{
  return std::max(lo, std::min(val, hi));
}

}  // namespace chassis_arbitration

// ═════════════════════════════════════════════════════════════════
//  main
// ═════════════════════════════════════════════════════════════════

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<chassis_arbitration::ChassisArbitrationNode>());
  rclcpp::shutdown();
  return 0;
}
