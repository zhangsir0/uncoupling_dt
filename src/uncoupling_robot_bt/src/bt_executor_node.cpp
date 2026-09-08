/**
 * @file bt_executor_node.cpp
 * @brief 摘钩机器人行为树执行器 — 所有 ROS2 I/O 统一管理
 *
 * ## 架构
 *   订阅 → 回调直接写黑板 → BT 叶子节点通过 InputPort/getAnyLocked 读取
 *   BT 叶子节点写黑板标记 → performTick 轮询 → 发布到话题
 *
 * ## 订阅 (话题→黑板)
 *   /co_arm_status, /arm_status, /ctrl_hmi, /detect_result,
 *   /ctrl_mag_stop, /ctrl_fence, /cips/hook_task_array,
 *   /cips_status, /speed_state
 *
 * ## 发布 (黑板→话题)
 *   /co_arm_cmd, /arm_cmd, /speed_command, /cips_cmd, /bt_hmi_state
 */

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "uncoupling_robot_bt/robot_nodes.h"
#include "data_interfaces/msg/hmi_command.hpp"
#include "data_interfaces/msg/bt_hmi_state.hpp"
#include "data_interfaces/msg/detect_result.hpp"
#include "data_interfaces/msg/hook_task_array.hpp"
#include "data_interfaces/msg/speed_command.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>

using namespace std::chrono_literals;

// 广播黑板写入到 root + 所有 subtree
#define SET_BB_ALL(tree, key, val) do { \
  (tree)->rootBlackboard()->set((key), (val)); \
  for (const auto& st : (tree)->subtrees) st->blackboard->set((key), (val)); \
} while(0)

class BtExecutorNode : public rclcpp::Node
{
public:
  BtExecutorNode() : Node("bt_executor_node")
  {
    // ── 参数 ─────────────────────────────────────────────────
    this->declare_parameter("bt_xml_path", "config/uncoupling_robot_bt_dual_mode.xml");
    if (!this->has_parameter("tick_rate"))
      this->declare_parameter("tick_rate", 50.0);
    if (!this->has_parameter("step_mode"))
      this->declare_parameter("step_mode", false);
    if (!this->has_parameter("groot_port"))
      this->declare_parameter("groot_port", 1667);
    if (!this->has_parameter("enable_groot"))
      this->declare_parameter("enable_groot", true);

    // ── 发布 ─────────────────────────────────────────────────
    hmi_state_pub_     = create_publisher<data_interfaces::msg::BtHmiState>("/bt_hmi_state", 10);
    co_arm_cmd_pub_    = create_publisher<std_msgs::msg::Int32MultiArray>("/co_arm_cmd", 10);
    arm_cmd_pub_       = create_publisher<std_msgs::msg::Int32MultiArray>("/arm_cmd", 10);
    speed_cmd_pub_     = create_publisher<data_interfaces::msg::SpeedCommand>("/speed_command", 10);
    cips_cmd_pub_      = create_publisher<std_msgs::msg::Int32MultiArray>("/cips_cmd", 10);

    hmi_state_timer_ = create_wall_timer(100ms, [this]() { publishHmiState(); });

    // ── Mock 心跳: 每 10s 触发一次 estop_heartbeat=1 (模拟心跳丢失) ──
    heartbeat_timer_ = create_wall_timer(10s, [this]() {
      // static bool toggle = false;
      // toggle = !toggle;
      // if (toggle && tree_) {
      //   RCLCPP_WARN(get_logger(), "[Mock] 心跳丢失 → estop_heartbeat=1");
      //   SET_BB_ALL(tree_, "estop_heartbeat", 1);
      // }
    });

    // ── 回调组 ───────────────────────────────────────────────
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // ═══════════════════════════════════════════════════════
    // 订阅: 收到消息 → 直接写黑板
    // ═══════════════════════════════════════════════════════

    // ── /co_arm_status → co_arm_40001/2/3 ──
    co_arm_status_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/co_arm_status", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (!tree_) {
          RCLCPP_WARN(get_logger(), "[co_arm] tree_ 为空, 丢弃 %zu 个数据", msg->data.size());
          return;
        }
        for (size_t i = 0; i + 1 < msg->data.size(); i += 2) {
          int32_t addr = static_cast<int32_t>(msg->data[i]);
          int32_t val  = static_cast<int32_t>(msg->data[i + 1]);
          // RCLCPP_INFO(get_logger(), "[co_arm]   解析: addr=%d val=%d", addr, val);
          if      (addr == 40001) SET_BB_ALL(tree_, "co_arm_40001", val);
          else if (addr == 40002) SET_BB_ALL(tree_, "co_arm_40002", val);
          else if (addr == 40003) SET_BB_ALL(tree_, "co_arm_40003", val);
        }
      }, sub_opt);

    // ── /arm_status → arm_40001/2/3 ──
    arm_status_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/arm_status", 10,
      [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!tree_) {
          RCLCPP_WARN(get_logger(), "[arm] tree_ 为空, 丢弃 %zu 个数据", msg->data.size());
          return;
        }
        for (size_t i = 0; i + 1 < msg->data.size(); i += 2) {
          int32_t addr = static_cast<int32_t>(msg->data[i]);
          int32_t val  = static_cast<int32_t>(msg->data[i + 1]);
          // RCLCPP_INFO(get_logger(), "[arm]   解析: addr=%d val=%d", addr, val);
          if      (addr == 40001) SET_BB_ALL(tree_, "arm_40001", val);
          else if (addr == 40002) SET_BB_ALL(tree_, "arm_40002", val);
          else if (addr == 40003) SET_BB_ALL(tree_, "arm_40003", val);
        }
      }, sub_opt);

    // ── /ctrl_hmi → hmi_ctrl_mode, work_mode ──
    hmi_command_sub_ = create_subscription<data_interfaces::msg::HmiCommand>(
      "/ctrl_hmi", 10,
      [this](const data_interfaces::msg::HmiCommand::SharedPtr msg) {
        handleHmiCommand(msg);
      }, sub_opt);

    // ── /detect_result → detect_id, detect_distance, detect_velocity ──
    detect_result_sub_ = create_subscription<data_interfaces::msg::DetectResult>(
      "/detect_result", 10,
      [this](const data_interfaces::msg::DetectResult::SharedPtr msg) {
        if (!tree_) return;
        SET_BB_ALL(tree_, "detect_id",       static_cast<int>(msg->current_id));
        SET_BB_ALL(tree_, "detect_distance", static_cast<double>(msg->current_distance));
        SET_BB_ALL(tree_, "detect_velocity", static_cast<double>(msg->relative_velocity));
        // RCLCPP_INFO(get_logger(), "[黑板] detect_id=%d dist=%.2f vel=%.2f",
        //             msg->current_id, msg->current_distance, msg->relative_velocity);
      }, sub_opt);

    // ── /ctrl_mag_stop → ctrl_mag_stop ──
    ctrl_mag_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/ctrl_mag_stop", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!tree_) return;
        SET_BB_ALL(tree_, "ctrl_mag_stop", msg->data);
      }, sub_opt);

    // ── /ctrl_fence → ctrl_fence ──
    ctrl_fence_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/ctrl_fence", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        if (!tree_) return;
        SET_BB_ALL(tree_, "ctrl_fence", static_cast<int>(msg->data));
      }, sub_opt);

    // ── /cips/hook_task_array → cips_total_hook_num, cips_task_array ──
    cips_task_sub_ = create_subscription<data_interfaces::msg::HookTaskArray>(
      "/cips/hook_task_array", 10,
      [this](const data_interfaces::msg::HookTaskArray::SharedPtr msg) {
        if (!tree_) return;
        SET_BB_ALL(tree_, "cips_total_hook_num", static_cast<int>(msg->total_hook_num));
        SET_BB_ALL(tree_, "cips_task_array", msg);
      }, sub_opt);

    // ── /cips_status → cips_40001 ──
    cips_status_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/cips_status", 10,
      [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!tree_) return;
        for (size_t i = 0; i + 1 < msg->data.size(); i += 2) {
          int32_t addr = static_cast<int32_t>(msg->data[i]);
          int32_t val  = static_cast<int32_t>(msg->data[i + 1]);
          if (addr == 40001) SET_BB_ALL(tree_, "cips_40001", val);
        }
      }, sub_opt);

    // ── /speed_state → speed_state (0=停止,1=运动中,2=对齐,3=完成) ──
    speed_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/speed_state", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        if (!tree_) return;
        SET_BB_ALL(tree_, "speed_state", static_cast<int>(msg->data));
        // RCLCPP_INFO(get_logger(), "[黑板] speed_state=%d", msg->data);
      }, sub_opt);

    // ── 初始化行为树 ─────────────────────────────────────────
    if (!initBehaviorTree()) {
      RCLCPP_ERROR(get_logger(), "行为树初始化失败!");
      rclcpp::shutdown();
      return;
    }

    // ── tick 循环 ────────────────────────────────────────────
    step_mode_ = get_parameter("step_mode").as_bool();
    if (!step_mode_) {
      double rate = get_parameter("tick_rate").as_double();
      RCLCPP_INFO(get_logger(), "等待 DDS discovery (300ms), 然后以 %.1f Hz 启动 tick", rate);
      startup_timer_ = create_wall_timer(300ms, [this]() {
        startup_timer_->cancel();
        double rate = get_parameter("tick_rate").as_double();
        tick_timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0 / rate),
          [this]() { performTick(); });
        RCLCPP_INFO(get_logger(), "行为树 tick 开始 (%.1f Hz)", rate);
      });
    } else {
      RCLCPP_INFO(get_logger(), "单步调试模式, 通过 /bt_step 控制");
    }
  }

private:
  // ═════════════════════════════════════════════════════════
  // 初始化行为树
  // ═════════════════════════════════════════════════════════
  bool initBehaviorTree()
  {
    uncoupling_robot::RegisterAllNodes(factory_, nullptr);

    std::string bt_xml_path = get_parameter("bt_xml_path").as_string();
    std::string full_path;
    for (const auto& p : {
        bt_xml_path,
        "/home/sz/colcon_ws/src/uncoupling_robot_bt/" + bt_xml_path,
        "/home/sz/workspace/src/uncoupling_robot_bt/" + bt_xml_path,
        "src/uncoupling_robot_bt/" + bt_xml_path,
    }) {
      if (std::filesystem::exists(p)) { full_path = p; break; }
    }
    if (full_path.empty()) {
      RCLCPP_ERROR(get_logger(), "找不到 BT XML: %s", bt_xml_path.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "加载: %s", full_path.c_str());

    try {
      factory_.registerBehaviorTreeFromFile(full_path);
      tree_ = std::make_unique<BT::Tree>(factory_.createTree("MainTree"));
      size_t n = 0;
      for (const auto& st : tree_->subtrees) n += st->nodes.size();
      RCLCPP_INFO(get_logger(), "行为树创建成功, %zu 个节点", n);
      BT::printTreeRecursively(tree_->rootNode());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "创建失败: %s", e.what());
      return false;
    }

    if (get_parameter("enable_groot").as_bool()) {
      int base = get_parameter("groot_port").as_int();
      for (int off = 0; off < 10; ++off) {
        try {
          groot_pub_ = std::make_unique<BT::Groot2Publisher>(*tree_, base + off);
          RCLCPP_INFO(get_logger(), "Groot2 端口=%d", base + off);
          break;
        } catch (...) { if (off == 9) RCLCPP_WARN(get_logger(), "Groot2 启动失败"); }
      }
    }
    return true;
  }

  // ═════════════════════════════════════════════════════════
  // Tick
  // ═════════════════════════════════════════════════════════
  void performTick()
  {
    if (!tree_) return;
    if (hmi_mode_in_bb_ == -1) return;  // HMI 手动模式

    // 首次 tick: ros_node + work_mode 注入黑板
    if (!ros_node_set_in_bb_) {
      auto self = shared_from_this();
      SET_BB_ALL(tree_, "ros_node", self);
      ros_node_set_in_bb_ = true;
    }
    if (!hmi_mode_injected_) {
      int m = (hmi_mode_in_bb_ >= 0) ? hmi_mode_in_bb_ : 1;
      SET_BB_ALL(tree_, "work_mode", m);
      hmi_mode_injected_ = true;
      RCLCPP_INFO(get_logger(), "[黑板] work_mode ← %d (%s)", m,
                  (hmi_mode_in_bb_ >= 0) ? "/ctrl_hmi" : "默认");
    }

    // ═════════════════════════════════════════════════════
    // ★ E_STOP 抢占式监测 (C++ 层)
    //    检测急停条件 → 暂停树而非 halt: tickOnce 跳过, 但 bridge 继续运行(发急停指令)
    //    进入 estop 有两种方式: ① checkEstopConditions() 返回 true  ② HMI mode=3
    //    HMI mode=4 解除后从原地继续执行
    // ═════════════════════════════════════════════════════
    if (!estop_active_ && checkEstopConditions()) {
      activateEstop("[条件触发]");
    }

    // ═════════════════════════════════════════════════════
    // ★ 先 Tick 再 Bridge: BT 节点写黑板 → 桥接立即发布，避免 +1 帧延迟
    // ═════════════════════════════════════════════════════
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    if (!estop_active_) {
      status = tree_->tickOnce();
      if (status == BT::NodeStatus::SUCCESS)
        RCLCPP_INFO(get_logger(), "任务成功完成!");
      else if (status == BT::NodeStatus::FAILURE)
        RCLCPP_ERROR(get_logger(), "任务失败!");
    }

    // 黑板 → 话题桥接: BT 叶子节点写黑板标记 → 这里检测变化 → publish

    // ── /co_arm_cmd ──
    bridgeInt32Pair("co_arm_cmd_reg", "co_arm_cmd_val",
                    last_co_arm_reg_, last_co_arm_val_, co_arm_cmd_pub_);

    // ── /arm_cmd ──
    bridgeInt32Pair("arm_cmd_reg", "arm_cmd_val",
                    last_arm_reg_, last_arm_val_, arm_cmd_pub_);

    // ── /speed_command ──
    bridgeSpeedCommand();

    // ── /cips_cmd ──
    bridgeInt32Pair("cips_cmd_reg", "cips_cmd_val",
                    last_cips_reg_, last_cips_val_, cips_cmd_pub_);
  }

  // ═════════════════════════════════════════════════════════
  // 黑板→话题桥接辅助
  // ═════════════════════════════════════════════════════════

  // 通用: 读取 reg_key + val_key, 变化时发布 Int32MultiArray[reg, val]
  // ★ 修复: getAnyLocked 的 shared_lock 必须在 SET_BB_ALL 的写锁之前释放
  template<typename PubT>
  void bridgeInt32Pair(const char* reg_key, const char* val_key,
                       int32_t& last_reg, int32_t& last_val, PubT& pub)
  {
    int32_t r = -1, v = -1;
    { auto a = tree_->rootBlackboard()->getAnyLocked(reg_key); if (a) try { r = a->cast<int32_t>(); } catch (...) {} }
    { auto b = tree_->rootBlackboard()->getAnyLocked(val_key); if (b) try { v = b->cast<int32_t>(); } catch (...) {} }
    if (r >= 0 && v >= 0 && (r != last_reg || v != last_val)) {
      auto msg = std_msgs::msg::Int32MultiArray();
      msg.data = {r, v};
      pub->publish(msg);
      last_reg = r; last_val = v;
      SET_BB_ALL(tree_, reg_key, -1);
      SET_BB_ALL(tree_, val_key, -1);
    }
  }

  // SpeedCommand 桥接: 黑板 → /speed_command (非 SpeedControlCommand 节点用)
  void bridgeSpeedCommand()
  {
    auto readI32 = [this](const char* k, int32_t def = -1) {
      auto a = tree_->rootBlackboard()->getAnyLocked(k);
      if (a) try { return a->cast< int32_t >(); } catch (...) {}
      return def;
    };
    int32_t gare = readI32("speed_cmd_gare");
    if (gare < 0) return;

    auto msg = data_interfaces::msg::SpeedCommand();
    msg.target_gare      = static_cast<uint8_t>(gare);
    msg.ctrl_mode        = static_cast<uint8_t>(readI32("speed_cmd_mode"));
    msg.target_id        = static_cast<uint8_t>(readI32("speed_cmd_target_id", 0));
    msg.position         = [&]{ auto a=tree_->rootBlackboard()->getAnyLocked("speed_cmd_position"); if(a)try{return static_cast<float>(a->cast<double>());}catch(...){} return 0.7f; }();
    msg.max_forward_vel  = [&]{ auto a=tree_->rootBlackboard()->getAnyLocked("speed_cmd_fwd_vel"); if(a)try{return static_cast<float>(a->cast<double>());}catch(...){} return 0.0f; }();
    msg.max_backward_vel = [&]{ auto a=tree_->rootBlackboard()->getAnyLocked("speed_cmd_bwd_vel"); if(a)try{return static_cast<float>(a->cast<double>());}catch(...){} return 0.0f; }();
    msg.max_acc          = [&]{ auto a=tree_->rootBlackboard()->getAnyLocked("speed_cmd_acc"); if(a)try{return static_cast<float>(a->cast<double>());}catch(...){} return 0.0f; }();
    msg.max_dec          = [&]{ auto a=tree_->rootBlackboard()->getAnyLocked("speed_cmd_dec"); if(a)try{return static_cast<float>(a->cast<double>());}catch(...){} return 0.0f; }();
    msg.coupling_count   = static_cast<uint8_t>(readI32("speed_cmd_coupling_count", 0));
    speed_cmd_pub_->publish(msg);
    SET_BB_ALL(tree_, "speed_cmd_gare", -1);
  }

  // ═════════════════════════════════════════════════════════
  // E_STOP 急停检测
  // ═════════════════════════════════════════════════════════

  // 急停条件检测: 逐一检查各急停信号, 任一触发即返回 true
  //   ① /ctrl_mag_stop (std_msgs/Bool) — 磁导航脱线/异常
  //   ② /ctrl_fence    (std_msgs/UInt8) — 电子围栏触发
  //   ③ estop_heartbeat                 — 心跳丢失
  bool checkEstopConditions()
  {
    if (!tree_) return false;

    // ① /ctrl_mag_stop → ctrl_mag_stop (bool)
    {
      auto a = tree_->rootBlackboard()->getAnyLocked("ctrl_mag_stop");
      if (a) try { if (a->cast<bool>()) { RCLCPP_WARN(get_logger(), "[E_STOP] ctrl_mag_stop=1"); return true; } } catch (...) {}
    }

    // // ② /ctrl_fence → ctrl_fence (int: 1=触发)
    // {
    //   auto b = tree_->rootBlackboard()->getAnyLocked("ctrl_fence");
    //   if (b) try { if (b->cast<int>() == 1) { RCLCPP_WARN(get_logger(), "[E_STOP] ctrl_fence=1"); return true; } } catch (...) {}
    // }

    // // ③ estop_heartbeat (int: 1=丢失)
    // {
    //   auto c = tree_->rootBlackboard()->getAnyLocked("estop_heartbeat");
    //   if (c) try { if (c->cast<int>() == 1) { RCLCPP_WARN(get_logger(), "[E_STOP] heartbeat丢失"); return true; } } catch (...) {}
    // }

    return false;
  }

  // 激活急停: 暂停树 tick, 写入刹车指令到黑板 (bridge 会发布到 /speed_command)
  void activateEstop(const char* source)
  {
    RCLCPP_ERROR(get_logger(), "[E_STOP] %s 抢占式急停! 暂停行为树, 等待 HMI mode=4 复位...", source);
    estop_active_ = true;
    SET_BB_ALL(tree_, "speed_cmd_gare", 1);         // gear=P
    SET_BB_ALL(tree_, "speed_cmd_mode", 0);          // 刹车
    SET_BB_ALL(tree_, "speed_cmd_fwd_vel", 0.0);
    SET_BB_ALL(tree_, "speed_cmd_bwd_vel", 0.0);
    SET_BB_ALL(tree_, "speed_cmd_acc", 0.0);
    SET_BB_ALL(tree_, "speed_cmd_dec", 0.0);
    SET_BB_ALL(tree_, "speed_cmd_coupling_count", 0);
  }

  // ═════════════════════════════════════════════════════════
  // HMI 控制
  // ═════════════════════════════════════════════════════════
  void handleHmiCommand(const data_interfaces::msg::HmiCommand::SharedPtr msg)
  {
    uint8_t mode = msg->hmi_ctrl_mode;
    RCLCPP_INFO(get_logger(), "[HMI] mode=%d gear=%d speed=%.2f brake=%d",
                mode, msg->ctrl_gare, msg->ctrl_speed, msg->ctrl_break);

    switch (mode) {
      case 0:  // 手动停止
        RCLCPP_WARN(get_logger(), "[HMI] 手动控制 → 停止行为树");
        if (tree_) tree_->haltTree();
        if (tick_timer_) tick_timer_->cancel();
        hmi_mode_in_bb_ = -1;
        break;
      case 1:  // 动态
      case 2:  // 静态
        RCLCPP_INFO(get_logger(), "[HMI] 切换: %s (work_mode=%d)",
                    mode == 1 ? "动态摘钩" : "静态摘钩", mode == 1 ? 1 : 0);
        if (hmi_mode_in_bb_ == -1 || !tree_) {
          tree_ = std::make_unique<BT::Tree>(factory_.createTree("MainTree"));
          ros_node_set_in_bb_ = false;
          hmi_mode_injected_ = false;
          recreateGrootPublisher();
          double rate = get_parameter("tick_rate").as_double();
          tick_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            [this]() { performTick(); });
          RCLCPP_INFO(get_logger(), "[HMI] 行为树已重建, tick 恢复");
        }
        SET_BB_ALL(tree_, "work_mode", (mode == 1 ? 1 : 0));
        hmi_mode_in_bb_ = (mode == 1 ? 1 : 0);
        break;
      case 3:  // 进入 ESTOP → 暂停树 (从原地暂停, 等待 mode=4 解除)
        RCLCPP_ERROR(get_logger(), "[HMI] mode=3 进入 ESTOP → 暂停行为树");
        if (tree_ && !estop_active_) activateEstop("[HMI mode=3]");
        break;
      case 4:  // 解除 ESTOP → 恢复树 (从暂停处继续)
        RCLCPP_INFO(get_logger(), "[HMI] mode=4 解除 ESTOP → 恢复行为树 tick");
        estop_active_ = false;
        if (tree_) SET_BB_ALL(tree_, "estop_heartbeat", 0);
        break;
    }
  }

  void recreateGrootPublisher()
  {
    if (!get_parameter("enable_groot").as_bool() || !tree_) return;
    int base = get_parameter("groot_port").as_int();
    groot_pub_.reset();
    for (int off = 0; off < 10; ++off) {
      try {
        groot_pub_ = std::make_unique<BT::Groot2Publisher>(*tree_, base + off);
        RCLCPP_INFO(get_logger(), "Groot2 已重建, 端口=%d", base + off);
        return;
      } catch (...) {}
    }
  }

  // ═════════════════════════════════════════════════════════
  // ═════════════════════════════════════════════════════════
  // HMI 状态 (10Hz)
  // ═════════════════════════════════════════════════════════
  void publishHmiState()
  {
    auto msg = data_interfaces::msg::BtHmiState();

    // ── E_STOP 激活时直接覆盖 (树已暂停 tick, 不会有 RUNNING 节点) ──
    if (estop_active_) {
      msg.active_subtree = "E_STOP";
      msg.active_leaf_node = "";
      msg.system_mode = 3;
      msg.remaining_tasks = -1;
      msg.detect_id = -1;
      hmi_state_pub_->publish(msg);
      return;
    }

    std::string subtree = "IDLE";
    std::string leaf;
    if (tree_) {
      size_t max_depth = 0;
      for (const auto& st : tree_->subtrees) {
        for (const auto& node : st->nodes) {
          if (node->status() == BT::NodeStatus::RUNNING) {
            std::string path = node->fullPath();
            if (path.size() > max_depth) {
              max_depth = path.size();
              subtree = resolveSubtreeFromPath(path);
              leaf = node->registrationName();
            }
          }
        }
      }
    }
    msg.active_subtree = subtree;
    msg.active_leaf_node = leaf;

    uint8_t mode = 1;
    if (subtree == "E_STOP") mode = 3;
    else if (hmi_mode_in_bb_ == -1) mode = 0;
    else {
      int wm = 1;
      if (tree_) {
        auto a = tree_->rootBlackboard()->getAnyLocked("work_mode");
        if (a) try { wm = a->cast<int>(); } catch (...) {}
      }
      mode = (wm == 0) ? 2 : 1;
    }
    msg.system_mode = mode;

    msg.remaining_tasks = -1;
    msg.detect_id = -1;
    if (tree_) {
      auto a = tree_->rootBlackboard()->getAnyLocked("remaining_tasks");
      if (a) try { msg.remaining_tasks = a->cast<int>(); } catch (...) {}
      auto b = tree_->rootBlackboard()->getAnyLocked("detect_id");
      if (b) try { msg.detect_id = b->cast<int>(); } catch (...) {}
    }
    hmi_state_pub_->publish(msg);
  }

  // 从 fullPath 中匹配实际的 BehaviorTree ID (动态读取 tree_->subtrees, 无需手动维护列表)
  std::string resolveSubtreeFromPath(const std::string& path) const
  {
    if (path.find("estop_actions") != std::string::npos) return "E_STOP";

    // 从 tree_->subtrees 动态获取所有 BehaviorTree ID
    // 按长度降序匹配, 避免 "S1_IDLE" 误匹配 "S11_STATIC_ALIGN" 中的 "S1"
    std::vector<std::string> ids;
    for (const auto& st : tree_->subtrees) {
      if (!st->tree_ID.empty() && st->tree_ID != "MainTree")
        ids.push_back(st->tree_ID);
    }
    std::sort(ids.begin(), ids.end(),
              [](const std::string& a, const std::string& b) { return a.size() > b.size(); });

    for (const auto& id : ids)
      if (path.find(id) != std::string::npos) return id;
    return "IDLE";
  }

  // ═════════════════════════════════════════════════════════
  // 成员
  // ═════════════════════════════════════════════════════════
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<BT::Groot2Publisher> groot_pub_;

  // 发布
  rclcpp::Publisher<data_interfaces::msg::BtHmiState>::SharedPtr hmi_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr co_arm_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<data_interfaces::msg::SpeedCommand>::SharedPtr speed_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cips_cmd_pub_;
  rclcpp::TimerBase::SharedPtr hmi_state_timer_;

  // 订阅
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr co_arm_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr arm_status_sub_;
  rclcpp::Subscription<data_interfaces::msg::HmiCommand>::SharedPtr hmi_command_sub_;
  rclcpp::Subscription<data_interfaces::msg::DetectResult>::SharedPtr detect_result_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ctrl_mag_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ctrl_fence_sub_;
  rclcpp::Subscription<data_interfaces::msg::HookTaskArray>::SharedPtr cips_task_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr cips_status_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr speed_state_sub_;

  // 桥接状态缓存
  int32_t last_co_arm_reg_ = -1, last_co_arm_val_ = -1;
  int32_t last_arm_reg_ = -1, last_arm_val_ = -1;
  int32_t last_cips_reg_ = -1, last_cips_val_ = -1;

  // HMI 控制
  int hmi_mode_in_bb_ = 1;
  bool hmi_mode_injected_ = false;
  bool ros_node_set_in_bb_ = false;

  // E_STOP 抢占
  bool estop_active_ = false;

  // Timer
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  bool step_mode_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtExecutorNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
