/**
 * @file chassis_arbitration_node.hpp
 * @brief 底盘控制仲裁节点 — 汇总所有底盘控制指令来源，按四级优先级统一仲裁
 *
 * 仲裁优先级: L1(安全) > L2(HMI手动) > L3(行为树) > L4(运动指令)
 * 三阶段执行模型: 阶段一(L1预检查) → 阶段二(L4→L3→L2级联) → 阶段三(L1后处理+裁剪)
 */

#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "yhs_can_interfaces/msg/auto_spd_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"

#include "data_interfaces/msg/spd_ctrl_cmd.hpp"
#include "data_interfaces/msg/speed_command.hpp"
#include "data_interfaces/msg/hmi_command.hpp"
#include "data_interfaces/msg/arbitration_state.hpp"
#include "data_interfaces/msg/ctrl_speed.hpp"


namespace chassis_arbitration
{

/**
 * @brief 仲裁节点状态机
 */
enum class ArbitrationStateMachine
{
  INIT = 0,        ///< 初始化，等待订阅就绪
  IDLE,            ///< 空闲，L4 两路均超时且无 HMI 输入
  ARBITRATING,     ///< 正常仲裁运行
  ESTOP,           ///< 紧急停车
  RECOVER,         ///< ESTOP 恢复，刹车渐进释放
};

/**
 * @brief 阶段二中传递的仲裁中间结果
 */
struct ArbitrationResult
{
  uint8_t  gear     = 4;    ///< 档位 1=P, 2=R, 3=N, 4=D
  float    velocity = 0.0f; ///< 速度 m/s (恒 ≥ 0)
  float    steering = 0.0f; ///< 转向角 度
  uint8_t  brake    = 0;    ///< 刹车 0-100

  uint8_t  active_level  = 0;  ///< 当前生效仲裁等级
  std::string active_source;   ///< 生效来源描述
  uint8_t  l4_source      = 0; ///< L4 数据源: 0=无, 1=speed_track, 2=position_nav
};

/**
 * @brief 底盘控制仲裁节点
 *
 * 订阅 8 个输入话题，执行三阶段仲裁，产出唯一的 /auto_spd_ctrl_cmd 输出。
 * 所有回调仅缓存消息和时间戳，仲裁逻辑在 50Hz 定时器线程中单线程执行，
 * 避免多源回调竞争。
 */
class ChassisArbitrationNode : public rclcpp::Node
{
public:
  ChassisArbitrationNode();
  ~ChassisArbitrationNode() override = default;

private:
  // ══════════════════════════════════════════════════════════════
  //  订阅回调（仅缓存消息 + 时间戳）
  // ══════════════════════════════════════════════════════════════

  void onCtrlSpeed(const data_interfaces::msg::CtrlSpeed::SharedPtr msg);
  void onCtrlPosition(const data_interfaces::msg::CtrlSpeed::SharedPtr msg);
  void onSpeedCommand(const data_interfaces::msg::SpeedCommand::SharedPtr msg);
  void onCtrlHmi(const data_interfaces::msg::HmiCommand::SharedPtr msg);
  void onCtrlFence(const std_msgs::msg::UInt8::SharedPtr msg);
  void onCtrlMagSteer(const std_msgs::msg::Float32::SharedPtr msg);
  void onCtrlMagStop(const std_msgs::msg::Bool::SharedPtr msg);
  void onChassisInfoFb(const yhs_can_interfaces::msg::ChassisInfoFb::SharedPtr msg);

  // ══════════════════════════════════════════════════════════════
  //  定时器回调
  // ══════════════════════════════════════════════════════════════

  /// @brief 50Hz 主仲裁循环
  void onArbitrationTick();

  /// @brief 10Hz 状态发布
  void onStatePublishTick();

  // ══════════════════════════════════════════════════════════════
  //  阶段一: L1 安全预检查
  // ══════════════════════════════════════════════════════════════

  /// @brief 检查是否触发 ESTOP 条件，触发则直接发布 ESTOP 命令
  /// @return true=已触发 ESTOP（本周期结束），false=安全，继续阶段二
  bool checkAndHandleEstop();

  /// @brief 发布 ESTOP 命令: gear=P, vel=0, brake=100, steering=mag或0
  void publishEstopCommand();

  // ══════════════════════════════════════════════════════════════
  //  阶段二: L4 → L3 → L2 级联构建
  // ══════════════════════════════════════════════════════════════

  /// @brief 阶段二入口：级联构建并返回中间结果
  ArbitrationResult buildCascadeResult();

  /// @brief L4 基础层：源选择 + 速度符号转换
  ArbitrationResult buildL4Base();

  /// @brief L3 行为树档位覆盖（仅 hmi_ctrl_mode ≠ 0 时生效）
  void applyL3Override(ArbitrationResult & result);

  /// @brief L2 HMI 手动接管（仅 hmi_ctrl_mode == 0 时生效）
  void applyL2Override(ArbitrationResult & result);

  // ══════════════════════════════════════════════════════════════
  //  阶段三: L1 安全后处理 + 边界裁剪
  // ══════════════════════════════════════════════════════════════

  /// @brief L1 安全后处理: fence 限速 + mag_steer 覆盖 + 速度裁剪 + 最终钳位
  void applyL1PostProcess(ArbitrationResult & result);

  // ══════════════════════════════════════════════════════════════
  //  状态机
  // ══════════════════════════════════════════════════════════════

  /// @brief 更新状态机
  void updateStateMachine(const ArbitrationResult & result);

  /// @brief 检查 ESTOP 恢复条件是否满足
  bool checkRecoverConditions();

  /// @brief RECOVER 状态：渐进释放刹车
  ArbitrationResult stepRecover();

  // ══════════════════════════════════════════════════════════════
  //  输出
  // ══════════════════════════════════════════════════════════════

  /// @brief 发布 /auto_spd_ctrl_cmd
  void publishCommand(const ArbitrationResult & result);

  // ══════════════════════════════════════════════════════════════
  //  工具函数
  // ══════════════════════════════════════════════════════════════

  /// @brief 检查消息是否超时
  bool isTimeout(const rclcpp::Time & stamp, int timeout_ms) const;

  /// @brief 检查是否从未收到过消息
  bool isNeverReceived(const rclcpp::Time & stamp) const;

  /// @brief 钳位值到 [lo, hi]
  static float clamp(float val, float lo, float hi);
  static uint8_t clampU8(uint8_t val, uint8_t lo, uint8_t hi);

  // ══════════════════════════════════════════════════════════════
  //  消息缓存 (带时间戳)
  // ══════════════════════════════════════════════════════════════

  // L4 输入
  data_interfaces::msg::CtrlSpeed::SharedPtr ctrl_speed_msg_;
  rclcpp::Time ctrl_speed_stamp_{0, 0, RCL_ROS_TIME};

  data_interfaces::msg::CtrlSpeed::SharedPtr ctrl_position_msg_;
  rclcpp::Time ctrl_position_stamp_{0, 0, RCL_ROS_TIME};

  // L3 输入
  data_interfaces::msg::SpeedCommand::SharedPtr speed_command_msg_;
  rclcpp::Time speed_command_stamp_{0, 0, RCL_ROS_TIME};

  // L2 输入
  data_interfaces::msg::HmiCommand::SharedPtr ctrl_hmi_msg_;
  rclcpp::Time ctrl_hmi_stamp_{0, 0, RCL_ROS_TIME};

  // L1 输入
  std_msgs::msg::UInt8::SharedPtr ctrl_fence_msg_;
  rclcpp::Time ctrl_fence_stamp_{0, 0, RCL_ROS_TIME};

  std_msgs::msg::Float32::SharedPtr ctrl_mag_steer_msg_;
  rclcpp::Time ctrl_mag_steer_stamp_{0, 0, RCL_ROS_TIME};

  std_msgs::msg::Bool::SharedPtr ctrl_mag_stop_msg_;
  rclcpp::Time ctrl_mag_stop_stamp_{0, 0, RCL_ROS_TIME};

  // 诊断输入
  yhs_can_interfaces::msg::ChassisInfoFb::SharedPtr chassis_info_fb_msg_;
  rclcpp::Time chassis_info_fb_stamp_{0, 0, RCL_ROS_TIME};

  // ══════════════════════════════════════════════════════════════
  //  发布器
  // ══════════════════════════════════════════════════════════════

  rclcpp::Publisher<yhs_can_interfaces::msg::AutoSpdCtrlCmd>::SharedPtr auto_spd_pub_;
  rclcpp::Publisher<data_interfaces::msg::ArbitrationState>::SharedPtr arbitration_state_pub_;

  // ══════════════════════════════════════════════════════════════
  //  订阅器
  // ══════════════════════════════════════════════════════════════

  rclcpp::Subscription<data_interfaces::msg::CtrlSpeed>::SharedPtr ctrl_speed_sub_;
  rclcpp::Subscription<data_interfaces::msg::CtrlSpeed>::SharedPtr ctrl_position_sub_;
  rclcpp::Subscription<data_interfaces::msg::SpeedCommand>::SharedPtr speed_command_sub_;
  rclcpp::Subscription<data_interfaces::msg::HmiCommand>::SharedPtr ctrl_hmi_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ctrl_fence_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ctrl_mag_steer_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ctrl_mag_stop_sub_;
  rclcpp::Subscription<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr chassis_info_fb_sub_;

  // ══════════════════════════════════════════════════════════════
  //  定时器
  // ══════════════════════════════════════════════════════════════

  rclcpp::TimerBase::SharedPtr arbitration_timer_;
  rclcpp::TimerBase::SharedPtr state_publish_timer_;

  // ══════════════════════════════════════════════════════════════
  //  状态机
  // ══════════════════════════════════════════════════════════════

  ArbitrationStateMachine state_{ArbitrationStateMachine::INIT};
  int recovery_brake_{100}; ///< RECOVER 状态刹车值，从 100 递减到 0
  int brake_step_{5};       ///< 每周期刹车减量

  /// @brief 最近一次仲裁结果，供状态发布使用
  ArbitrationResult last_result_;

  // ══════════════════════════════════════════════════════════════
  //  参数
  // ══════════════════════════════════════════════════════════════

  // 超时参数 (ms)
  int timeout_ctrl_speed_ms_{200};
  int timeout_ctrl_position_ms_{200};
  int timeout_speed_command_ms_{500};
  int timeout_ctrl_hmi_ms_{500};
  int timeout_ctrl_fence_ms_{300};
  int timeout_ctrl_mag_steer_ms_{300};
  int timeout_ctrl_mag_stop_ms_{300};

  // 安全参数
  bool enable_fence_{false};            ///< 是否启用 /ctrl_fence 围栏功能
  bool magnetic_guide_enabled_{true};   ///< 是否启用磁导航（未铺设磁导线时设为 false）
  float fence_decelerate_velocity_{0.3f};
  float max_velocity_{2.0f};
  float max_steering_angle_{30.0f};

  // 频率参数
  double arbitration_rate_{50.0};
  double state_publish_rate_{10.0};

  // L4 源选择
  int l4_position_nav_threshold_{20};

  // ══════════════════════════════════════════════════════════════
  //  看门狗
  // ══════════════════════════════════════════════════════════════

  int watchdog_counter_{0};
  static constexpr int WATCHDOG_LIMIT = 250; ///< 5s @ 50Hz 无回调触发重启
};

}  // namespace chassis_arbitration
