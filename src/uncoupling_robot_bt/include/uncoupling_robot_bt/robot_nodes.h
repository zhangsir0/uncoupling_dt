#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "data_interfaces/msg/speed_command.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <optional>
#include <string>

namespace uncoupling_robot
{

// ================================================================
// Mock 注入: 检查节点是否被 Qt GUI 工具 mock
// bt_executor_node 通过 /bt/mock/set 话题接收 "NodeName:STATUS",
// 写入 blackboard: mock_<NodeName> = int (1=RUNNING, 2=SUCCESS, 3=FAILURE)
// ================================================================
std::optional<BT::NodeStatus> checkMockStatus(const std::string& node_name,
                                               BT::Blackboard::Ptr bb);
#define MOCK_CHECK_NODE() \
  do { \
    auto __mock = ::uncoupling_robot::checkMockStatus(name(), config().blackboard); \
    if (__mock) return __mock.value(); \
  } while(0)

// ================================================================
// 黑板读写辅助 (所有 ROS2 I/O 集中在 bt_executor_node)
//
// BT 节点通过黑板读写与外界交互:
//   InputPort  → 从黑板读取 (bt_executor 订阅回调写入)
//   OutputPort → 写入黑板   (bt_executor performTick 轮询发布)
//   标记位     → 写入黑板 flag (bt_executor 检测变化后发布并清除)
//
// 标记位模式 (用于指令下发: /co_arm_cmd, /arm_cmd, /speed_command, /cips_cmd):
//   BT节点: bb->set("xxx_reg", addr); bb->set("xxx_val", value);
//   bt_executor: 检测变化 → publish → 清除标记位
// ================================================================

// ╔══════════════════════════════════════════════════════════════╗
// ║                   S0: 上电自检                               ║
// ╚══════════════════════════════════════════════════════════════╝

class CheckCommunication : public BT::SyncActionNode
{
public:
  CheckCommunication(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class CheckSensors : public BT::SyncActionNode
{
public:
  CheckSensors(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class CheckActuators : public BT::SyncActionNode
{
public:
  CheckActuators(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║                   S1: 空闲待命                                ║
// ╚══════════════════════════════════════════════════════════════╝

class IsBatteryNormal : public BT::SyncActionNode
{
public:
  IsBatteryNormal(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class WaitBatteryRecovery : public BT::StatefulActionNode
{
public:
  WaitBatteryRecovery(const std::string& n, const BT::NodeConfig& c) : StatefulActionNode(n,c) {}
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}
  static BT::PortsList providedPorts() { return {}; }
};

// 等待 CIPS 任务: 阻塞等待 blackboard("cips_40001") == 1
// 收到触发信号后读取 cips_task_array → 写黑板 → 返回 SUCCESS
class WaitForCIPSTask : public BT::StatefulActionNode
{
public:
  WaitForCIPSTask(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<int>("timeout", 0, "超时ms") }; }
private:
  int timeout_ms_ = 0;
  rclcpp::Time start_time_;
};

// ╔══════════════════════════════════════════════════════════════╗
// ║                S2: 双臂复位校验                                ║
// ╚══════════════════════════════════════════════════════════════╝

class IsArmStowed : public BT::SyncActionNode
{
public:
  IsArmStowed(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<std::string>("arm", "left", "left/right") }; }
};

class IsCoArmHome : public BT::SyncActionNode
{
public:
  IsCoArmHome(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class IsArmHome : public BT::SyncActionNode
{
public:
  IsArmHome(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class CheckCoSpeedArm : public BT::SyncActionNode
{
public:
  CheckCoSpeedArm(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<std::string>("command", "RESET", "RESET/DEPLOY/ATTACH") }; }
};

class CheckMechanicalArm : public BT::SyncActionNode
{
public:
  CheckMechanicalArm(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<std::string>("command", "RESET", "RESET/LIFT/START_WORK") }; }
};

// 阻塞等待共速臂到位 (替代 RetryUntilSuccessful + CheckCoSpeedArm)
class WaitCoSpeedArm : public BT::StatefulActionNode
{
public:
  WaitCoSpeedArm(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return {
      BT::InputPort<std::string>("command", "RESET", "RESET/DEPLOY/ATTACH"),
      BT::InputPort<double>("timeout", -1.0, "超时秒数, -1=无限"),
  };}
private:
  std::string command_;
  double timeout_ = -1.0;
  rclcpp::Time start_time_;
};

// 阻塞等待机械臂到位 (替代 RetryUntilSuccessful + CheckMechanicalArm)
class WaitMechanicalArm : public BT::StatefulActionNode
{
public:
  WaitMechanicalArm(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return {
      BT::InputPort<std::string>("command", "RESET", "RESET/HOME/ISOK"),
      BT::InputPort<double>("timeout", -1.0, "超时秒数, -1=无限"),
  };}
private:
  std::string command_;
  double timeout_ = -1.0;
  rclcpp::Time start_time_;
};

// ╔══════════════════════════════════════════════════════════════╗
// ║                S3: 磁导航循迹                                 ║
// ╚══════════════════════════════════════════════════════════════╝

class MagneticGuideOnline : public BT::SyncActionNode
{
public:
  MagneticGuideOnline(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class NavigateToWaitArea : public BT::StatefulActionNode
{
public:
  NavigateToWaitArea(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return {
      BT::InputPort<double>("speed", 1.0, "行进速度 m/s"),
      BT::InputPort<std::string>("mode", "magnetic_guide", "导航模式")
  };}
private:
  double speed_ = 1.0;
  std::string mode_;
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            S4: 占位等待 — 雷达间隙计数                         ║
// ╚══════════════════════════════════════════════════════════════╝

// 等待目标车厢间隙 → 与 LockTargetGap 合并
// 从 cips_task_array[task_round] 获取 train_id 作为 target_gap_id
// detect_id == target_gap_id → SUCCESS, 否则 RUNNING
// ★ ActionNodeBase: tick() 直接调用, 不用 onStart/onRunning (StatefulActionNode 生命周期失败)
class WaitForTargetGap : public BT::StatefulActionNode
{
public:
  WaitForTargetGap(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<int>("timeout", 0, "超时ms") }; }
  // static BT::PortsList providedPorts()
  // { return {
  //     BT::OutputPort<int>("gap_id", "{target_gap_id}", "锁定间隙ID(=train_id)")
  // };}
private:

};

// ── 任务信息提取: cips_task_array[task_round] → 黑板各变量 ──
class ExtractTaskInfo : public BT::SyncActionNode
{
public:
  ExtractTaskInfo(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║                S5: MPC 动态追车                                ║
// ╚══════════════════════════════════════════════════════════════╝

class GenerateInterceptTrajectory : public BT::SyncActionNode
{
public:
  GenerateInterceptTrajectory(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class MPCTrackApproach : public BT::StatefulActionNode
{
public:
  MPCTrackApproach(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<double>("timeout", 30.0, "超时(秒)") }; }
private:
  rclcpp::Time start_time_;
  double timeout_ = 30.0;
  int consecutive_converged_ = 0;
};

class IsTrackingErrorWithin : public BT::SyncActionNode
{
public:
  IsTrackingErrorWithin(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<double>("tolerance", 0.05, "容许误差") }; }
};

class IsTargetInRange : public BT::SyncActionNode
{
public:
  IsTargetInRange(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            S6: 锁定相对静止                                    ║
// ╚══════════════════════════════════════════════════════════════╝

class SetMPCWeights : public BT::SyncActionNode
{
public:
  SetMPCWeights(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return {
      BT::InputPort<double>("speed_weight", 1.0, "w_v"),
      BT::InputPort<double>("position_weight", 1.0, "w_x")
  };}
};

class IsStabilized : public BT::SyncActionNode
{
public:
  IsStabilized(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<double>("timeout", 2000.0, "稳态计时器 ms") }; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            S7: 共速臂展开                                      ║
// ╚══════════════════════════════════════════════════════════════╝

// ★ 直接发布到 /co_arm_cmd, 不经过黑板桥接
// SET_CAR_TYPE 时从 cips_task_array[task_round].train_type_code 取值
class SendCoSpeedArmCommand : public BT::SyncActionNode
{
public:
  SendCoSpeedArmCommand(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return {
      BT::InputPort<std::string>("command", "DEPLOY", "RESET/DEPLOY/DEMAGNETIZE/SET_MODE/SET_CAR_TYPE")
  };}
private:
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
  bool pub_initialized_ = false;
};

class IsCoSpeedArmDeployed : public BT::SyncActionNode
{
public:
  IsCoSpeedArmDeployed(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            S8: 微减速贴附                                      ║
// ╚══════════════════════════════════════════════════════════════╝

class ApplyMicroDeceleration : public BT::SyncActionNode
{
public:
  ApplyMicroDeceleration(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<double>("decel_rate", 0.05, "减速率 m/s^2") }; }
};

class CheckAttachmentForce : public BT::SyncActionNode
{
public:
  CheckAttachmentForce(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<double>("threshold", 100.0, "贴附力阈值 N") }; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            S9: 摘钩作业                                        ║
// ╚══════════════════════════════════════════════════════════════╝

class SwitchToNeutralMode : public BT::SyncActionNode
{
public:
  SwitchToNeutralMode(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

// ★ 直接发布到 /arm_cmd, 不经过黑板桥接 (避免锁竞争)
class SendMechanicalArmCommand : public BT::SyncActionNode
{
public:
  SendMechanicalArmCommand(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<std::string>("command", "LIFT", "RESET/LIFT/START_WORK/STOP/SET_MODE") }; }
private:
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
  bool pub_initialized_ = false;
};

// ★ 直接发布到 /cips_cmd, RECEIVED→40001=2, REQUEST→40001=4
class SendCipsCommand : public BT::SyncActionNode
{
public:
  SendCipsCommand(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<std::string>("command", "RECEIVED", "RECEIVED/REQUEST") }; }
private:
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
  bool pub_initialized_ = false;
};

class IsUncoupled : public BT::SyncActionNode
{
public:
  IsUncoupled(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            S11: 任务分发                                       ║
// ╚══════════════════════════════════════════════════════════════╝

class CheckRemainingTasks : public BT::SyncActionNode
{
public:
  CheckRemainingTasks(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            S10/12: 返航                                        ║
// ╚══════════════════════════════════════════════════════════════╝

class NavigateToOrigin : public BT::StatefulActionNode
{
public:
  NavigateToOrigin(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<float>("nav_pos", 0,"导航位置") }; }
private:
  std::string mode_;
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            E_STOP: 全局急停                                    ║
// ╚══════════════════════════════════════════════════════════════╝

class IsHeartbeatLost : public BT::SyncActionNode
{
public:
  IsHeartbeatLost(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class IsAtPositionLimit : public BT::SyncActionNode
{
public:
  IsAtPositionLimit(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class IsCollisionRisk : public BT::SyncActionNode
{
public:
  IsCollisionRisk(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class EmergencyBrake : public BT::SyncActionNode
{
public:
  EmergencyBrake(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class CutPower : public BT::SyncActionNode
{
public:
  CutPower(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class ReportFault : public BT::SyncActionNode
{
public:
  ReportFault(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<std::string>("fault_code", "", "故障码") }; }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  bool pub_initialized_ = false;
};

class ReportInfo : public BT::SyncActionNode
{
public:
  ReportInfo(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<std::string>("info_code", "", "信息码") }; }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  bool pub_initialized_ = false;
};

// ── 辅助动作节点 (E_STOP / S1 流程) ──
class ReportEStop : public BT::SyncActionNode {
public: ReportEStop(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override; static BT::PortsList providedPorts() { return {}; }
};
class ReportTaskComplete : public BT::SyncActionNode {
public: ReportTaskComplete(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override; static BT::PortsList providedPorts() { return {}; }
};
class RejectIncomingTasks : public BT::SyncActionNode {
public: RejectIncomingTasks(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override; static BT::PortsList providedPorts() { return {}; }
};
class ParseTaskMessage : public BT::SyncActionNode {
public: ParseTaskMessage(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override; static BT::PortsList providedPorts() { return {}; }
};
class StartGapCounting : public BT::SyncActionNode {
public: StartGapCounting(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override; static BT::PortsList providedPorts() { return {}; }
};
class ResetTargetSpeed : public BT::SyncActionNode {
public: ResetTargetSpeed(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override; static BT::PortsList providedPorts() { return {}; }
};
class RetractCoSpeedArm : public BT::SyncActionNode {
public: RetractCoSpeedArm(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override; static BT::PortsList providedPorts() { return {}; }
};
class IsBeforeWork : public BT::SyncActionNode {
public: IsBeforeWork(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override; static BT::PortsList providedPorts() { return {}; }
};

class WaitForManualReset : public BT::CoroActionNode
{
public:
  WaitForManualReset(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus tick() override;
  void halt() override;
  static BT::PortsList providedPorts() { return {}; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            通用动作节点                                        ║
// ╚══════════════════════════════════════════════════════════════╝

// ★ 直接发布到 /speed_command (参考 SendCoSpeedArmCommand, 绕过黑板桥接)
class SpeedControlCommand : public BT::SyncActionNode
{
public:
  SpeedControlCommand(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return {
      BT::InputPort<int>("gear", 1, "档位 1=P,2=R,3=N,4=D"),
      BT::InputPort<int>("ctrl_mode", 0, "0=刹车 1=跟车 2=微减速 3=恢复 4=减速停车 10=定速到点"),
      BT::InputPort<double>("position", 0.7, "相对目标距离"),
      BT::InputPort<double>("max_forward_vel", 2.0, "最大前进速度 m/s"),
      BT::InputPort<double>("max_backward_vel", 0.5, "最大后退速度 m/s"),
      BT::InputPort<double>("max_acc", 0.5, "最大加速度 m/s^2"),
      BT::InputPort<double>("max_dec", 1.0, "最大减速度 m/s^2"),
  };}
private:
  rclcpp::Publisher<data_interfaces::msg::SpeedCommand>::SharedPtr pub_;
  bool pub_initialized_ = false;
};

// 等待 speed_state == command, 超时返回 FAILURE
class CheckCtrlStatus : public BT::StatefulActionNode
{
public:
  CheckCtrlStatus(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return {
      BT::InputPort<int>("command", 2, "期望 speed_state 值 0=停止 1=运动中 2=已对齐 3=完成"),
      BT::InputPort<int>("timeout", 10, "超时秒数"),
  };}
private:
  int command_ = 2;
  double timeout_ = 10.0;
  rclcpp::Time start_time_;
};

class ApplyParkingBrake : public BT::SyncActionNode
{
public:
  ApplyParkingBrake(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class ReleaseParkingBrake : public BT::SyncActionNode
{
public:
  ReleaseParkingBrake(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class SendArmCommand : public BT::SyncActionNode
{
public:
  SendArmCommand(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<std::string>("arm", "left", "left/right"),
             BT::InputPort<std::string>("command", "RESET", "指令") }; }
};

class WaitStabilization : public BT::SyncActionNode
{
public:
  WaitStabilization(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<int>("timeout", 2000, "等待时间 ms") }; }
};

// 纯等待节点: 阻塞指定时长后返回 SUCCESS, -1 表示永久等待
class WaitDelay : public BT::StatefulActionNode
{
public:
  WaitDelay(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<int>("delay_s", 1, "等待时长 s, -1=永久") }; }
private:
  rclcpp::Time start_time_;
  int delay_s_ = 1;
};

class WaitForUncoupleConfirm : public BT::SyncActionNode
{
public:
  WaitForUncoupleConfirm(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<int>("timeout", 5000, "等待时间 ms") }; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║         动/静态工作模式                                        ║
// ╚══════════════════════════════════════════════════════════════╝

class IsStaticMode : public BT::SyncActionNode
{
public:
  IsStaticMode(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

class IsDynamicMode : public BT::SyncActionNode
{
public:
  IsDynamicMode(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║        静态摘钩模式节点                                        ║
// ╚══════════════════════════════════════════════════════════════╝

class ComputeStaticTargetPose : public BT::SyncActionNode
{
public:
  ComputeStaticTargetPose(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<int>("target_gap_id", 0, "目标间隙编号") }; }
};

class NavigateToStaticTarget : public BT::StatefulActionNode
{
public:
  NavigateToStaticTarget(const std::string& n, const BT::NodeConfig& c);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts()
  { return {
      BT::InputPort<double>("speed", 1.0, "行进速度 m/s"),
      BT::InputPort<double>("timeout", 30.0, "超时(秒)")
  };}
private:
  double speed_ = 1.0;
  double timeout_ = 30.0;
  rclcpp::Time start_time_;
  int consecutive_converged_ = 0;
};

class IsAtTargetPose : public BT::SyncActionNode
{
public:
  IsAtTargetPose(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  { return { BT::InputPort<double>("tolerance", 0.05, "容差(m)") }; }
};

// 根据 detect_id 与 train_id 判断距离，写入 static_target_ctrl_mode 供下一个 SpeedControlCommand 使用
class CheckStaticTargetPose : public BT::SyncActionNode
{
public:
  CheckStaticTargetPose(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n,c) {}
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() 
  { return {
      BT::InputPort<double>("position", 0.8, "目标距离"),
  }; }
};

// ╔══════════════════════════════════════════════════════════════╗
// ║            注册                                               ║
// ╚══════════════════════════════════════════════════════════════╝
void RegisterAllNodes(BT::BehaviorTreeFactory& factory, rclcpp::Node::SharedPtr node);

}  // namespace uncoupling_robot
