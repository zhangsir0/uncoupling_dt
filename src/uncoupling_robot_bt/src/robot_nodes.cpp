#include "uncoupling_robot_bt/robot_nodes.h"
#include "data_interfaces/msg/hook_task_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

namespace uncoupling_robot
{

// ================================================================
// Mock 注入
// ================================================================
std::optional<BT::NodeStatus> checkMockStatus(const std::string& node_name,
                                               BT::Blackboard::Ptr bb)
{
  std::string key = "mock_" + node_name;
  auto any = bb->getAnyLocked(key);
  if (any) {
    try { int val = any->cast<int>(); if (val >= 1 && val <= 3) return static_cast<BT::NodeStatus>(val); }
    catch (...) {}
  }
  return std::nullopt;
}

// ================================================================
// ★ 安全黑板读写 — 避免 getAnyLocked + set 死锁
//
// 原理: BehaviorTree.CPP 的 Blackboard 使用全局 shared_mutex.
//       getAnyLocked() 获取 shared_lock(读锁),
//       bb->set() / setOutput() 获取 unique_lock(写锁).
//       如果在读锁释放前尝试获取写锁 → 死锁!
//
// 解决: 所有读取放在内层 {} 中, 提取值后立即释放锁, 再写入.
//       bbRead<T>() 封装此模式.
// ================================================================
template<typename T>
static T bbRead(BT::Blackboard::Ptr bb, const std::string& key, T def = T{})
{
  T v = def;
  { auto a = bb->getAnyLocked(key); if (a) try { v = a->cast<T>(); } catch (...) {} }
  return v;
}
// 常用类型的快捷版
static int bbReadInt(BT::Blackboard::Ptr bb, const std::string& key, int def = -1)
{ return bbRead<int>(bb, key, def); }
static double bbReadDouble(BT::Blackboard::Ptr bb, const std::string& key, double def = 0.0)
{ return bbRead<double>(bb, key, def); }

// ================================================================
// 辅助: 写黑板标记 → bt_executor 轮询发布
// ================================================================
static void setBBCmd(BT::Blackboard::Ptr bb, const std::string& reg_key, int32_t reg,
                     const std::string& val_key, int32_t val)
{
  bb->set(reg_key, reg);
  bb->set(val_key, val);
}

// 写 SpeedCommand 标记
static void setBBSpeedCmd(BT::Blackboard::Ptr bb,
                          int gare, int mode, int target_id, float position,
                          float fwd_vel, float bwd_vel, float acc, float dec,
                          int coupling_count = 0)
{
  bb->set("speed_cmd_gare",           gare);
  bb->set("speed_cmd_mode",           mode);
  bb->set("speed_cmd_target_id",      target_id);
  bb->set("speed_cmd_position",       static_cast<double>(position));
  bb->set("speed_cmd_fwd_vel",        static_cast<double>(fwd_vel));
  bb->set("speed_cmd_bwd_vel",        static_cast<double>(bwd_vel));
  bb->set("speed_cmd_acc",            static_cast<double>(acc));
  bb->set("speed_cmd_dec",            static_cast<double>(dec));
  bb->set("speed_cmd_coupling_count", coupling_count);
}

// ================================================================
// S0: 上电自检
// ================================================================
BT::NodeStatus CheckCommunication::tick()
{
  MOCK_CHECK_NODE();
  RCLCPP_INFO(rclcpp::get_logger("bt"), "通信链路自检通过");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckSensors::tick()
{
  MOCK_CHECK_NODE();
  RCLCPP_INFO(rclcpp::get_logger("bt"), "传感器状态正常");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckActuators::tick()
{
  MOCK_CHECK_NODE();
  RCLCPP_INFO(rclcpp::get_logger("bt"), "执行器状态正常");
  return BT::NodeStatus::SUCCESS;
}

// ================================================================
// S1: 空闲待命
// ================================================================
namespace { double g_mock_battery_soc = 30.0; }

BT::NodeStatus IsBatteryNormal::tick()
{
  MOCK_CHECK_NODE();
  double soc = g_mock_battery_soc;
  RCLCPP_INFO(rclcpp::get_logger("bt_battery"), "SOC: %.1f%%", soc);
  return (soc > 20.0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus WaitBatteryRecovery::onStart()
{
  MOCK_CHECK_NODE();
  RCLCPP_WARN(rclcpp::get_logger("bt_battery"), "低电量待机 SOC=%.1f%%", g_mock_battery_soc);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitBatteryRecovery::onRunning()
{
  MOCK_CHECK_NODE();
  g_mock_battery_soc += 2.0;
  if (g_mock_battery_soc > 30.0) { RCLCPP_INFO(rclcpp::get_logger("bt_battery"), "电量恢复!"); return BT::NodeStatus::SUCCESS; }
  return BT::NodeStatus::RUNNING;
}

// ----- WaitForCIPSTask -----
// 阻塞等待 blackboard("cips_40001") == 1 (bt_executor 从 /cips_status 桥接)
// 收到后读取 cips_task_array → 写黑板 → SUCCESS
WaitForCIPSTask::WaitForCIPSTask(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::NodeStatus WaitForCIPSTask::onStart()
{
  MOCK_CHECK_NODE();
  if (auto t = getInput<int>("timeout")) timeout_ms_ = t.value();
  start_time_ = rclcpp::Clock().now();
  RCLCPP_INFO(rclcpp::get_logger("bt_cips"), "等待 CIPS 触发 (cips_40001==1)...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForCIPSTask::onRunning()
{
  MOCK_CHECK_NODE();
  if (timeout_ms_ > 0) {
    auto elapsed = (rclcpp::Clock().now() - start_time_).seconds() * 1000.0;
    if (elapsed > timeout_ms_) { RCLCPP_WARN(rclcpp::get_logger("bt_cips"), "超时"); return BT::NodeStatus::FAILURE; }
  }

  auto bb = config().blackboard;

  // 检查 cips_40001 (读锁立即释放)
  if (bbReadInt(bb, "cips_40001") != 1) return BT::NodeStatus::RUNNING;

  // 读取缓存的 HookTaskArray (读锁立即释放)
  std::shared_ptr<data_interfaces::msg::HookTaskArray> tasks;
  {
    auto a = bb->getAnyLocked("cips_task_array");
    if (!a) { RCLCPP_WARN(rclcpp::get_logger("bt_cips"), "cips_40001==1 但 cips_task_array 为空"); return BT::NodeStatus::RUNNING; }
    try { tasks = a->cast<std::shared_ptr<data_interfaces::msg::HookTaskArray>>(); } catch (...) { return BT::NodeStatus::FAILURE; }
  }  // ← 锁已释放, 下面可以安全 set

  if (!tasks) return BT::NodeStatus::FAILURE;

  bb->set("remaining_tasks", static_cast<int>(tasks->total_hook_num));
  if (!tasks->hook_tasks.empty()) {
    // bb->set("target_gap_id", static_cast<int>(tasks->hook_tasks[0].train_id));
    RCLCPP_INFO(rclcpp::get_logger("bt_cips"),
                "CIPS OK: total=%d first_id=%d train=%s hook=%d",
                tasks->total_hook_num, tasks->hook_tasks[0].train_id,
                tasks->hook_tasks[0].train_number_str.c_str(), tasks->hook_tasks[0].hook_lever_type);
  }
  // 清除触发标志
  // bb->set("cips_40001", 2);
  bb->set("task_round", 0);
  return BT::NodeStatus::SUCCESS;
}

void WaitForCIPSTask::onHalted()
{ RCLCPP_WARN(rclcpp::get_logger("bt_cips"), "被中断"); }

// ================================================================
// S2: 双臂复位校验
// ================================================================
BT::NodeStatus IsArmStowed::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

BT::NodeStatus IsCoArmHome::tick()
{
  MOCK_CHECK_NODE();
  return (bbReadInt(config().blackboard, "co_arm_40001") == 2) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsArmHome::tick()
{
  MOCK_CHECK_NODE();
  return (bbReadInt(config().blackboard, "arm_40001") == 2) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckCoSpeedArm::tick()
{
  MOCK_CHECK_NODE();
  auto cmd = getInput<std::string>("command");
  int expected = (cmd.value() == "RESET") ? 2 : (cmd.value() == "DEPLOY" ? 5 : (cmd.value() == "ATTACH" ? 6 : -1));
  int status = bbReadInt(config().blackboard, "co_arm_40001");
  if( status == expected ) { 
    RCLCPP_INFO(rclcpp::get_logger("bt_arm"), "CheckCoSpeedArm: command=%s expected=%d status=%d SUCCESS",
              cmd.value().c_str(), expected, status);
  }
  if (status >= 110) return BT::NodeStatus::SUCCESS;  // co-arm故障, 直接返回 SUCCESS, 避免阻塞
  return (status == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckMechanicalArm::tick()
{
  MOCK_CHECK_NODE();
  auto cmd = getInput<std::string>("command");
  int expected = (cmd.value() == "RESET" || cmd.value() == "HOME") ? 2 : cmd.value() == "ISOK" ? 5 : -1;
  int status = bbReadInt(config().blackboard, "arm_40001");
  if( status == expected ) {
    RCLCPP_INFO(rclcpp::get_logger("bt_arm"), "CheckMechanicalArm: command=%s expected=%d status=%d SUCCESS",
              cmd.value().c_str(), expected, status);
  }
  if (status >= 110) return BT::NodeStatus::SUCCESS;  // 机械臂故障, 直接返回 SUCCESS, 避免阻塞
  return (status == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ── WaitCoSpeedArm: 阻塞等待共速臂到位 ──
WaitCoSpeedArm::WaitCoSpeedArm(const std::string& n, const BT::NodeConfig& c)
  : StatefulActionNode(n, c)
{
  command_ = getInput<std::string>("command").value_or("RESET");
  timeout_ = getInput<double>("timeout").value_or(-1.0);
}

BT::NodeStatus WaitCoSpeedArm::onStart()
{
  start_time_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node")->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitCoSpeedArm::onRunning()
{
  MOCK_CHECK_NODE();
  int expected = (command_ == "RESET") ? 2 : (command_ == "DEPLOY" ? 5 : (command_ == "ATTACH" ? 6 : -1));
  int status = bbReadInt(config().blackboard, "co_arm_40001");
  if (status == expected ) {
    RCLCPP_INFO(rclcpp::get_logger("bt_arm"), "WaitCoSpeedArm: command=%s expected=%d status=%d SUCCESS",
                command_.c_str(), expected, status);
    return BT::NodeStatus::SUCCESS;
  }
  if (status >= 110) {
    RCLCPP_WARN(rclcpp::get_logger("bt_arm"), "WaitCoSpeedArm: command=%s expected=%d status=%d SUCCESS",
                command_.c_str(), expected, status);
    return BT::NodeStatus::FAILURE;
  }
  if (timeout_ > 0) {
    auto now = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node")->now();
    if ((now - start_time_).seconds() >= timeout_) {
      RCLCPP_WARN(rclcpp::get_logger("bt_arm"), "WaitCoSpeedArm: timeout %.1fs", timeout_);
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void WaitCoSpeedArm::onHalted() {}

// ── WaitMechanicalArm: 阻塞等待机械臂到位 ──
WaitMechanicalArm::WaitMechanicalArm(const std::string& n, const BT::NodeConfig& c)
  : StatefulActionNode(n, c)
{
  command_ = getInput<std::string>("command").value_or("RESET");
  timeout_ = getInput<double>("timeout").value_or(-1.0);
}

BT::NodeStatus WaitMechanicalArm::onStart()
{
  start_time_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node")->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitMechanicalArm::onRunning()
{
  MOCK_CHECK_NODE();
  int expected = (command_ == "RESET" || command_ == "HOME") ? 2 : command_ == "ISOK" ? 5 : -1;
  int status = bbReadInt(config().blackboard, "arm_40001");
  if (status == expected ) {
    RCLCPP_INFO(rclcpp::get_logger("bt_arm"), "WaitMechanicalArm: command=%s expected=%d status=%d SUCCESS",
                command_.c_str(), expected, status);
    return BT::NodeStatus::SUCCESS;
  }
  if (timeout_ > 0) {
    auto now = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node")->now();
    if ((now - start_time_).seconds() >= timeout_) {
      RCLCPP_WARN(rclcpp::get_logger("bt_arm"), "WaitMechanicalArm: timeout %.1fs", timeout_);
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void WaitMechanicalArm::onHalted() {}

// ── ExtractTaskInfo: 从 cips_task_array[task_round] 提取任务字段 ──
BT::NodeStatus ExtractTaskInfo::tick()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;

  int task_round = bbReadInt(bb, "task_round", 0);
  int remaining_tasks = bbReadInt(bb, "remaining_tasks", 0);

  // 读取 cips_task_array (锁立即释放)
  std::shared_ptr<data_interfaces::msg::HookTaskArray> tasks;
  {
    auto a = bb->getAnyLocked("cips_task_array");
    if (!a) {
      RCLCPP_ERROR(rclcpp::get_logger("bt_task"), "cips_task_array 为空");
      return BT::NodeStatus::FAILURE;
    }
    try { tasks = a->cast<std::shared_ptr<data_interfaces::msg::HookTaskArray>>(); }
    catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("bt_task"), "cips_task_array 类型转换失败");
      return BT::NodeStatus::FAILURE;
    }
  }  // ← 锁释放

  if (!tasks || task_round < 0 || static_cast<size_t>(task_round) >= tasks->hook_tasks.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("bt_task"),
      "task_round=%d 越界 (total=%zu)", task_round, tasks ? tasks->hook_tasks.size() : 0);
    return BT::NodeStatus::FAILURE;  // 越界时返回 SUCCESS, 避免阻塞
  }

  const auto& t = tasks->hook_tasks[task_round];

  // 写入黑板 (锁已释放, 安全写入)
  bb->set("train_id",         static_cast<int>(t.train_id));
  bb->set("train_type_code",  static_cast<int>(t.train_type_code));
  bb->set("coupling_count",   static_cast<int>(t.coupling_count));
  bb->set("air_pipe_off",     t.air_pipe_off);
  bb->set("carriage_brake",   t.carriage_brake);
  bb->set("train_number_str", t.train_number_str);
  bb->set("hook_lever_type",  t.hook_lever_type);

  RCLCPP_INFO(rclcpp::get_logger("bt_task"),
    "提取任务 task_round=%d: train_id=%d type=%d num=%s hook=%s",
    task_round, t.train_id, t.train_type_code,
    t.train_number_str.c_str(), t.hook_lever_type ? "上作用" : "下作用");
  bb->set("task_round", task_round + 1);
  RCLCPP_INFO(rclcpp::get_logger("bt_task"), "task_round=%d, remaining_tasks=%d ", task_round, remaining_tasks);
  bb->set("remaining_tasks", remaining_tasks - 1);
  return BT::NodeStatus::SUCCESS;
}
// ================================================================
// S3: 磁导航循迹
// ================================================================
BT::NodeStatus MagneticGuideOnline::tick()
{
  MOCK_CHECK_NODE();
  return BT::NodeStatus::SUCCESS;  // mock: 在线
}

NavigateToWaitArea::NavigateToWaitArea(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::NodeStatus NavigateToWaitArea::onStart()
{
  MOCK_CHECK_NODE();
  if (auto sp = getInput<double>("speed")) speed_ = sp.value();
  // 写黑板: gear=D, speed_cmd_mode=1(开始跟车)
  auto bb = config().blackboard;
  setBBSpeedCmd(bb, 4, 1, 0, 0, 2.0f, 0.5f, 0.5f, 1.0f);
  RCLCPP_INFO(rclcpp::get_logger("bt_nav"), "磁导航循迹 %.1f m/s", speed_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToWaitArea::onRunning()
{
  MOCK_CHECK_NODE();
  if (bbReadInt(config().blackboard, "speed_state") == 3) return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::RUNNING;
}

void NavigateToWaitArea::onHalted()
{
  auto bb = config().blackboard;
  setBBSpeedCmd(bb, 1, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f);  // P档+刹车
}

// ================================================================
// S4: 间隙定位
// ================================================================
WaitForTargetGap::WaitForTargetGap(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::NodeStatus WaitForTargetGap::onStart()
{
  MOCK_CHECK_NODE();
  RCLCPP_INFO(rclcpp::get_logger("bt_gap"),
    "等待目标车厢 ...");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForTargetGap::onRunning()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;
  int task_round = bbReadInt(bb, "task_round");
  if (task_round < 0) { 
    RCLCPP_ERROR(rclcpp::get_logger("bt_cips"), "task_round 未初始化: %d", task_round); 
    return BT::NodeStatus::FAILURE; 
  }
  int detect_id = bbReadInt(bb, "detect_id", -1);
  if ( detect_id < -1 ) {
    // RCLCPP_INFO(rclcpp::get_logger("bt_cips"), "未检测到车厢, detect_id=%d", detect_id);
    return BT::NodeStatus::RUNNING;
  }
  int train_id = bbReadInt(bb, "train_id");
  if (train_id > detect_id) {
    RCLCPP_WARN(rclcpp::get_logger("bt_cips"), "目标未到达! detect_id=%d target_gap_id=%d", detect_id, train_id);
    return BT::NodeStatus::RUNNING;
  }
  if (train_id == detect_id) {
    RCLCPP_INFO(rclcpp::get_logger("bt_cips"), "已锁定目标车厢! detect_id=%d == target_gap_id=%d", detect_id, train_id);
    return BT::NodeStatus::SUCCESS;
  }
  if (train_id < detect_id) {
    RCLCPP_WARN(rclcpp::get_logger("bt_cips"), "错失目标! detect_id=%d train_id=%d", detect_id, train_id);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForTargetGap::onHalted()
{ RCLCPP_WARN(rclcpp::get_logger("bt_waitgap"), "被中断"); }




// ================================================================
// S5: MPC 追车
// ================================================================
BT::NodeStatus GenerateInterceptTrajectory::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

MPCTrackApproach::MPCTrackApproach(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::NodeStatus MPCTrackApproach::onStart()
{
  MOCK_CHECK_NODE();
  if (auto t = getInput<double>("timeout")) timeout_ = t.value();
  start_time_ = rclcpp::Clock().now();
  // gear=D, ctrl_mode=1 (开始跟车)
  setBBSpeedCmd(config().blackboard, 4, 1, 0, 0, 3.0f, 0.5f, 0.5f, 1.0f);
  RCLCPP_INFO(rclcpp::get_logger("bt_mpc"), "MPC 追车开始");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MPCTrackApproach::onRunning()
{
  MOCK_CHECK_NODE();
  if ((rclcpp::Clock().now() - start_time_).seconds() > timeout_) {
    RCLCPP_WARN(rclcpp::get_logger("bt_mpc"), "MPC 超时");
    return BT::NodeStatus::FAILURE;
  }
  // 读 speed_state: 2=对齐目标 → SUCCESS
  int st = bbReadInt(config().blackboard, "speed_state");
  if (st == 2) {
    if (++consecutive_converged_ >= 3) { RCLCPP_INFO(rclcpp::get_logger("bt_mpc"), "MPC 追车完成"); return BT::NodeStatus::SUCCESS; }
  } else { consecutive_converged_ = 0; }
  return BT::NodeStatus::RUNNING;
}

void MPCTrackApproach::onHalted()
{ setBBSpeedCmd(config().blackboard, 1, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f); }

BT::NodeStatus IsTrackingErrorWithin::tick()
{
  MOCK_CHECK_NODE();
  auto tol = getInput<double>("tolerance");
  double err = bbReadDouble(config().blackboard, "detect_distance");
  return (err < tol.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsTargetInRange::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

// ================================================================
// S6: 锁定相对静止
// ================================================================
BT::NodeStatus SetMPCWeights::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

BT::NodeStatus IsStabilized::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

// ================================================================
// S7: 共速臂展开
// ================================================================
BT::NodeStatus SendCoSpeedArmCommand::tick()
{
  MOCK_CHECK_NODE();
  auto cmd = getInput<std::string>("command");

  // 解析指令
  int reg = 40001, val = 1;  // 默认 RESET
  if (cmd.value() == "RESET" || cmd.value() == "HOME") { reg = 40001; val = 1; }
  else if (cmd.value() == "DEPLOY")      { reg = 40001; val = 3; }
  else if (cmd.value() == "DEMAGNETIZE") { reg = 40001; val = 7; }
  else if (cmd.value() == "STOP")        { reg = 40001; val = 0; }
  else if (cmd.value() == "AUTO_MODE")   { reg = 40003; val = 0; }
  else if (cmd.value() == "SET_CAR_TYPE") {
    reg = 40008;
    val = bbReadInt(config().blackboard, "train_type_code");
  }

  // 懒初始化 publisher (安全读取 ros_node, 锁立即释放)
  if (!pub_initialized_) {
    rclcpp::Node::SharedPtr node;
    { auto a = config().blackboard->getAnyLocked("ros_node");
      if (a) try { node = a->cast<rclcpp::Node::SharedPtr>(); } catch (...) {} }
    if (node) {
      pub_ = node->create_publisher<std_msgs::msg::Int32MultiArray>("/co_arm_cmd", 10);
      pub_initialized_ = true;
    }
  }

  // 直接发布
  if (pub_initialized_) {
    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data = {reg, val};
    pub_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("bt_coarm"), "Co-ARM cmd send: reg=%d val=%d", reg, val);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsCoSpeedArmDeployed::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

// ================================================================
// S8: 微减速贴附
// ================================================================
BT::NodeStatus ApplyMicroDeceleration::tick()
{
  MOCK_CHECK_NODE();
  // ctrl_mode=2 (微减速)
  setBBSpeedCmd(config().blackboard, 4, 2, 0, 0, 0.5f, 0.3f, 0.3f, 1.0f);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckAttachmentForce::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

// ================================================================
// S9: 摘钩
// ================================================================
BT::NodeStatus SwitchToNeutralMode::tick()
{
  MOCK_CHECK_NODE();
  // gear=N(3), ctrl_mode=0(刹车)
  setBBSpeedCmd(config().blackboard, 3, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f);
  RCLCPP_INFO(rclcpp::get_logger("bt_uncouple"), "空档从动 gear=N");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SendMechanicalArmCommand::tick()
{
  MOCK_CHECK_NODE();
  auto cmd = getInput<std::string>("command");

  int reg = 40001, val = 1;  // RESET
  if (cmd.value() == "RESET" )             { reg = 40001; val = 1; }
  else if (cmd.value() == "DEPLOY")        { reg = 40001; val = 3; }
  else if (cmd.value() == "STOP")          { reg = 40001; val = 0; }
  else if (cmd.value() == "AUTO_MODE")     { reg = 40002; val = 0; }
  else if (cmd.value() == "SET_CAR_TYPE")  { reg = 40008; val = bbReadInt(config().blackboard, "train_type_code"); }

  // 懒初始化 publisher (安全读取 ros_node, 锁立即释放)
  if (!pub_initialized_) {
    rclcpp::Node::SharedPtr node;
    { auto a = config().blackboard->getAnyLocked("ros_node");
      if (a) try { node = a->cast<rclcpp::Node::SharedPtr>(); } catch (...) {} }
    if (node) {
      pub_ = node->create_publisher<std_msgs::msg::Int32MultiArray>("/arm_cmd", 10);
      pub_initialized_ = true;
    }
  }

  // 直接发布
  if (pub_initialized_) {
    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data = {reg, val};
    pub_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("bt_arm"), "ARM cmd send: reg=%d val=%d", reg, val);
  }
  return BT::NodeStatus::SUCCESS;
}

// ── SendCipsCommand ──
// 直接发布 /cips_cmd: RECEIVED→40001=2, REQUEST→40001=4
BT::NodeStatus SendCipsCommand::tick()
{
  MOCK_CHECK_NODE();
  auto cmd = getInput<std::string>("command");

  int reg = 40001, val = 2;  // 默认 RECEIVED
  if (cmd.value() == "RECEIVED") { val = 2; }
  else if (cmd.value() == "REQUEST") { val = 4; }

  // 懒初始化 publisher (安全读 ros_node)
  if (!pub_initialized_) {
    rclcpp::Node::SharedPtr node;
    { auto a = config().blackboard->getAnyLocked("ros_node");
      if (a) try { node = a->cast<rclcpp::Node::SharedPtr>(); } catch (...) {} }
    if (node) {
      pub_ = node->create_publisher<std_msgs::msg::Int32MultiArray>("/cips_cmd", 10);
      pub_initialized_ = true;
    }
  }

  if (pub_initialized_) {
    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data = {reg, val};
    pub_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("bt_cips_cmd"), "CIPS cmd send: reg=%d val=%d (%s)", reg, val, cmd.value().c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

// ── SpeedControlCommand: 直接发布 /speed_command ──
// 参考 SendCoSpeedArmCommand 的懒初始化 publisher 模式, 不经过黑板桥接
BT::NodeStatus SpeedControlCommand::tick()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;

  int gear      = getInput<int>("gear").value_or(1);
  int ctrl_mode = getInput<int>("ctrl_mode").value_or(0);
  float position = static_cast<float>(getInput<double>("position").value_or(0.7));
  float fwd_vel = static_cast<float>(getInput<double>("max_forward_vel").value_or(2.0));
  float bwd_vel = static_cast<float>(getInput<double>("max_backward_vel").value_or(0.5));
  float acc     = static_cast<float>(getInput<double>("max_acc").value_or(0.5));
  float dec     = static_cast<float>(getInput<double>("max_dec").value_or(1.0));

  int target_id     = bbReadInt(bb, "train_id", -1);
  int coupling_count = bbReadInt(bb, "coupling_count", 0);

  // 懒初始化 publisher (从黑板安全读取 ros_node)
  if (!pub_initialized_) {
    rclcpp::Node::SharedPtr node;
    { auto a = bb->getAnyLocked("ros_node");
      if (a) try { node = a->cast<rclcpp::Node::SharedPtr>(); } catch (...) {} }
    if (node) {
      pub_ = node->create_publisher<data_interfaces::msg::SpeedCommand>("/speed_command", 10);
      pub_initialized_ = true;
    }
  }

  // 直接发布 (不经过黑板, 不走 bridgeSpeedCommand)
  if (pub_initialized_) {
    auto msg = data_interfaces::msg::SpeedCommand();
    msg.target_gare      = static_cast<uint8_t>(gear);
    msg.ctrl_mode        = static_cast<uint8_t>(ctrl_mode);
    msg.target_id        = static_cast<uint8_t>(target_id);
    msg.position         = position;
    msg.max_forward_vel  = fwd_vel;
    msg.max_backward_vel = bwd_vel;
    msg.max_acc          = acc;
    msg.max_dec          = dec;
    msg.coupling_count   = static_cast<uint8_t>(coupling_count);
    pub_->publish(msg);
  }

  RCLCPP_INFO(rclcpp::get_logger("bt_speed"),
    "SpeedCmd: gear=%d mode=%d target=%d coupling=%d fwd=%.1f bwd=%.1f",
    gear, ctrl_mode, target_id, coupling_count, fwd_vel, bwd_vel);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsUncoupled::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

// ================================================================
// S11: 任务分发
// ================================================================
BT::NodeStatus CheckRemainingTasks::tick()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;
  int remaining = bbReadInt(bb, "remaining_tasks");
  RCLCPP_INFO(rclcpp::get_logger("bt_tasks"), "remaining_tasks=%d", remaining);
  return (remaining > 0) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}


// ================================================================
// S10/12: 返航
// ================================================================
NavigateToOrigin::NavigateToOrigin(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::NodeStatus NavigateToOrigin::onStart()
{
  MOCK_CHECK_NODE();
  if (auto m = getInput<std::string>("mode")) mode_ = m.value();
  // setBBSpeedCmd(config().blackboard, 4, 10, 0, 0, 2.0f, 0.5f, 0.5f, 1.0f);
  RCLCPP_INFO(rclcpp::get_logger("bt_nav"), "返航中...");
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToOrigin::onRunning()
{
  MOCK_CHECK_NODE();
  if (bbReadInt(config().blackboard, "speed_state") == 3) return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::SUCCESS;
}

void NavigateToOrigin::onHalted()
{ setBBSpeedCmd(config().blackboard, 1, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f); }

// ================================================================
// E_STOP
// ================================================================
BT::NodeStatus IsHeartbeatLost::tick()
{
  MOCK_CHECK_NODE();
  // ★ C++ 抢占层写入的 E_STOP 标记 (estop_heartbeat=1)
  if (bbReadInt(config().blackboard, "estop_heartbeat") == 1)
    return BT::NodeStatus::SUCCESS;
  // Mock 心跳 toggle (每 10s)
  static auto last_toggle = std::chrono::steady_clock::now();
  static bool ok = true;
  auto now = std::chrono::steady_clock::now();
  if (now - last_toggle >= std::chrono::seconds(10)) { ok = !ok; last_toggle = now;
    RCLCPP_WARN(rclcpp::get_logger("bt_heartbeat"), "Mock: 心跳 %s", ok ? "正常" : "丢失!"); }
  return ok ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsAtPositionLimit::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::FAILURE; }

BT::NodeStatus IsCollisionRisk::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::FAILURE; }

BT::NodeStatus EmergencyBrake::tick()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;
  // setBBSpeedCmd(bb, 1, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f);
  RCLCPP_ERROR(rclcpp::get_logger("bt_estop"), "紧急制动!!!");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CutPower::tick()
{
  MOCK_CHECK_NODE();
  RCLCPP_ERROR(rclcpp::get_logger("bt"), "切断动力电源!");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ReportFault::tick()
{
  MOCK_CHECK_NODE();
  auto code = getInput<std::string>("fault_code");
  std::string fault_str = code.has_value() ? code.value() : "UNKNOWN";
  RCLCPP_ERROR(rclcpp::get_logger("bt_fault"), "上报故障: %s", fault_str.c_str());

  // 懒初始化 publisher，发送到 /global/common_alarm
  if (!pub_initialized_) {
    rclcpp::Node::SharedPtr node;
    { auto a = config().blackboard->getAnyLocked("ros_node");
      if (a) try { node = a->cast<rclcpp::Node::SharedPtr>(); } catch (...) {} }
    if (node) {
      pub_ = node->create_publisher<std_msgs::msg::String>("/global/common_alarm", 10);
      pub_initialized_ = true;
    }
  }

  if (pub_initialized_) {
    auto msg = std_msgs::msg::String();
    msg.data = fault_str;
    pub_->publish(msg);
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ReportInfo::tick()
{
  MOCK_CHECK_NODE();
  auto code = getInput<std::string>("info_code");
  std::string info_str = code.has_value() ? code.value() : "UNKNOWN";
  RCLCPP_INFO(rclcpp::get_logger("bt_info"), "上报信息: %s", info_str.c_str());

  // 懒初始化 publisher，发送到 /global/common_info
  if (!pub_initialized_) {
    rclcpp::Node::SharedPtr node;
    { auto a = config().blackboard->getAnyLocked("ros_node");
      if (a) try { node = a->cast<rclcpp::Node::SharedPtr>(); } catch (...) {} }
    if (node) {
      pub_ = node->create_publisher<std_msgs::msg::String>("/global/common_info", 10);
      pub_initialized_ = true;
    }
  }

  if (pub_initialized_) {
    auto msg = std_msgs::msg::String();
    msg.data = info_str;
    pub_->publish(msg);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ReportEStop::tick()
{ MOCK_CHECK_NODE(); RCLCPP_ERROR(rclcpp::get_logger("bt"), "向 CIPS 上报急停!"); return BT::NodeStatus::SUCCESS; }

BT::NodeStatus ReportTaskComplete::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

BT::NodeStatus RejectIncomingTasks::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

BT::NodeStatus ParseTaskMessage::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

BT::NodeStatus StartGapCounting::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

BT::NodeStatus ResetTargetSpeed::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

BT::NodeStatus RetractCoSpeedArm::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

BT::NodeStatus IsBeforeWork::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

// ----- WaitForManualReset -----
WaitForManualReset::WaitForManualReset(const std::string& name, const BT::NodeConfig& config)
  : BT::CoroActionNode(name, config) {}

BT::NodeStatus WaitForManualReset::tick()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;
  RCLCPP_ERROR(rclcpp::get_logger("bt_reset"), "系统故障! 等待人工复位...");
  while (true) {
    // RCLCPP_WARN(rclcpp::get_logger("bt_reset"), "setStatusRunningAndYield之前打印：%d", bbReadInt(bb, "hmi_ctrl_mode"));
    // setStatusRunningAndYield();
    // RCLCPP_WARN(rclcpp::get_logger("bt_reset"), "setStatusRunningAndYield之后打印：%d", bbReadInt(bb, "hmi_ctrl_mode"));

    int hmi_mode = bbReadInt(bb, "hmi_ctrl_mode");
    if (hmi_mode == 3) {
      RCLCPP_INFO(rclcpp::get_logger("bt_reset"), ">>> HMI 解除 ESTOP! <<<");
      bb->set("hmi_ctrl_mode", 0);
      bb->set("estop_heartbeat", 0);   // 清除 C++ 层的 E_STOP 标记
      return BT::NodeStatus::SUCCESS;
    }
    static int log_tick = 0;
    if (log_tick++ % 50 == 0)
      RCLCPP_WARN(rclcpp::get_logger("bt_reset"), "等待复位... (%d ticks)", log_tick);
  }
}

void WaitForManualReset::halt()
{
  RCLCPP_WARN(rclcpp::get_logger("bt_reset"), "复位等待被中断");
  CoroActionNode::halt();
}


// ================================================================
// 通用动作
// ================================================================
// ── CheckCtrlStatus: 等待 speed_state 达到指定值 ──
CheckCtrlStatus::CheckCtrlStatus(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::NodeStatus CheckCtrlStatus::onStart()
{
  MOCK_CHECK_NODE();
  if (auto c = getInput<int>("command")) command_ = c.value();
  if (auto t = getInput<int>("timeout"))  timeout_  = static_cast<double>(t.value());
  start_time_ = rclcpp::Clock().now();
  RCLCPP_INFO(rclcpp::get_logger("bt_ctrl"), "等待 speed_state==%d, timeout=%.0fs", command_, timeout_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckCtrlStatus::onRunning()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;

  int state = bbReadInt(bb, "speed_state", -1);

  if (state == command_) {
    RCLCPP_INFO(rclcpp::get_logger("bt_ctrl"), "speed_state==%d 达成", command_);
    return BT::NodeStatus::SUCCESS;
  }

  double elapsed = (rclcpp::Clock().now() - start_time_).seconds();
  if (elapsed > timeout_) {
    RCLCPP_WARN(rclcpp::get_logger("bt_ctrl"),
      "超时 %.1fs: speed_state=%d, 期望=%d", elapsed, state, command_);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void CheckCtrlStatus::onHalted()
{
  RCLCPP_WARN(rclcpp::get_logger("bt_ctrl"), "被中断");
}

BT::NodeStatus ApplyParkingBrake::tick()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;
  setBBSpeedCmd(bb, 1, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f);
  RCLCPP_INFO(rclcpp::get_logger("bt_brake"), "驻车制动 → gear=P, ctrl_mode=刹车");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ReleaseParkingBrake::tick()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;
  setBBSpeedCmd(bb, 4, 1, 0, 0, 2.0f, 0.5f, 0.5f, 1.0f);
  RCLCPP_INFO(rclcpp::get_logger("bt_brake"), "解除驻车 → gear=D, ctrl_mode=跟车");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SendArmCommand::tick()
{
  MOCK_CHECK_NODE();
  auto cmd = getInput<std::string>("command");
  auto bb = config().blackboard;
  int val = (cmd.value() == "RESET") ? 1 : (cmd.value() == "LIFT" ? 3 : 1);
  setBBCmd(bb, "arm_cmd_reg", 40001, "arm_cmd_val", val);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitStabilization::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

// ── WaitDelay: 纯等待节点 ──
WaitDelay::WaitDelay(const std::string& n, const BT::NodeConfig& c)
  : StatefulActionNode(n, c)
{
  if (auto delay = getInput<int>("delay_s")) delay_s_ = *delay;
}

BT::NodeStatus WaitDelay::onStart()
{
  start_time_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node")->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitDelay::onRunning()
{
  if (delay_s_ == -1) return BT::NodeStatus::RUNNING;  // -1 → 永久等待
  auto now = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node")->now();
  if ((now - start_time_).seconds() >= delay_s_) return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::RUNNING;
}

void WaitDelay::onHalted() {}

BT::NodeStatus WaitForUncoupleConfirm::tick()
{ MOCK_CHECK_NODE(); return BT::NodeStatus::SUCCESS; }

// ================================================================
// 动/静态模式
// ================================================================
BT::NodeStatus IsStaticMode::tick()
{
  MOCK_CHECK_NODE();
  return (bbReadInt(config().blackboard, "work_mode") == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsDynamicMode::tick()
{
  MOCK_CHECK_NODE();
  return (bbReadInt(config().blackboard, "work_mode") == 1) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ================================================================
// 静态摘钩
// ================================================================
BT::NodeStatus ComputeStaticTargetPose::tick()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;
  // TODO: 根据 target_gap_id 计算实际目标位姿
  bb->set("target_x", 10.0);
  bb->set("target_y", 0.0);
  bb->set("target_yaw", 0.0);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckStaticTargetPose::tick()
{
  MOCK_CHECK_NODE();
  auto bb = config().blackboard;
  int detect_id = bbReadInt(bb, "detect_id", -1);
  int train_id  = bbReadInt(bb, "train_id", -1); 
  int detect_distance = bbReadInt(bb, "detect_distance", -1);
  auto position = getInput<double>("position");
  if (detect_id < -1 || train_id <= 0) {
    RCLCPP_WARN(rclcpp::get_logger("bt_static"),
      "detect_id=%d train_id=%d 数据不可用", detect_id, train_id);
    return BT::NodeStatus::FAILURE;
  }

  int ctrl_mode; int gear;
  if (detect_id  > train_id) {
    ctrl_mode = 10;  // 已接近目标 → 定速到点
    gear = 4;  // D档
    RCLCPP_INFO(rclcpp::get_logger("bt_static"),
      "接近目标: detect_id=%d train_id=%d → ctrl_mode=10(定速到点)", detect_id, train_id);
  } 
  else if (detect_id  < train_id){
    ctrl_mode = 11;  // 未到目标
    gear = 2;  // P档
    RCLCPP_INFO(rclcpp::get_logger("bt_static"),
      "未到目标: detect_id=%d train_id=%d → ctrl_mode=11", detect_id, train_id);
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("bt_static"),
        "已到目标: detect_id=%d train_id=%d  detect_distance=%d position=%f", detect_id, train_id, detect_distance, position.value());
    if (detect_distance >= position.value()) {
      ctrl_mode = 10;  // 已到目标
      gear = 4;  // D档
    } else {
      ctrl_mode = 11;  // 未到目标
      gear = 2;  // P档
    }
  }

  bb->set("static_target_ctrl_mode", ctrl_mode);
  bb->set("static_target_gear", gear);
  return BT::NodeStatus::SUCCESS;
}

NavigateToStaticTarget::NavigateToStaticTarget(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::NodeStatus NavigateToStaticTarget::onStart()
{
  MOCK_CHECK_NODE();
  if (auto s = getInput<double>("speed")) speed_ = s.value();
  if (auto t = getInput<double>("timeout")) timeout_ = t.value();
  start_time_ = rclcpp::Clock().now();
  // ctrl_mode=10 (前进定速+到点)
  setBBSpeedCmd(config().blackboard, 4, 10, 0, 0, static_cast<float>(speed_), 0.5f, 0.5f, 1.0f);
  RCLCPP_INFO(rclcpp::get_logger("bt_nav"), "静态导航 %.1f m/s", speed_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToStaticTarget::onRunning()
{
  MOCK_CHECK_NODE();
  if ((rclcpp::Clock().now() - start_time_).seconds() > timeout_) return BT::NodeStatus::FAILURE;
  int st = bbReadInt(config().blackboard, "speed_state");
  if (st == 3) { if (++consecutive_converged_ >= 3) return BT::NodeStatus::SUCCESS; }
  else consecutive_converged_ = 0;
  return BT::NodeStatus::RUNNING;
}

void NavigateToStaticTarget::onHalted()
{ setBBSpeedCmd(config().blackboard, 1, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f); }

BT::NodeStatus IsAtTargetPose::tick()
{
  MOCK_CHECK_NODE();
  return (bbReadInt(config().blackboard, "speed_state") == 3) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}



// ================================================================
// 注册
// ================================================================
void RegisterAllNodes(BT::BehaviorTreeFactory& factory, rclcpp::Node::SharedPtr)
{
  // S0
  factory.registerNodeType<CheckCommunication>("CheckCommunication");
  factory.registerNodeType<CheckSensors>("CheckSensors");
  factory.registerNodeType<CheckActuators>("CheckActuators");
  // S1
  factory.registerNodeType<IsBatteryNormal>("IsBatteryNormal");
  factory.registerNodeType<WaitBatteryRecovery>("WaitBatteryRecovery");
  factory.registerNodeType<WaitForCIPSTask>("WaitForCIPSTask");
  // S2
  factory.registerNodeType<IsArmStowed>("IsArmStowed");
  factory.registerNodeType<IsCoArmHome>("IsCoArmHome");
  factory.registerNodeType<IsArmHome>("IsArmHome");
  factory.registerNodeType<CheckCoSpeedArm>("CheckCoSpeedArm");
  factory.registerNodeType<WaitCoSpeedArm>("WaitCoSpeedArm");
  factory.registerNodeType<CheckMechanicalArm>("CheckMechanicalArm");
  factory.registerNodeType<WaitMechanicalArm>("WaitMechanicalArm");
  // S3
  factory.registerNodeType<MagneticGuideOnline>("MagneticGuideOnline");
  factory.registerNodeType<NavigateToWaitArea>("NavigateToWaitArea");
  // S4
  factory.registerNodeType<WaitForTargetGap>("WaitForTargetGap");
  factory.registerNodeType<ExtractTaskInfo>("ExtractTaskInfo");
  // S5
  factory.registerNodeType<GenerateInterceptTrajectory>("GenerateInterceptTrajectory");
  factory.registerNodeType<MPCTrackApproach>("MPCTrackApproach");
  factory.registerNodeType<IsTrackingErrorWithin>("IsTrackingErrorWithin");
  factory.registerNodeType<IsTargetInRange>("IsTargetInRange");
  // S6
  factory.registerNodeType<SetMPCWeights>("SetMPCWeights");
  factory.registerNodeType<IsStabilized>("IsStabilized");
  // S7
  factory.registerNodeType<SendCoSpeedArmCommand>("SendCoSpeedArmCommand");
  factory.registerNodeType<IsCoSpeedArmDeployed>("IsCoSpeedArmDeployed");
  // S8
  factory.registerNodeType<ApplyMicroDeceleration>("ApplyMicroDeceleration");
  factory.registerNodeType<CheckAttachmentForce>("CheckAttachmentForce");
  // S9
  factory.registerNodeType<SwitchToNeutralMode>("SwitchToNeutralMode");
  factory.registerNodeType<SendMechanicalArmCommand>("SendMechanicalArmCommand");
  factory.registerNodeType<SendCipsCommand>("SendCipsCommand");
  factory.registerNodeType<IsUncoupled>("IsUncoupled");
  // S11
  factory.registerNodeType<CheckRemainingTasks>("CheckRemainingTasks");
  // S10/12
  factory.registerNodeType<NavigateToOrigin>("NavigateToOrigin");
  // E_STOP
  factory.registerNodeType<IsHeartbeatLost>("IsHeartbeatLost");
  factory.registerNodeType<IsAtPositionLimit>("IsAtPositionLimit");
  factory.registerNodeType<IsCollisionRisk>("IsCollisionRisk");
  factory.registerNodeType<EmergencyBrake>("EmergencyBrake");
  factory.registerNodeType<CutPower>("CutPower");
  factory.registerNodeType<ReportFault>("ReportFault");
  factory.registerNodeType<ReportInfo>("ReportInfo");
  factory.registerNodeType<ReportEStop>("ReportEStop");
  factory.registerNodeType<ReportTaskComplete>("ReportTaskComplete");
  factory.registerNodeType<WaitForManualReset>("WaitForManualReset");
  // S1 辅助
  factory.registerNodeType<RejectIncomingTasks>("RejectIncomingTasks");
  factory.registerNodeType<ParseTaskMessage>("ParseTaskMessage");
  factory.registerNodeType<StartGapCounting>("StartGapCounting");
  // S9/S11 辅助
  factory.registerNodeType<ResetTargetSpeed>("ResetTargetSpeed");
  factory.registerNodeType<RetractCoSpeedArm>("RetractCoSpeedArm");
  factory.registerNodeType<IsBeforeWork>("IsBeforeWork");
  // 通用
  factory.registerNodeType<SpeedControlCommand>("SpeedControlCommand");
  factory.registerNodeType<CheckCtrlStatus>("CheckCtrlStatus");
  factory.registerNodeType<ApplyParkingBrake>("ApplyParkingBrake");
  factory.registerNodeType<ReleaseParkingBrake>("ReleaseParkingBrake");
  factory.registerNodeType<SendArmCommand>("SendArmCommand");
  factory.registerNodeType<WaitStabilization>("WaitStabilization");
  factory.registerNodeType<WaitDelay>("WaitDelay");
  factory.registerNodeType<WaitForUncoupleConfirm>("WaitForUncoupleConfirm");
  // 模式
  factory.registerNodeType<IsStaticMode>("IsStaticMode");
  factory.registerNodeType<IsDynamicMode>("IsDynamicMode");
  // 静态
  factory.registerNodeType<ComputeStaticTargetPose>("ComputeStaticTargetPose");
  factory.registerNodeType<NavigateToStaticTarget>("NavigateToStaticTarget");
  factory.registerNodeType<IsAtTargetPose>("IsAtTargetPose");
  factory.registerNodeType<CheckStaticTargetPose>("CheckStaticTargetPose");
}

}  // namespace uncoupling_robot
