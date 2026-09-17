# HMI 统一状态话题 — 设计说明

> 设计目标: 用一个话题 ` /bt_hmi_state` 向 HMI 人机界面提供行为树当前模式、子树阶段和 E_STOP 状态的完整信息，替代目前分散的 `/bt_status` + `/bt_current_state` + `/bt/blackboard` 话题。

---

## 一、消息定义

### 新增 `msg/BtHmiState.msg`

```msg
# BtHmiState.msg — 行为树统一状态 (发布给 HMI 人机界面)
# 发布频率: 5-10hz
# 发布者: bt_executor_node
# 订阅者: HMI 人机交互

# ══════════════════════════════════════════════════════
# 1. 系统运行模式
# ══════════════════════════════════════════════════════
uint8 system_mode
# 0 = MANUAL      — HMI 手动控制 (行为树已停止, hmi_ctrl_mode=0)
# 1 = AUTO_DYNAMIC — 自动动态摘钩 (work_mode=1, MPC 追车)
# 2 = AUTO_STATIC  — 自动静态摘钩 (work_mode=0, PID 导航)
# 3 = E_STOP       — 紧急停止 (E_STOP 序列执行中)

# ══════════════════════════════════════════════════════
# 2. 当前活动子树
# ══════════════════════════════════════════════════════
string active_subtree
# 可能值:
#   "IDLE"               — 行为树空闲 (尚未开始或已完成)
#   "S0_INIT"            — 上电自检
#   "S1_IDLE"            — 空闲待命 (等待 CIPS 任务)
#   "S2_ARM_CHECK"       — 双臂复位校验
#   "S3_NAV_WAIT_AREA"   — 前往待位区
#   "S4_TARGET_POSITIONING" — 雷达锁定中
#   "S5_TRACK_APPROACH"  — 动态追车 (动态)
#   "S6_SYNC_STABILIZE"  — 维持共速 (动态)
#   "S7_ARM_DEPLOY"      — 共速臂展开 (动态)
#   "S8_ATTACH_DECEL"    — 微减速贴附 (动态)
#   "S9_UNCOUPLE"        — 空档与提钩 (动态)
#   "S11_ARM_RESET"      — 共速臂复位 (动态)
#   "S5_STATIC_ALIGN"    — 静态对齐导航 (静态)
#   "S9_STATIC_UNCOUPLE" — 机械臂提钩 (静态)
#   "S11_STATIC_RESET"   — 静态复位 (静态)
#   "S12_RETURN_HOME"    — 返航结束
#   "E_STOP"             — 急停处理中

# ══════════════════════════════════════════════════════
# 3. 运行参数快照
# ══════════════════════════════════════════════════════
int32 remaining_tasks     # 剩余摘钩任务数 (-1=未知/未设置)
int32 gap_count           # 当前间隙计数 (-1=未知)
string active_leaf_node   # 当前正在执行的叶子节点名称 (如 "WaitForCIPSTask")
```

---

## 二、状态判定逻辑

### 2.1 `system_mode` 判定

```cpp
// 优先级: E_STOP > MANUAL > AUTO_DYNAMIC > AUTO_STATIC
if (estop_active) {
  system_mode = 3;  // E_STOP
} else if (hmi_mode_in_bb_ == -1) {
  system_mode = 0;  // MANUAL
} else if (work_mode == 1) {
  system_mode = 1;  // AUTO_DYNAMIC
} else if (work_mode == 0) {
  system_mode = 2;  // AUTO_STATIC
}
```

### 2.2 `active_subtree` 判定

通过遍历所有子树节点，找到状态为 `RUNNING` 的节点，从其 `fullPath()` 中提取子树 ID。

**fullPath 示例**:
```
MainTree.global_estop_monitor.main_flow.ForceSuccess.KeepRunningUntilFailure.
IdleToMissionLoop.RetryUntilSuccessful.SingleHookExecutionFlow.
work_mode_dispatcher.dynamic_mode_flow.S5_TRACK_APPROACH.
track_approach.track_with_guard.MPCTrackApproach
```

**提取规则** (按优先级匹配):

```
fullPath 中包含 "estop_actions"        → active_subtree = "E_STOP"
fullPath 中包含 "S0_INIT"              → active_subtree = "S0_INIT"
fullPath 中包含 "S1_IDLE"              → active_subtree = "S1_IDLE"
fullPath 中包含 "S2_ARM_CHECK"         → active_subtree = "S2_ARM_CHECK"
fullPath 中包含 "S3_NAV_WAIT_AREA"     → active_subtree = "S3_NAV_WAIT_AREA"
fullPath 中包含 "S4_TARGET_POSITIONING" → active_subtree = "S4_TARGET_POSITIONING"
fullPath 中包含 "S5_TRACK_APPROACH"    → active_subtree = "S5_TRACK_APPROACH"
fullPath 中包含 "S5_STATIC_ALIGN"      → active_subtree = "S5_STATIC_ALIGN"
fullPath 中包含 "S6_SYNC_STABILIZE"    → active_subtree = "S6_SYNC_STABILIZE"
fullPath 中包含 "S7_ARM_DEPLOY"        → active_subtree = "S7_ARM_DEPLOY"
fullPath 中包含 "S8_ATTACH_DECEL"      → active_subtree = "S8_ATTACH_DECEL"
fullPath 中包含 "S9_UNCOUPLE"          → active_subtree = "S9_UNCOUPLE"
fullPath 中包含 "S9_STATIC_UNCOUPLE"   → active_subtree = "S9_STATIC_UNCOUPLE"
fullPath 中包含 "S11_ARM_RESET"        → active_subtree = "S11_ARM_RESET"
fullPath 中包含 "S11_STATIC_RESET"     → active_subtree = "S11_STATIC_RESET"
fullPath 中包含 "S12_RETURN_HOME"      → active_subtree = "S12_RETURN_HOME"
无 RUNNING 节点，tree status==IDLE    → active_subtree = "IDLE"
```

> **注意**: 由于 `S6_SYNC_STABILIZE` 在动态流程中出现两次 (`_phase="PRE_WORK"` 和 `_phase="POST_WORK"`)，仅从 fullPath 中提取 SubTree ID 无法区分前后阶段。HMI 如果需要区分，可以结合 `active_leaf_node` 判断——PRE 阶段执行 `WaitStabilization`，POST 阶段执行 `RetractCoSpeedArm`。

### 2.3 `estop_active` 判定

```cpp
// E_STOP 激活条件: 当前存在 RUNNING 节点且该节点位于 estop_actions 子树中
estop_active = (active_subtree == "E_STOP");
```

### 2.4 `active_leaf_node` 判定

```cpp
// 找到所有 RUNNING 节点中最深的那个 (叶子节点)
// 取其 registrationName() 或 name()
// 如: "WaitForManualReset", "MPCTrackApproach", "CheckCommunication"
```

---

## 三、实现位置

所有逻辑在 `bt_executor_node.cpp` 中实现，不需要修改 `robot_nodes.cpp`：


---

## 四、HMI 端使用示例

### 4.1 主题订阅

```bash
ros2 topic echo /bt_hmi_state
```

输出示例 (动态模式正在追车):
```yaml
system_mode: 1          # AUTO_DYNAMIC
active_subtree: "S5_TRACK_APPROACH"
active_leaf_node: "MPCTrackApproach"
estop_active: false
work_mode: 1
remaining_tasks: 2
gap_count: 0
```

输出示例 (E_STOP 急停中):
```yaml
system_mode: 3          # E_STOP
active_subtree: "E_STOP"
active_leaf_node: "WaitForManualReset"
estop_active: true
work_mode: 1
remaining_tasks: 1
gap_count: 1
```

输出示例 (手动模式):
```yaml
system_mode: 0          # MANUAL
active_subtree: "IDLE"
active_leaf_node: ""
estop_active: false
work_mode: 1
remaining_tasks: -1
gap_count: -1
```

### 4.2 HMI 界面可实现的显示

**主控页面只显示active_subtree**

| HMI 控件 | 数据源 | 显示逻辑 |
|----------|--------|---------|
| 模式指示灯 | `system_mode` | 0=灰色(手动), 1=蓝色(动态), 2=绿色(静态), 3=红色(E_STOP) |
| 当前步骤 | `active_subtree` | 显示 Sx 名称和中文描述 |
| 当前动作 | `active_leaf_node` | 显示具体节点名称 |
| 剩余任务 | `remaining_tasks` | "剩余 N 钩" |
| 间隙计数 | `gap_count` | "当前第 N 个间隙" |


---

控制按钮解释：上部五个按钮均需长按1s后有效
1. 自动动态提钩：执行命令如下
   ros2 topic pub --once /co_arm_status std_msgs/msg/Float32MultiArray '{data: [40003,0]}'
   ros2 topic pub --once /arm_status std_msgs/msg/Int32MultiArray '{data: [40002,0]}'
   /ctrl_hmi 发布一次 hmi_ctrl_mode=1
2. 自动静态提钩：
   ros2 topic pub --once /co_arm_status std_msgs/msg/Float32MultiArray '{data: [40003,0]}'
   ros2 topic pub --once /arm_status std_msgs/msg/Int32MultiArray '{data: [40002,0]}'
   /ctrl_hmi 发布一次 hmi_ctrl_mode=2
3. 手动遥控：
   ros2 topic pub --once /co_arm_status std_msgs/msg/Float32MultiArray '{data: [40003,1]}'
   ros2 topic pub --once /arm_status std_msgs/msg/Int32MultiArray '{data: [40002,1]}'
   /ctrl_hmi 持续发布 hmi_ctrl_mode=0，其他字段为中部的档位按钮和速度条，发布频率50hz
4. 解除ESTOP 
   /ctrl_hmi 发布一次 hmi_ctrl_mode=3
5. 重置全部任务
  重置服务 
  服务名称：/srv/reset_counter
  类型：std_srvs/srv/Trigger
  # Request：HMI
  空（仅触发信号）
  ---
  # Response：清零
  bool success
  string msg