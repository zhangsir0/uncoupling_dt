# 行为树话题与黑板变量参考

> 自动生成自 `bt_executor_node.cpp`, `robot_nodes.cpp/h`, 及所有 `.msg` 文件  
> 日期: 2026-07-09

---

## 自定义消息定义

| 消息文件 | 完整 ROS2 类型 | 字段 |
|----------|---------------|------|
| `SpeedCommand.msg` | `uncoupling_robot_bt::msg::SpeedCommand` | `uint8 target_gare`, `uint8 ctrl_mode`, `uint8 target_id`, `uint8 coupling_count`, `float32 position`, `float32 max_forward_vel`, `float32 max_backward_vel`, `float32 max_acc`, `float32 max_dec` |
| `HmiCommand.msg` | `uncoupling_robot_bt::msg::HmiCommand` | `uint8 hmi_ctrl_mode`, `uint8 ctrl_gare`, `float32 ctrl_speed`, `uint8 ctrl_break` |
| `BtHmiState.msg` | `uncoupling_robot_bt::msg::BtHmiState` | `uint8 system_mode`, `string active_subtree`, `int32 remaining_tasks`, `int32 gap_count`, `string active_leaf_node` |
| `DetectResult.msg` | `uncoupling_robot_bt::msg::DetectResult` | `uint8 current_id`, `float32 current_distance`, `float32 relative_velocity` |
| `HookTaskArray.msg` | `uncoupling_robot_bt::msg::HookTaskArray` | `std_msgs/Header header`, `int32 total_hook_num`, `HookTask[] hook_tasks` |
| `HookTask.msg` | `uncoupling_robot_bt::msg::HookTask` | `uint8 train_id`, `uint8 train_type_code`, `uint8 coupling_count`, `bool air_pipe_off`, `bool carriage_brake`, `string train_number_str`, `bool hook_lever_type` |

---

## 订阅话题（→黑板）

### `/co_arm_status`
- **类型**: `std_msgs/msg/Float32MultiArray`
- **写入黑板**: `co_arm_40001`, `co_arm_40002`, `co_arm_40003` (int32)
- **逻辑**: 遍历 `data` 中 `[addr, val]` 对，匹配地址 40001/40002/40003

```bash
ros2 topic pub --once /co_arm_status std_msgs/msg/Float32MultiArray \
  '{data: [40001.0, 2.0, 40002.0, 1.0, 40003.0, 0.0]}'
```

### `/arm_status`
- **类型**: `std_msgs/msg/Int32MultiArray`
- **写入黑板**: `arm_40001`, `arm_40002`, `arm_40003` (int32)

```bash
ros2 topic pub --once /arm_status std_msgs/msg/Int32MultiArray \
  '{data: [40001, 2, 40002, 0, 40003, 0]}'
```

### `/ctrl_hmi`
- **类型**: `uncoupling_robot_bt/msg/HmiCommand`
- **写入黑板**: `work_mode` (间接，通过 `handleHmiCommand()`)
- **行为**:
  - `hmi_ctrl_mode=0`: 停止行为树 (halt + cancel timer)
  - `hmi_ctrl_mode=1`: work_mode=1 动态摘钩
  - `hmi_ctrl_mode=2`: work_mode=0 静态摘钩
  - `hmi_ctrl_mode=3`: 解除 E_STOP

```bash
# 动态模式
ros2 topic pub --once /ctrl_hmi uncoupling_robot_bt/msg/HmiCommand \
  '{hmi_ctrl_mode: 1, ctrl_gare: 4, ctrl_speed: 2.0, ctrl_break: 0}'

# 手动停止
ros2 topic pub --once /ctrl_hmi uncoupling_robot_bt/msg/HmiCommand \
  '{hmi_ctrl_mode: 0, ctrl_gare: 1, ctrl_speed: 0.0, ctrl_break: 0}'

# 解除急停
ros2 topic pub --once /ctrl_hmi uncoupling_robot_bt/msg/HmiCommand \
  '{hmi_ctrl_mode: 3, ctrl_gare: 4, ctrl_speed: 0.0, ctrl_break: 0}'
```

### `/detect_result`
- **类型**: `uncoupling_robot_bt/msg/DetectResult`
- **写入黑板**: `detect_id` (int), `detect_distance` (double), `detect_velocity` (double)

```bash
ros2 topic pub --once /detect_result uncoupling_robot_bt/msg/DetectResult \
  '{current_id: 20, current_distance: 3.25, relative_velocity: 0.5}'
```

### `/ctrl_mag_stop`
- **类型**: `std_msgs/msg/Bool`
- **写入黑板**: `ctrl_mag_stop` (bool) — `true` 触发 E_STOP

```bash
ros2 topic pub --once /ctrl_mag_stop std_msgs/msg/Bool '{data: true}'
```

### `/ctrl_fence`
- **类型**: `std_msgs/msg/UInt8`
- **写入黑板**: `ctrl_fence` (int) — `1` 触发 E_STOP

```bash
ros2 topic pub --once /ctrl_fence std_msgs/msg/UInt8 '{data: 1}'
```

### `/cips/hook_task_array`
- **类型**: `uncoupling_robot_bt/msg/HookTaskArray`
- **写入黑板**: `cips_total_hook_num` (int), `cips_task_array` (shared_ptr\<HookTaskArray\>)

```bash
ros2 topic pub --once /cips/hook_task_array uncoupling_robot_bt/msg/HookTaskArray \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "cips"}, total_hook_num: 3, hook_tasks: [
    {train_id: 20, train_type_code: 1, coupling_count: 20, air_pipe_off: false, carriage_brake: false, train_number_str: "C70", hook_lever_type: false},
    {train_id: 27, train_type_code: 2, coupling_count: 7, air_pipe_off: true, carriage_brake: false, train_number_str: "C64", hook_lever_type: true},
    {train_id: 29, train_type_code: 3, coupling_count: 2, air_pipe_off: false, carriage_brake: true, train_number_str: "P70ak", hook_lever_type: false}
  ]}'
```
ros2 topic pub --once /cips/hook_task_array data_interfaces/msg/HookTaskArray \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "cips"}, total_hook_num: 3, hook_tasks: [
    {train_id: 1, train_type_code: 1, coupling_count: 20, air_pipe_off: false, carriage_brake: false, train_number_str: "C70", hook_lever_type: false},
    {train_id: 3, train_type_code: 2, coupling_count: 7, air_pipe_off: true, carriage_brake: false, train_number_str: "C64", hook_lever_type: true},
    {train_id: 5, train_type_code: 3, coupling_count: 2, air_pipe_off: false, carriage_brake: true, train_number_str: "P70ak", hook_lever_type: false}
  ]}'

### `/cips_status`
- **类型**: `std_msgs/msg/Int32MultiArray`
- **写入黑板**: `cips_40001` (int32) — `1` 触发任务开始

```bash
ros2 topic pub --once /cips_status std_msgs/msg/Int32MultiArray '{data: [40001, 1]}'
```

### `/speed_state`
- **类型**: `std_msgs/msg/UInt8`
- **写入黑板**: `speed_state` (int)
  - `0` = 停止
  - `1` = 运动中
  - `2` = 已对齐目标
  - `3` = 完成

```bash
ros2 topic pub --once /speed_state std_msgs/msg/UInt8 '{data: 3}'
```

---

## 发布话题（黑板→话题）

### `/bt_hmi_state`
- **类型**: `uncoupling_robot_bt/msg/BtHmiState`
- **频率**: 10 Hz
- **读取黑板**: `work_mode`, `remaining_tasks`, `cips_total_hook_num`, 树遍历获取 `active_subtree`/`active_leaf_node`

```bash
ros2 topic echo /bt_hmi_state
```

### `/global/common_alarm`
- **类型**: `std_msgs/msg/String`
- **频率**: 事件驱动 (`ReportFault` 节点每次 tick 发布一次)
- **发布者**: `ReportFault` 叶子节点 (直接发布，不经黑板桥接)

```bash
ros2 topic echo /global/common_alarm
```

### `/speed_command`
- **类型**: `uncoupling_robot_bt/msg/SpeedCommand`
- **频率**: 事件驱动 (检测 `speed_cmd_gare >= 0`)
- **发布者**: `bt_executor_node` → `bridgeSpeedCommand()`
- **读取黑板**:

| 黑板变量 | 消息字段 |
|----------|----------|
| `speed_cmd_gare` | `target_gare` |
| `speed_cmd_mode` | `ctrl_mode` |
| `speed_cmd_target_id` | `target_id` |
| `speed_cmd_coupling_count` | `coupling_count` |
| `speed_cmd_position` | `position` |
| `speed_cmd_fwd_vel` | `max_forward_vel` |
| `speed_cmd_bwd_vel` | `max_backward_vel` |
| `speed_cmd_acc` | `max_acc` |
| `speed_cmd_dec` | `max_dec` |

```bash
ros2 topic echo /speed_command

ros2 topic pub --once /speed_command data_interfaces/msg/SpeedCommand \
  '{target_gare: 4, ctrl_mode: 1, target_id: 1, coupling_count: 1, position: 0.6, max_forward_vel: 2.0, max_backward_vel: 1.0, max_acc: 0.5, max_dec: 1.0}'
```
ros2 topic pub --once /speed_command data_interfaces/msg/SpeedCommand \
  '{target_gare: 4, ctrl_mode: 2, target_id: 1, coupling_count: 1, position: 0.95, max_forward_vel: 2.0, max_backward_vel: 1.0, max_acc: 0.5, max_dec: 1.0}'
```

ros2 bag record /arbitration_state  /auto_spd_ctrl_cmd  /ctrl_hmi  /ctrl_mag_steer /ctrl_speed /detect_result  /speed_command /ctrl_speed /speed_state /tf  /tf_static /lidar_merged/points  -o ziyun_speed_0806_i_4

ros2 bag record /arbitration_state  /arm_cmd /arm_status  /auto_spd_ctrl_cmd /chassis_info_fb  /cips/hook_task_array /cips_status  /co_arm_cmd  /co_arm_status  /ctrl_speed /detect_result  /debug/processed_points /speed_command  /speed_state -o ziyun_command_info_


ros2 run target_recognition_node target_recognition_node_exe --ros-args -p debug_mode:=true

uint8 target_gare       # 目标档位: 1=P档, 2=R档, 3=N档, 4=D档
uint8 ctrl_mode         # 运动模式: 0=刹车, 1=开始跟车, 2=微减速, 3=恢复跟随, 4=减速停车, 10=前进定速+到点, 11=后退定速+到点
uint8 target_id         # 目标车厢 ID
uint8  coupling_count   # 连挂数
float32 position        # 相对目标距离
float32 max_forward_vel # 最大前进速度 (m/s)
float32 max_backward_vel# 最大后退速度 (m/s)
float32 max_acc         # 最大前进加速度 (m/s²)
float32 max_dec         # 最大减速度 (m/s²)

### `/co_arm_cmd`
- **类型**: `std_msgs/msg/Int32MultiArray` (`[reg, val]`)
- **发布者**: `bt_executor_node` 桥接 + `SendCoSpeedArmCommand` 直接发布
- **指令映射**:

| 指令 | reg | val |
|------|-----|-----|
| RESET/HOME | 40001 | 1 |
| DEPLOY | 40001 | 3 |
| DEMAGNETIZE | 40001 | 7 |
| STOP | 40001 | 0 |
| AUTO_MODE | 40003 | 0 |
| SET_CAR_TYPE | 40008 | `train_type_code` |

```bash
ros2 topic echo /co_arm_cmd
```

### `/arm_cmd`
- **类型**: `std_msgs/msg/Int32MultiArray` (`[reg, val]`)
- **发布者**: `bt_executor_node` 桥接 + `SendMechanicalArmCommand` 直接发布
- **指令映射**:

| 指令 | reg | val |
|------|-----|-----|
| RESET | 40001 | 1 |
| DEPLOY | 40001 | 3 |
| STOP | 40001 | 0 |
| AUTO_MODE | 40002 | 0 |
| SET_CAR_TYPE | 40008 | `train_type_code` |

```bash
ros2 topic echo /arm_cmd
```

### `/cips_cmd`
- **类型**: `std_msgs/msg/Int32MultiArray` (`[reg, val]`)
- **发布者**: `bt_executor_node` 桥接 + `SendCipsCommand` 直接发布
- **指令映射**:

| 指令 | reg | val |
|------|-----|-----|
| RECEIVED | 40001 | 2 |
| REQUEST | 40001 | 4 |

```bash
ros2 topic echo /cips_cmd
```

---

## 黑板变量完整清单

### 传感器/状态输入（订阅写入，BT节点读取）

| 变量 | 类型 | 写入者 | 读取者 |
|------|------|--------|--------|
| `co_arm_40001` | int32 | `/co_arm_status` | `IsCoArmHome`, `CheckCoSpeedArm` |
| `co_arm_40002` | int32 | `/co_arm_status` | — |
| `co_arm_40003` | int32 | `/co_arm_status` | — |
| `arm_40001` | int32 | `/arm_status` | `IsArmHome`, `CheckMechanicalArm` |
| `arm_40002` | int32 | `/arm_status` | — |
| `arm_40003` | int32 | `/arm_status` | — |
| `cips_40001` | int32 | `/cips_status` | `WaitForCIPSTask` |
| `cips_task_array` | shared_ptr\<HookTaskArray\> | `/cips/hook_task_array` | `WaitForCIPSTask`, `ExtractTaskInfo` |
| `cips_total_hook_num` | int | `/cips/hook_task_array` | `publishHmiState` |
| `detect_id` | int | `/detect_result` | `WaitForTargetGap` |
| `detect_distance` | double | `/detect_result` | `IsTrackingErrorWithin` |
| `detect_velocity` | double | `/detect_result` | — |
| `speed_state` | int | `/speed_state` | `MPCTrackApproach`, `NavigateToWaitArea`, `NavigateToOrigin`, `NavigateToStaticTarget`, `IsAtTargetPose` |
| `ctrl_mag_stop` | bool | `/ctrl_mag_stop` | `performTick()` E_STOP 检查 |
| `ctrl_fence` | int | `/ctrl_fence` | `performTick()` E_STOP 检查 |
| `estop_heartbeat` | int | `performTick()` / `WaitForManualReset` | `IsHeartbeatLost`, `performTick()` E_STOP 检查 |

### 任务/配置变量（BT节点写入，其他BT节点读取）

| 变量 | 类型 | 写入者 | 读取者 |
|------|------|--------|--------|
| `task_round` | int | `WaitForCIPSTask` | `ExtractTaskInfo` |
| `remaining_tasks` | int | `WaitForCIPSTask` | `CheckRemainingTasks`, `publishHmiState` |
| `target_gap_id` | int | `WaitForCIPSTask` | `WaitForTargetGap` |
| `train_id` | int | `ExtractTaskInfo` | `SpeedControlCommand` |
| `train_type_code` | int | `ExtractTaskInfo` | `SendCoSpeedArmCommand`, `SendMechanicalArmCommand` |
| `coupling_count` | int | `ExtractTaskInfo` | `SpeedControlCommand` |
| `air_pipe_off` | bool | `ExtractTaskInfo` | — |
| `carriage_brake` | bool | `ExtractTaskInfo` | — |
| `train_number_str` | string | `ExtractTaskInfo` | — |
| `hook_lever_type` | bool | `ExtractTaskInfo` | — |
| `work_mode` | int | `handleHmiCommand()` / `performTick()` | `IsStaticMode`, `IsDynamicMode`, `publishHmiState` |
| `hmi_ctrl_mode` | int | — (外部设置) | `WaitForManualReset` |
| `ros_node` | shared_ptr\<Node\> | `performTick()` (首次tick) | `SendCoSpeedArmCommand`, `SendMechanicalArmCommand`, `SendCipsCommand` (懒初始化 publisher) |

### 静态模式变量

| 变量 | 类型 | 写入者 | 读取者 |
|------|------|--------|--------|
| `target_x` | double | `ComputeStaticTargetPose` | — |
| `target_y` | double | `ComputeStaticTargetPose` | — |
| `target_yaw` | double | `ComputeStaticTargetPose` | — |

### 命令标记（BT节点写入，bt_executor_node 桥接读取并发布）

| 黑板变量 | 类型 | 写入者 | 发布到 |
|----------|------|--------|--------|
| `speed_cmd_gare` | int32 | `setBBSpeedCmd()` | `/speed_command` (触发标记) |
| `speed_cmd_mode` | int32 | `setBBSpeedCmd()` | `/speed_command` |
| `speed_cmd_target_id` | int32 | `setBBSpeedCmd()` | `/speed_command` |
| `speed_cmd_coupling_count` | int32 | `setBBSpeedCmd()` | `/speed_command` |
| `speed_cmd_position` | double | `setBBSpeedCmd()` | `/speed_command` |
| `speed_cmd_fwd_vel` | double | `setBBSpeedCmd()` | `/speed_command` |
| `speed_cmd_bwd_vel` | double | `setBBSpeedCmd()` | `/speed_command` |
| `speed_cmd_acc` | double | `setBBSpeedCmd()` | `/speed_command` |
| `speed_cmd_dec` | double | `setBBSpeedCmd()` | `/speed_command` |
| `co_arm_cmd_reg` | int32 | 桥接内部 | `/co_arm_cmd` |
| `co_arm_cmd_val` | int32 | 桥接内部 | `/co_arm_cmd` |
| `arm_cmd_reg` | int32 | `SendArmCommand` | `/arm_cmd` |
| `arm_cmd_val` | int32 | `SendArmCommand` | `/arm_cmd` |
| `cips_cmd_reg` | int32 | 桥接内部 | `/cips_cmd` |
| `cips_cmd_val` | int32 | 桥接内部 | `/cips_cmd` |

### Mock 测试变量

| 变量 | 类型 | 写入者 | 读取者 |
|------|------|--------|--------|
| `mock_<NodeName>` | int | `bt_mock_gui.py` | `checkMockStatus()` (所有 BT 节点) |

---

## 数据流总览

```
                      LISTENING (订阅)
                      ================

[CIPS]          /cips/hook_task_array  →  cips_total_hook_num, cips_task_array
[CIPS]          /cips_status           →  cips_40001
[Perception]    /detect_result          →  detect_id, detect_distance, detect_velocity
[HMI]           /ctrl_hmi               →  work_mode (间接)
[Co-Arm]        /co_arm_status          →  co_arm_40001/2/3
[Arm]           /arm_status             →  arm_40001/2/3
[Safety]        /ctrl_mag_stop          →  ctrl_mag_stop
[Fence]         /ctrl_fence             →  ctrl_fence
[Chassis]       /speed_state            →  speed_state

                      PUBLISHING (发布)
                      =================

bt_executor  →  /bt_hmi_state    (10 Hz)       →  HMI 显示
bt_executor  →  /co_arm_cmd      (event)       →  Co-arm 控制器
robot_nodes  →  /co_arm_cmd      (event)       →  Co-arm 控制器 (直接)
bt_executor  →  /arm_cmd         (event)       →  Arm 控制器
robot_nodes  →  /arm_cmd         (event)       →  Arm 控制器 (直接)
bt_executor  →  /speed_command   (event)       →  底盘速度控制器
bt_executor  →  /cips_cmd        (event)       →  CIPS 平台
robot_nodes  →  /cips_cmd        (event)       →  CIPS 平台 (直接)
```

---

## 典型测试序列

```bash
# 1. 注入 CIPS 任务
ros2 topic pub --once /cips/hook_task_array uncoupling_robot_bt/msg/HookTaskArray \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "cips"}, total_hook_num: 1, hook_tasks: [
    {train_id: 1, train_type_code: 1, coupling_count: 0, air_pipe_off: false, carriage_brake: false, train_number_str: "C1", hook_lever_type: false}
  ]}'

# 2. 触发任务开始
ros2 topic pub --once /cips_status std_msgs/msg/Int32MultiArray '{data: [40001, 1]}'

# 3. 模拟 co-arm 复位
ros2 topic pub --once /co_arm_status std_msgs/msg/Float32MultiArray '{data: [40001.0, 2.0]}'

# 4. 模拟 arm 复位
ros2 topic pub --once /arm_status std_msgs/msg/Int32MultiArray '{data: [40001, 2]}'

# 5. 模拟检测到目标车厢
ros2 topic pub --once /detect_result uncoupling_robot_bt/msg/DetectResult \
  '{current_id: 1, current_distance: 1.25, relative_velocity: 0.5}'

# 6. 模拟速度跟踪完成
ros2 topic pub --once /speed_state std_msgs/msg/UInt8 '{data: 2}'

# 7. 急停测试
ros2 topic pub --once /ctrl_mag_stop std_msgs/msg/Bool '{data: true}'

# 8. 解除急停
ros2 topic pub --once /ctrl_hmi uncoupling_robot_bt/msg/HmiCommand \
  '{hmi_ctrl_mode: 3, ctrl_gare: 4, ctrl_speed: 0.0, ctrl_break: 0}'

# 9. 查看 BT 状态
ros2 topic echo /bt_hmi_state
```


```bash
# 1. 注入 CIPS 任务
ros2 topic pub --once /cips/hook_task_array data_interfaces/msg/HookTaskArray \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "cips"}, total_hook_num: 1, hook_tasks: [
    {train_id: 1, train_type_code: 1, coupling_count: 0, air_pipe_off: false, carriage_brake: false, train_number_str: "C1", hook_lever_type: false}
  ]}'

# 2. 触发任务开始
ros2 topic pub --once /cips_status std_msgs/msg/Int32MultiArray '{data: [40001, 1]}'

# 3. 模拟 co-arm 复位
ros2 topic pub --once /co_arm_status std_msgs/msg/Float32MultiArray '{data: [40001.0, 2.0]}'

# 4. 模拟 arm 复位
ros2 topic pub --once /arm_status std_msgs/msg/Int32MultiArray '{data: [40001, 2]}'

# 5. 模拟检测到目标车厢
ros2 topic pub --once /detect_result data_interfaces/msg/DetectResult \
  '{current_id: 0, current_distance: 1.25, relative_velocity: 0.5}'

# 6. 模拟速度跟踪完成
ros2 topic pub --once /speed_state std_msgs/msg/UInt8 '{data: 2}'

# 7. 急停测试
ros2 topic pub --once /ctrl_mag_stop std_msgs/msg/Bool '{data: true}'

# 8. 解除急停
ros2 topic pub --once /ctrl_hmi data_interfaces/msg/HmiCommand \
  '{hmi_ctrl_mode: 3, ctrl_gare: 4, ctrl_speed: 0.0, ctrl_break: 0}'

# 9. 查看 BT 状态
ros2 topic echo /bt_hmi_state
```