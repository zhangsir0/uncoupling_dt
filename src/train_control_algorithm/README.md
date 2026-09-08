# train_control_algorithm — 纵向速度跟踪控制

位置 PID + 状态机的速度跟踪控制，用于摘钩机器人底盘跟车闭环。

---

## 一、功能介绍

### 1.1 节点

| 节点 | 可执行文件 | 说明 |
|------|-----------|------|
| `speed_track_node` | `speed_track_node` | **控制节点**：状态机 + 位置 PID，发布速度/刹车/档位指令 |
| `test_upstream_node` | `test_upstream_node` | **测试桥接**：仿真用，模拟上游数据（Gazebo odom → 感知/底盘话题） |

> `test_upstream_node` 仅仿真测试用，真车不上。以下主要介绍 `speed_track_node`。

### 1.2 控制律

```
speed_cmd = target_speed + position_PID(position_error)

其中：
  target_speed  = ego_speed + relative_velocity
  position_error = relative_distance - position_offset

  position_PID = k_p_pos × e  +  k_i_pos × ∫e·dt  +  k_d_pos × de/dt
                  (积分抗饱和 ±1.0 m·s)

输出经加速度限制 [max_decel, max_accel] 和速度限幅 [min_speed, max_speed] 后发布。
```

### 1.3 数据流

```
行为树                          感知端                       底盘
  │                               │                           │
  │  /speed_command               │  /detect_result            │  /chassis_info_fb
  │  SpeedCommand                  │  DetectResult              │  ChassisInfoFb
  │  ├─ ctrl_mode     模式        │  ├─ current_id 检测车厢号   │  └─ ctrl_fb
  │  ├─ target_id     目标车厢    │  ├─ current_distance 距离   │     .ctrl_fb_velocity
  │  ├─ position      位置修正    │  └─ relative_velocity 相对速度│    自车速度 (m/s)
  │  ├─ max_forward_vel           │                             │
  │  ├─ max_backward_vel          │                             │
  │  ├─ max_acc                   │                             │
  │  └─ max_dec                   │                             │
  │                               │                             │
  ▼                               ▼                             ▼
┌──────────────────────────────────────────────────────────────────┐
│                      speed_track_node (50Hz)                     │
│                                                                  │
│  ctrl_mode → 状态机切换 (0/1/2/3/4/10/11)                       │
│  target_id vs current_id → ID 匹配 (不匹配不执行 PID)            │
│  max_* → 动态覆盖限幅参数                                        │
│                                                                  │
│  控制律: speed_cmd = target_speed + position_PID(position_error) │
│                                                                  │
└────────────────────────────┬─────────────────────────────────────┘
                             │
               /ctrl_speed (CtrlSpeed, 50Hz)
               ├─ ctrl_speed   速度指令 (m/s)
               ├─ ctrl_break   刹车 0–100
               └─ ctrl_gare    档位

               /speed_state (UInt8)
               └─ 1=跟车中 / 2=共速 / 3=停车
```

### 1.4 输入话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/speed_command` | `SpeedCommand` | 行为树下发：模式、目标、限幅 |
| `/detect_result` | `DetectResult` | 感知端反馈：车厢号、距离、相对速度 |
| `/chassis_info_fb` | `ChassisInfoFb` | 底盘反馈：自车速度 (`ctrl_fb.ctrl_fb_velocity`) |

**SpeedCommand 字段**

| 字段 | 类型 | 说明 |
|------|------|------|
| `target_gare` | uint8 | 目标档位: 1=P, 2=R, 3=N, 4=D |
| `ctrl_mode` | uint8 | 模式 0/1/2/3/4/10/11 |
| `target_id` | uint8 | 目标车厢号（与 current_id 匹配后才执行 PID） |
| `coupling_count` | uint8 | 连挂数 |
| `position` | float32 | 相对目标距离 (m) |
| `max_forward_vel` | float32 | 前进速度上限 (m/s) |
| `max_backward_vel` | float32 | 后退速度上限 (m/s) |
| `max_acc` | float32 | 加速度上限 (m/s²) |
| `max_dec` | float32 | 减速度上限 (m/s²) |

**DetectResult 字段**

| 字段 | 类型 | 说明 |
|------|------|------|
| `current_id` | uint8 | 检测到的车厢号 |
| `current_distance` | float32 | 雷达→脱钩距离 (m，带符号) |
| `relative_velocity` | float32 | 车厢相对小车速度 (m/s) |

### 1.5 输出话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/ctrl_speed` | `CtrlSpeed` | 50Hz | 速度+刹车+档位，发给底盘仲裁 |
| `/speed_state` | `UInt8` | 事件 | 1=跟车中 / 2=共速 / 3=停车 |

### 1.6 状态机模式

| 模式 | 名称 | 行为 | PID |
|:---:|------|------|:---:|
| 0 | 刹车 | 速度置零，输出刹车值 | |
| 1 | 开始跟车 | 位置 PID 跟车（ID 匹配后执行，禁止倒车） | ✓ |
| 2 | 微调 | PID 输出 + `task2_vel_offset` | ✓ |
| 3 | 恢复跟车 | 等同模式 1 | ✓ |
| 4 | 停车 | 以 `task4_dec_valude` 平缓减速，速度降至 `cruise_vel_threshold` 以下直接刹车 | |
| 10 | 前进巡航+定点 | 定速前进，检测到目标后留在本模式用宽松阈值 PID 逼近，满足即刹车 | ✓* |
| 11 | 后运动+定点 | 定速后退，检测到目标后留在本模式用宽松阈值 PID 逼近，满足即刹车 | ✓* |

> \*模式 10/11 逼近阶段启用 PID，收敛阈值使用 `cruise_pos_threshold` / `cruise_vel_threshold`，比模式 1 的紧阈值宽松。

### 1.7 车辆状态反馈

| speed_state | 含义 | 判定 |
|:---:|------|------|
| 1 | 跟车中 | 正在调整 |
| 2 | 共速 | \|position_error\|<阈值 且 \|速度差\|<阈值 持续 stable_duration 秒 |
| 3 | 停车 | \|speed_cmd\| < 0.01 |

> `position_error = relative_distance - position_offset`，与控制律中的误差量保持一致。

---

## 二、使用方式

### 2.1 前置依赖

**源码组成**（`src/` 下两个包）：

```
src/
├── data_interfaces/           # 自定义消息（SpeedCommand / DetectResult / CtrlSpeed）
└── train_control_algorithm/   # PID 控制节点 + 测试桥接
```

**第三方依赖**：`yhs_can_interfaces`（ChassisInfoFb 消息定义），来自 FR-mega 底盘 CAN 接口包，需自行放入工作空间的 `src/` 下编译。

**运行依赖**：上游必须提供三个话题：

| 话题 | 类型 | 来源 |
|------|------|------|
| `/speed_command` | `SpeedCommand` | 行为树 |
| `/detect_result` | `DetectResult` | 感知节点 |
| `/chassis_info_fb` | `ChassisInfoFb` | 底盘 CAN |

输出 `/ctrl_speed` 发给底盘仲裁器。

### 2.2 首次构建

```bash
cd <your_ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2.3 启动命令

需要 **3 个终端**：

```bash
# === 终端 1：启动仿真(实机不用) ===
source /opt/ros/humble/setup.bash
source install/setup.bash
source env.sh
ros2 launch train_sim train_sim.launch.py x1_initial_x:=-5.0 x1_initial_y:=-380.0

# === 终端 2：启动 PID 控制节点 ===
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/zengxianwei/Desktop/work_resource/project_resource/code_folder/colcon_ws_dt_20260622/colcon_ws/install/setup.zsh
ros2 run train_control_algorithm speed_track_node

# === 终端 3（模拟上游数据-测试使用） ===
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/zengxianwei/Desktop/work_resource/project_resource/code_folder/colcon_ws_dt_20260622/colcon_ws/install/setup.zsh
ros2 run train_control_algorithm test_upstream_node


# === 终端 4（控制仿真-测试使用） ===
# Gazebo列车出发
source /opt/ros/humble/setup.bash
ros2 topic pub --once /train/speed std_msgs/msg/Float32 "{data: 1.0}"

# 切换模式
ros2 param set /test_upstream_node target_mode 1    # 跟车
ros2 param set /test_upstream_node target_mode 10   # 前进巡航
ros2 param set /test_upstream_node target_mode 0    # 刹车

# 监控
ros2 topic echo /ctrl_speed
ros2 topic echo /speed_state
```

### 2.3 测试流程

```bash
# ① 列车出发
ros2 topic pub --once /train/speed std_msgs/msg/Float32 "{data: 2.0}"

# ② X1 前进巡航（检测到车厢自动切跟车）
ros2 param set /test_upstream_node target_mode 10

# ③ 跟车
ros2 param set /test_upstream_node target_mode 1

# ④ 微调
ros2 param set /test_upstream_node target_mode 2

# ⑤ 停车
ros2 param set /test_upstream_node target_mode 4

# ⑥ 切换跟踪目标
ros2 param set /test_upstream_node target_carriage_id 3
```

---

## 三、动态调参

参数无需重启节点，两种方式调参：

### 3.1 rqt 图形界面（推荐）

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

选择 `/speed_track_node`，滑块拖动，实时生效。

### 3.2 命令行

```bash
ros2 param set /speed_track_node <参数名> <值>
```

### 3.3 参数表

**PID 系数**（归属 controller，0.02s 下个周期生效）

| 参数 | 默认 | 范围 | 步长 | 说明 |
|------|------|------|------|------|
| `k_p_pos` | 0.3 | 0.0–5.0 | 0.01 | 比例系数 |
| `k_i_pos` | 0.01 | 0.0–2.0 | 0.001 | 积分系数 |
| `k_d_pos` | 0.05 | 0.0–2.0 | 0.01 | 微分系数 |

**限幅**（归属 controller）

| 参数 | 默认 | 范围 | 步长 | 说明 |
|------|------|------|------|------|
| `max_accel` | 1.5 | 0.1–10.0 | 0.1 | 加速度上限 (m/s²) |
| `max_decel` | -2.0 | -10.0…-0.1 | 0.1 | 减速度下限 (m/s²) |
| `max_speed` | 5.0 | 0.1–20.0 | 0.1 | 输出速度上限 (m/s) |
| `min_speed` | -1.0 | -20.0…-0.1 | 0.1 | 输出速度下限 (m/s) |

> `max_speed`/`min_speed` 会被 SpeedCommand 中的 `max_forward_vel`/`max_backward_vel` 动态覆盖。

**业务参数**（归属 local）

| 参数 | 默认 | 范围 | 步长 | 说明 |
|------|------|------|------|------|
| `task2_vel_offset` | -0.2 | -2.0–0.0 | 0.05 | 模式 2 微减速偏移 (m/s) |
| `task4_dec_valude` | -0.4 | -2.0–0.0 | 0.05 | 模式 4 停车减速度 (m/s²) |
| `task10_forward_vel` | 0.5 | 0.1–5.0 | 0.1 | 模式 10 前进巡航速度 (m/s) |
| `task11_backward_vel` | -0.5 | -5.0…-0.1 | 0.1 | 模式 11 后退巡航速度 (m/s) |
| `task0_brake_value` | 50 | 0–100 | 1.0 | 模式 0 刹车力度 |

**稳定性判定**（归属 local）

| 参数 | 默认 | 范围 | 步长 | 说明 |
|------|------|------|------|------|
| `stable_pos_threshold` | 0.1 | 0.01–2.0 | 0.01 | 位置稳定阈值 (m) |
| `stable_vel_threshold` | 0.05 | 0.01–1.0 | 0.01 | 速度稳定阈值 (m/s) |
| `stable_duration` | 3.0 | 1.0–10.0 | 0.5 | 稳定判定持续时间 (s) |

**巡航定点快速收敛**（归属 local）

| 参数 | 默认 | 范围 | 步长 | 说明 |
|------|------|------|------|------|
| `cruise_pos_threshold` | 0.15 | 0.01–1.0 | 0.01 | 模式10/11 逼近位置宽阈值 (m) |
| `cruise_vel_threshold` | 0.5 | 0.01–2.0 | 0.01 | 模式10/11 逼近 + 模式4 停车末端速度宽阈值 (m/s) |

### 3.4 调参示例

```bash
# 加大 P，响应更快
ros2 param set /speed_track_node k_p_pos 0.8

# 加大积分，消除稳态误差
ros2 param set /speed_track_node k_i_pos 0.03

# 限制最高速度
ros2 param set /speed_track_node max_speed 3.0

# 放宽稳定判定
ros2 param set /speed_track_node stable_duration 2.0

# 查看当前所有参数
ros2 param list /speed_track_node
ros2 param get /speed_track_node k_p_pos
```
