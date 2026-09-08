# 到点控制节点 (PositionController) 设计文档 v2

## 1. 概述

### 1.1 功能定位

`position_controller_node` 是一个独立的 ROS2 C++ 节点，负责控制底盘从当前位置行驶到站场内预定义的固定目标位置（到点控制）。该节点服务于行为树 `S3_NAV_WAIT_AREA` 子树，实现"前往待位区"的自动导航。

### 1.2 核心功能

- 接收 `/speed_command` 的 `ctrl_mode`（≥20 时激活本节点）和 `coupling_count`（1~50 索引）
- 从外部 CSV 文件加载 `coupling_count → 目标位置` 的映射表
- 从 fixposition 获取融合定位信息（里程计速度 + 全局 LLH 坐标）
- 根据参考点+方位角+距离计算目标 WGS84 坐标
- 计算当前位置到目标位置的距离，规划梯形速度曲线
- 输出底盘速度控制指令 `/ctrl_position`（SpdCtrlCmd）
- 到达目标位置后自动刹车，发布到达状态

### 1.3 ctrl_mode 职责划分

| ctrl_mode 范围 | 负责节点 | 说明 |
|----------------|----------|------|
| `1 ~ 4` | `speed_ctrl` 节点（跟车控制） | 本节点忽略 |
| `10 ~ 11` | `speed_ctrl` 节点（定速到点） | 本节点忽略 |
| `0` | **所有节点** | 通用刹车/停止指令 |
| `≥ 20` | **position_controller_node** | 到点导航控制 |

> **本节点只响应 `ctrl_mode ∈ {0} ∪ [20, ∞)` 的指令。`ctrl_mode < 20` 且 `≠ 0` 的消息由其他节点处理，本节点忽略。**

---

## 2. 数据流

```
/speed_command (SpeedCommand)──────┐
   ├── ctrl_mode (≥20 激活)        │
   └── coupling_count (1~50)       │
                                   ├──► position_controller_node
/fixposition/odometry (Odometry)───┤         │
   └── twist.twist (速度反馈)      │         │  目标坐标计算
                                   │         │  + 距离计算
/fixposition/llh (FpaLlh)──────────┘         │  + 梯形速度规划
   └── position.x (lat)                      │
   └── position.y (lon)                      ▼
   └── position.z (height)          /ctrl_position (SpdCtrlCmd)
                                         ├── ctrl_gare  (1/2/3/4)
              ┌───────────────────────────┤
              │                           ├── ctrl_speed (m/s)
   targets.csv (coupling_count → 目标)    └── ctrl_break (0-100)
```

### 2.1 输入话题

| 话题 | 消息类型 | 频率 | 用途 |
|------|----------|------|------|
| `/speed_command` | `uncoupling_robot_bt/msg/SpeedCommand` | 事件驱动 | `ctrl_mode` 控制启停（≥20 激活），`coupling_count` 索引目标 |
| `/fixposition/odometry` | `nav_msgs/msg/Odometry` | ~50Hz | 融合里程计，提供当前速度 `twist.twist.linear` |
| `/fixposition/llh` | `fixposition_driver_msgs/msg/FpaLlh` | ~10Hz | WGS84 经纬度+海拔（`position.x/y/z`），用于绝对位置计算 |

### 2.2 输出话题

| 话题 | 消息类型 | 频率 | 用途 |
|------|----------|------|------|
| `/ctrl_position` | `tigou_interfaces/msg/SpdCtrlCmd` | 50Hz | 底盘档位+速度+刹车指令 |
| `/ctrl_position/status` | `std_msgs/msg/String` | 10Hz | 状态反馈: `"IDLE"`, `"MOVING"`, `"ARRIVED"` |
| `/ctrl_position/distance` | `std_msgs/msg/Float64` | 10Hz | 距目标点的实时距离 (m) |

### 2.3 SpdCtrlCmd 消息定义

```msg
uint8   ctrl_gare      # 1=P, 2=R, 3=N, 4=D
float32 ctrl_speed     # m/s, 正值=前进, 负值=后退
uint8   ctrl_break     # 0-100
```

---

## 3. 目标位置配置文件

### 3.1 文件路径

```
uncoupling_robot_bt/config/targets.csv
```

运行时通过 ROS2 参数 `targets_file` 指定，默认值：
```
targets_file: "config/targets.csv"
```
（相对于节点工作目录或通过 launch 文件传入绝对路径）

### 3.2 CSV 格式

支持两种格式，由列数自动识别：

#### 格式A: 参考点 + 方位角 + 距离（6 列）

```csv
# coupling_count, ref_lat, ref_lon, ref_height, azimuth_deg, distance_m
# azimuth_deg: 从参考点出发的方位角, 0=北, 90=东
# distance_m: 沿方位角方向的偏移距离 (米)
1, 31.2304000, 121.4737000, 4.5, 15.3, 8.5
2, 31.2304000, 121.4737000, 4.5, 15.3, 17.0
3, 31.2304000, 121.4737000, 4.5, 15.3, 25.5
...
50, 31.2304000, 121.4737000, 4.5, 345.0, 403.0
```

#### 格式B: 双参考点 + 距离（8 列，推荐）

```csv
# coupling_count, ptA_lat, ptA_lon, ptA_h, ptB_lat, ptB_lon, ptB_h, distance_m
# ptA → ptB 的方向为方位线, 目标位置 = ptA + distance_m * 单位方向向量
# 优点: 不需要手动测量方位角, 只需记录两个 GPS 点
1, 31.2304000, 121.4737000, 4.5, 31.2304500, 121.4737200, 4.5, 8.5
2, 31.2304000, 121.4737000, 4.5, 31.2304500, 121.4737200, 4.5, 17.0
3, 31.2304000, 121.4737000, 4.5, 31.2304500, 121.4737200, 4.5, 25.5
...
50, 31.2304000, 121.4737000, 4.5, 31.2304500, 121.4737200, 4.5, 403.0
```

### 3.3 格式说明

| 字段 | 单位 | 说明 |
|------|------|------|
| `coupling_count` | — | 连挂数 1~50，作为索引键 |
| `ptA_lat / ptA_lon` | 度 (WGS84) | 参考起点 A 的经纬度 |
| `ptA_h` | 米 | 参考起点 A 的椭球高 |
| `ptB_lat / ptB_lon` | 度 (WGS84) | 参考终点 B 的经纬度（格式B 专用） |
| `ptB_h` | 米 | 参考终点 B 的椭球高（格式B 专用） |
| `azimuth_deg` | 度 | 方位角，0=北，90=东，180=南，270=西（格式A 专用） |
| `distance_m` | 米 | 从参考点 A 沿方位线方向的偏移距离 |

### 3.4 目标位置计算逻辑

```cpp
// ── 步骤1: 根据 coupling_count 查找对应的行 ──
struct TargetRow {
  int    count;
  double ref_lat, ref_lon, ref_h;   // 参考点 A (两种格式共用)
  double azimuth_deg;               // 方位角 (格式A 直接读取; 格式B 由 ptA→ptB 计算)
  double distance_m;                // 偏移距离
};

// 查找: 遍历 CSV 行, 匹配 coupling_count
// 未找到时: 报错 + 保持刹车

// ── 步骤2: 计算方位角 (仅格式B) ──
// 使用 bearing 公式从 ptA → ptB 计算:
double bearing = bearingTo(ptA_lat, ptA_lon, ptB_lat, ptB_lon);  // 弧度, 0=北

// ── 步骤3: 根据参考点+方位角+距离计算目标坐标 ──
// "给定起点、方位角、距离，求终点坐标" (Destination point given distance and bearing)
double angular_dist = distance_m / R_EARTH;  // 角距离 (弧度)
double target_lat = asin(
    sin(ref_lat_rad) * cos(angular_dist) +
    cos(ref_lat_rad) * sin(angular_dist) * cos(azimuth_rad)
);
double target_lon = ref_lon_rad + atan2(
    sin(azimuth_rad) * sin(angular_dist) * cos(ref_lat_rad),
    cos(angular_dist) - sin(ref_lat_rad) * sin(target_lat)
);
double target_h = ref_h;  // 高度直接使用参考点高度
```

### 3.5 CSV 解析实现

```cpp
// 使用 C++ 标准库逐行解析, 无需第三方依赖
// 忽略以 '#' 开头的注释行和空行
// 自动检测列数 (line.split(',') 或手动解析):
//   7 列 = 格式A (coupling_count + lat + lon + h + azimuth + distance)
//   9 列 = 格式B (coupling_count + latA + lonA + hA + latB + lonB + hB + distance)
//
// 构建 std::unordered_map<int, TargetRow> 用于 O(1) 查找

std::unordered_map<int, TargetRow> targets_map_;

bool loadTargetsFile(const std::string& filepath) {
  std::ifstream f(filepath);
  if (!f.is_open()) { RCLCPP_ERROR(...); return false; }

  std::string line;
  int lineno = 0;
  while (std::getline(f, line)) {
    lineno++;
    // 去除首尾空白
    // 跳过空行和 '#' 注释行
    if (line.empty() || line[0] == '#') continue;

    auto cols = splitCSV(line);  // 按逗号分割
    TargetRow row;

    if (cols.size() == 7) {
      // 格式A: count, lat, lon, h, azimuth, distance
      row.count      = std::stoi(cols[0]);
      row.ref_lat    = std::stod(cols[1]);
      row.ref_lon    = std::stod(cols[2]);
      row.ref_h      = std::stod(cols[3]);
      row.azimuth_deg = std::stod(cols[4]);
      row.distance_m  = std::stod(cols[5]);
    } else if (cols.size() == 9) {
      // 格式B: count, latA, lonA, hA, latB, lonB, hB, distance
      double latB = std::stod(cols[4]);
      double lonB = std::stod(cols[5]);
      row.count      = std::stoi(cols[0]);
      row.ref_lat    = std::stod(cols[1]);
      row.ref_lon    = std::stod(cols[2]);
      row.ref_h      = std::stod(cols[3]);
      row.azimuth_deg = bearingTo(row.ref_lat, row.ref_lon, latB, lonB) * 180.0 / M_PI;
      row.distance_m  = std::stod(cols[7]);
    } else {
      RCLCPP_WARN(get_logger(), "第%d行: 列数=%zu, 跳过", lineno, cols.size());
      continue;
    }

    targets_map_[row.count] = row;
  }

  RCLCPP_INFO(get_logger(), "已加载 %zu 个目标位置", targets_map_.size());
  return !targets_map_.empty();
}
```

---

## 4. ctrl_mode 语义

| ctrl_mode | 本节点行为 |
|-----------|-----------|
| `0` | 刹车停止（gear=P, speed=0, brake=100）→ 回到 IDLE |
| `1~19` | **忽略**（由 `speed_ctrl` 节点处理） |
| `20` | 前进行驶到 coupling_count 对应的目标位置（S3 专用） |
| `21` | （预留）后退行驶到 coupling_count 对应的目标位置 |
| `22+` | （预留）未来扩展 |

> **到达后自动刹车**: 当 distance ≤ threshold 且稳定 arrival_duration 秒后，无论 ctrl_mode 仍为 20 还是其他值，节点自动进入 ARRIVED 状态并刹车。

---

## 5. 控制算法

### 5.1 距离计算

使用 **Haversine** 公式计算当前位置到目标位置的球面距离：

```
a = sin²(Δlat/2) + cos(lat1) * cos(lat2) * sin²(Δlon/2)
c = 2 * atan2(√a, √(1−a))
distance = R_EARTH * c      # R_EARTH = 6371000.0 m
```

### 5.2 速度规划（梯形速度曲线）

```
              max_vel ───┐         ┌───
                        │ 匀速段  │
                       ╱          ╲
  加速段              ╱            ╲              减速段
  (max_acc)          ╱              ╲            (max_dec)
 ──────────────────╱────────────────╲──────────────► 距离
 0              acc_end          dec_start      target
```

```cpp
double computeSpeedCmd(double distance, double current_speed, double dt,
                       double max_vel, double max_acc, double max_dec,
                       double threshold) {
  // 1. 到达判定
  if (distance <= threshold) return 0.0;

  // 2. 减速距离: 从当前速度减到 0 所需距离
  double decel_dist = (current_speed * current_speed) / (2.0 * max_dec);

  // 3. 速度指令
  double target_speed;
  if (distance <= decel_dist + threshold) {
    // 减速段: 确保在到达阈值前停止
    target_speed = std::sqrt(2.0 * max_dec * (distance - threshold));
  } else if (current_speed < max_vel) {
    // 加速段
    target_speed = current_speed + max_acc * dt;
  } else {
    // 匀速段
    target_speed = max_vel;
  }

  return std::clamp(target_speed, 0.0, max_vel);
}
```

### 5.3 状态机

```
                  ctrl_mode=0
          ┌─────────────────────────────┐
          │                             │
          ▼                             │
    ┌──────────┐   ctrl_mode ≥ 20   ┌───┴──────┐
    │  IDLE    │ ──────────────────► │  MOVING  │
    │  (刹车)  │                     │  (前进)  │
    └──────────┘                     └──┬───┬───┘
          ▲                             │   │
          │      distance ≤ threshold   │   │
          │   && stable ≥ duration      │   │
          └─────────────────────────────┘   │
                 ARRIVED (自动刹车)          │
                                            │
                  ctrl_mode=0 (手动停止)─────┘
```

| 状态 | 条件 | 输出 |
|------|------|------|
| `IDLE` | 初始状态 或 `ctrl_mode == 0` | gear=P, speed=0, brake=100 |
| `MOVING` | `ctrl_mode ≥ 20` 且 distance > threshold | gear=D, speed=计算值, brake=0 |
| `ARRIVED` | distance ≤ threshold 持续 arrival_duration 秒 | gear=P, speed=0, brake=100 |

**状态切换沿处理**:
- IDLE → MOVING: 根据 coupling_count 重新查找目标行，计算目标坐标
- MOVING → ARRIVED: 发布 `/ctrl_position/status = "ARRIVED"`（bt_executor_node 桥接至黑板 speed_state=3）
- MOVING → IDLE: ctrl_mode 变为 0 时手动停止
- ARRIVED → MOVING: ctrl_mode 再次 ≥ 20 且 coupling_count 变化时重新出发

---

## 6. 节点架构

### 6.1 类设计

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "fixposition_driver_msgs/msg/fpa_llh.hpp"
#include "uncoupling_robot_bt/msg/speed_command.hpp"
#include "tigou_interfaces/msg/spd_ctrl_cmd.hpp"

class PositionControllerNode : public rclcpp::Node
{
public:
  PositionControllerNode();

private:
  // ── CSV 目标文件加载 ──
  struct TargetRow {
    int    count;
    double ref_lat, ref_lon, ref_h;
    double azimuth_deg;
    double distance_m;
  };
  bool loadTargetsFile(const std::string& filepath);
  bool lookupTarget(int coupling_count, TargetRow& row);

  // ── 坐标计算 ──
  double haversineDistance(double lat1, double lon1, double lat2, double lon2);
  double bearingTo(double lat1, double lon1, double lat2, double lon2);
  void   computeTargetLLH(const TargetRow& row,
                          double& tgt_lat, double& tgt_lon, double& tgt_h);

  // ── 订阅回调 ──
  void speedCmdCallback(const SpeedCommand::SharedPtr msg);
  void odomCallback(const Odometry::SharedPtr msg);
  void llhCallback(const FpaLlh::SharedPtr msg);

  // ── 控制循环 (timer 驱动, 50Hz) ──
  void controlLoop();

  // ── 速度规划 ──
  double computeSpeedCmd(double distance, double cur_speed, double dt);
  void   publishStop();
  void   publishCmd(int gear, double speed);
  void   publishStatus(const std::string& status, double distance);

  // ── 订阅 ──
  rclcpp::Subscription<SpeedCommand>::SharedPtr          speed_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<FpaLlh>::SharedPtr                 llh_sub_;

  // ── 发布 ──
  rclcpp::Publisher<SpdCtrlCmd>::SharedPtr     ctrl_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_pub_;
  rclcpp::TimerBase::SharedPtr                 control_timer_;
  rclcpp::TimerBase::SharedPtr                 status_timer_;  // 10Hz

  // ── 目标映射表 ──
  std::unordered_map<int, TargetRow> targets_map_;

  // ── 定位数据 ──
  double current_lat_ = 0.0, current_lon_ = 0.0, current_height_ = 0.0;
  double current_speed_ = 0.0;   // 合速度 (m/s), 来自 odometry
  bool   llh_valid_ = false;

  // ── 当前目标 ──
  double target_lat_ = 0.0, target_lon_ = 0.0, target_height_ = 0.0;
  int    target_count_ = -1;

  // ── 控制参数 ──
  double max_forward_vel_   = 2.0;
  double max_acc_           = 0.5;
  double max_dec_           = 1.0;
  double arrival_threshold_ = 0.15;
  double arrival_duration_  = 2.0;
  double control_rate_      = 50.0;

  // ── 状态 ──
  enum State { IDLE, MOVING, ARRIVED };
  State  state_            = IDLE;
  int    ctrl_mode_        = 0;
  int    coupling_count_   = 0;
  bool   active_           = false;  // ctrl_mode >= 20
  double arrival_timer_    = 0.0;
};
```

### 6.2 控制循环主体

```cpp
void PositionControllerNode::controlLoop()
{
  double dt = 1.0 / control_rate_;

  // ── ctrl_mode 过滤: < 20 且 ≠ 0 的消息忽略 ──
  if (ctrl_mode_ > 0 && ctrl_mode_ < 20) {
    // 不归本节点处理 (speed_ctrl 的 domain)
    return;
  }

  // ── mode=0 或非 active → 刹车 ──
  if (ctrl_mode_ == 0 || !active_) {
    if (state_ != IDLE) {
      state_ = IDLE;
      RCLCPP_INFO(get_logger(), "状态: IDLE (刹车)");
    }
    publishStop();
    return;
  }

  // ── 重新查找目标 (coupling_count 变化时) ──
  if (target_count_ != coupling_count_) {
    TargetRow row;
    if (!lookupTarget(coupling_count_, row)) {
      RCLCPP_ERROR(get_logger(),
        "coupling_count=%d 在 targets.csv 中未找到, 保持刹车", coupling_count_);
      publishStop();
      return;
    }
    computeTargetLLH(row, target_lat_, target_lon_, target_height_);
    target_count_ = coupling_count_;
    arrival_timer_ = 0.0;
    state_ = MOVING;
    RCLCPP_INFO(get_logger(),
      "目标: count=%d → (%.7f, %.7f, %.2f) azimuth=%.1f° distance=%.1fm",
      target_count_, target_lat_, target_lon_, target_height_,
      row.azimuth_deg, row.distance_m);
  }

  // ── 定位检查 ──
  if (!llh_valid_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "等待 /fixposition/llh 定位数据...");
    publishStop();
    return;
  }

  // ── 距离计算 ──
  double distance = haversineDistance(current_lat_, current_lon_,
                                       target_lat_, target_lon_);

  // ── 到达判定 ──
  if (distance <= arrival_threshold_) {
    arrival_timer_ += dt;
    if (arrival_timer_ >= arrival_duration_) {
      if (state_ != ARRIVED) {
        RCLCPP_INFO(get_logger(), "已到达! 距离=%.3fm", distance);
        state_ = ARRIVED;
      }
      publishStop();
      return;
    }
  } else {
    arrival_timer_ = 0.0;
    state_ = MOVING;
  }

  // ── 速度计算 + 发布 ──
  double speed = computeSpeedCmd(distance, current_speed_, dt,
                                  max_forward_vel_, max_acc_, max_dec_,
                                  arrival_threshold_);
  publishCmd(4, speed);  // gear=D (前进)
}
```

---

## 7. 与行为树 S3_NAV_WAIT_AREA 的交互

### 7.1 BT XML 设计

```xml
<BehaviorTree ID="S3_NAV_WAIT_AREA">
  <Sequence name="nav_to_wait_area">
    <!-- ① 下发到点指令: ctrl_mode=20 激活 position_controller_node
         coupling_count 自动从黑板读取 -->
    <Action ID="SpeedControlCommand" gear="4" ctrl_mode="20"
            max_forward_vel="2.0" max_acc="0.5" max_dec="1.0"/>

    <!-- ② 等待到达: speed_state==3 (ARRIVED)
         position_controller_node → /ctrl_position/status "ARRIVED"
         → bt_executor_node 桥接 → 黑板 speed_state=3 -->
    <Action ID="CheckCtrlStatus" command="3" timeout="120"/>

    <!-- ③ 到达后刹车 (写黑板 → bridge 发布 ctrl_mode=0) -->
    <Action ID="SpeedControlCommand" gear="4" ctrl_mode="0"/>
  </Sequence>
</BehaviorTree>
```

### 7.2 到达状态桥接

`bt_executor_node` 新增订阅 `/ctrl_position/status`，回写黑板：

```cpp
// bt_executor_node.cpp 新增
ctrl_position_status_sub_ = this->create_subscription<std_msgs::msg::String>(
  "/ctrl_position/status", 10,
  [this](const std_msgs::msg::String::SharedPtr msg) {
    if (tree_ && msg->data == "ARRIVED") {
      SET_BB_ALL(tree_, "speed_state", 3);  // 通知 CheckCtrlStatus
    } else if (tree_ && msg->data == "MOVING") {
      SET_BB_ALL(tree_, "speed_state", 1);
    } else if (tree_ && msg->data == "IDLE") {
      SET_BB_ALL(tree_, "speed_state", 0);
    }
  }
);
```

### 7.3 黑板变量交互

| 黑板变量 | 写入者 | 读取者 | 说明 |
|----------|--------|--------|------|
| `coupling_count` | `ExtractTaskInfo` | `SpeedControlCommand` → `/speed_command` | 连挂数 1~50 |
| `speed_state` | `bt_executor_node` (桥接自 `/ctrl_position/status`) | `CheckCtrlStatus` | 0=IDLE, 1=MOVING, 3=ARRIVED |
| `speed_cmd_mode` | `SpeedControlCommand` | `bridgeSpeedCommand()` | 控制模式 20 |

### 7.4 完整数据流（端到端）

```
BT: ExtractTaskInfo
      │ coupling_count → 黑板
      ▼
BT: SpeedControlCommand(ctrl_mode=20) → 黑板 speed_cmd_mode=20 + coupling_count
      │
      ▼
bt_executor_node::bridgeSpeedCommand() → 发布 /speed_command {ctrl_mode:20, coupling_count:N}
      │
      ▼
position_controller_node::speedCmdCallback()
      │ 查找 targets.csv[N] → 计算目标坐标
      │ 启动控制循环
      ▼
position_controller_node::controlLoop() (50Hz)
      │ 读取 /fixposition/llh → 当前坐标
      │ 计算 Haversine 距离
      │ 梯形速度规划
      ▼
/ctrl_position {ctrl_gare:4, ctrl_speed:V, ctrl_break:0}

     ... 行驶中 ...

position_controller_node: 到达判定
      │
      ▼
/ctrl_position/status: "ARRIVED"
      │
      ▼
bt_executor_node 桥接 → 黑板 speed_state=3
      │
      ▼
BT: CheckCtrlStatus(command=3) → SUCCESS
      │
      ▼
BT: SpeedControlCommand(ctrl_mode=0) → 黑板 speed_cmd_mode=0
      │
      ▼
position_controller_node: 收到 ctrl_mode=0 → publishStop()
```

---

## 8. 文件清单

### 8.1 新增文件

```
uncoupling_robot_bt/
├── src/
│   └── position_controller_node.cpp           # 到点控制节点实现
├── config/
│   ├── targets.csv                            # 目标位置映射表 (coupling_count → 目标)
│   └── position_controller.yaml               # ROS2 参数 (控制参数)
├── launch/
│   └── position_controller.launch.py           # 启动文件
└── docs/
    └── position_controller_design.md           # 本文档
```

### 8.2 修改文件

| 文件 | 修改内容 |
|------|----------|
| `CMakeLists.txt` | 添加 `position_controller_node` 编译目标 |
| `package.xml` | 添加 `fixposition_driver_msgs` 依赖 |
| `config/uncoupling_robot_bt_dual_mode.xml` | S3 子树: `ctrl_mode=20` + `CheckCtrlStatus(3)` |
| `src/bt_executor_node.cpp` | 订阅 `/ctrl_position/status` → 回写黑板 `speed_state` |

---

## 9. 编译依赖

```xml
<!-- package.xml 新增 -->
<depend>fixposition_driver_msgs</depend>
<depend>tigou_interfaces</depend>
```

```cmake
# CMakeLists.txt 新增
find_package(fixposition_driver_msgs REQUIRED)
find_package(tigou_interfaces REQUIRED)

add_executable(position_controller_node
  src/position_controller_node.cpp
)
ament_target_dependencies(position_controller_node
  rclcpp std_msgs nav_msgs
  fixposition_driver_msgs
  tigou_interfaces
  uncoupling_robot_bt
)

install(TARGETS position_controller_node
  DESTINATION lib/${PROJECT_NAME})
install(FILES config/targets.csv
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES config/position_controller.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
```

---

## 10. 启动方式

```bash
# 默认启动 (targets.csv 在默认路径)
ros2 launch uncoupling_robot_bt position_controller.launch.py

# 指定自定义 targets 文件
ros2 launch uncoupling_robot_bt position_controller.launch.py \
  targets_file:=/home/user/my_targets.csv

# 直接运行
ros2 run uncoupling_robot_bt position_controller_node \
  --ros-args -p targets_file:="config/targets.csv"
```

### launch 文件模板

```python
# position_controller.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('uncoupling_robot_bt')
    targets_file = LaunchConfiguration('targets_file',
        default=os.path.join(pkg_dir, 'config', 'targets.csv'))

    return LaunchDescription([
        Node(
            package='uncoupling_robot_bt',
            executable='position_controller_node',
            name='position_controller_node',
            parameters=[
                os.path.join(pkg_dir, 'config', 'position_controller.yaml'),
                {'targets_file': targets_file},
            ],
            output='screen',
        ),
    ])
```

---

## 11. 参数汇总

```yaml
# config/position_controller.yaml
position_controller_node:
  ros__parameters:
    # 目标文件路径
    targets_file: ""               # 空=使用 launch 传入的路径

    # 控制参数
    control_rate: 50.0             # Hz, 控制循环频率
    max_forward_vel: 2.0           # m/s, 最大前进速度
    max_acc: 0.5                   # m/s², 最大加速度
    max_dec: 1.0                   # m/s², 最大减速度
    arrival_threshold: 0.15        # m, 到达判定距离阈值
    arrival_duration: 2.0          # s, 稳定持续时间 (防抖动)
```

---

## 12. 后续扩展方向

1. **YAW 角控制**: 增加角速度输出，使底盘在到达目标点的同时对准正确朝向
2. **磁导航精调**: 到点后（距离 < 0.5m）切换到磁导航传感器做最终精确定位
3. **避障集成**: 订阅 lidar 障碍物检测，遇到障碍物自动停车并上报
4. **中途路径点**: 支持多个中间点顺序导航（A→B→C→...），处理弯道场景
5. **ctrl_mode=21 后退模式**: 实现后退到点功能
