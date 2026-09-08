#!/usr/bin/env python3
"""速度跟踪控制节点 —— ROS2 适配节点 (v2.2)。

职责：
    - 订阅 /speed_command (SpeedCommand消息) 获取运动模式、目标车厢号、位置修正值及动态限幅
    - 订阅 /detect_result (DetectResult消息) 获取感知端相对距离/速度/车厢ID
    - 订阅 /chassis_info_fb (ChassisInfoFb消息) 获取自车绝对速度
    - 根据状态机执行 7 种控制模式 (0/1/2/3/4/10/11)
    - 发布 /ctrl_speed (CtrlSpeed消息) 速度+刹车指令 (50Hz)
    - 发布 /speed_state (UInt8消息) 车辆状态反馈
    - PID 动态调参支持
    - ID 匹配校验：仅当 target_id == current_id 时执行跟车 PID

控制律：
    speed_cmd = target_speed + position_PID(position_error)
    target_speed = ego_speed + relative_velocity（目标车厢绝对速度）
    position_error = relative_distance - position_offset（position_offset 来自行为树）

状态机模式：
    0  - 刹车：ctrl_speed=0, ctrl_break=task0_brake_value
    1  - 开始跟车：基于 /detect_result 的位置 PID 跟车
    2  - 微调：跟车输出 + task2_vel_offset
    3  - 恢复跟车：等同模式 1
    4  - 停车：以 task4_dec_valude 平缓减速到 0
    10 - 前进巡航+定点：定速前进，收到 /detect_result → 切模式 1
    11 - 后运动+定点：定速后退，收到 /detect_result → 切模式 1

speed_state 编码：
    1 - 开始跟车
    2 - 正在共速（跟车稳定）
    3 - 完成停车

参数默认值见 config/speed_track_params.yaml。
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
from yhs_can_interfaces.msg import ChassisInfoFb

from data_interfaces.msg import CtrlSpeed, DetectResult, SpeedCommand
from .longitudinal_pid_controller import LongitudinalPIDController


# ═══════════════════════════════════════════════════════════════════════════════
# 参数规则表：一行定义一个参数的 校验 + 归属
#
#   "controller" → 更新到 LongitudinalPIDController
#   "local"      → setattr(self, name, value)
# ═══════════════════════════════════════════════════════════════════════════════

def _ge(v):           return lambda x: x >= v
def _gt(v):           return lambda x: x > v
def _lt(v):           return lambda x: x < v

# 参数规则表：定义参数的校验方法
PARAM_RULES = {
    # ── 位置环 PID ──
    "k_p_pos":              (_ge(0.0),   "k_p_pos must be >= 0",         "controller"),
    "k_i_pos":              (_ge(0.0),   "k_i_pos must be >= 0",         "controller"),
    "k_d_pos":              (_ge(0.0),   "k_d_pos must be >= 0",         "controller"),
    # ── 限幅 ──
    "max_accel":            (_gt(0.0),   "max_accel must be > 0",        "controller"),
    "max_decel":            (_lt(0.0),   "max_decel must be < 0",        "controller"),
    "max_speed":            (_gt(0.0),   "max_speed must be > 0",        "controller"),
    "min_speed":            (_lt(0.0),   "min_speed must be < 0",        "controller"),
    # ── 业务参数 ──
    "task2_vel_offset":   (None,      None,                           "local"),
    "task4_dec_valude":   (None,      None,                           "local"),
    "task10_forward_vel":  (None,      None,                           "local"),
    "task11_backward_vel": (None,      None,                           "local"),
    "task0_brake_value":           (None,      None,                           "local"),
    # ── 跟车稳定性判定 ──
    "stable_pos_threshold": (None,       None,                           "local"),
    "stable_vel_threshold": (None,       None,                           "local"),
    "stable_duration":      (_gt(0.0),   "stable_duration must be > 0",  "local"),
    # ── 巡航定点快速收敛 ──
    "cruise_pos_threshold": (None,       None,                           "local"),
    "cruise_vel_threshold": (None,       None,                           "local"),
    "use_speed_feedforward": (None,      None,                           "local"),
    "feedforward_window":   (_gt(0.0),   "feedforward_window must be > 0",  "local"),
    "feedforward_weight":   (None,       None,                           "local"),
    "_integral_pos_error_limit": (_gt(0.0), "integral_pos_limit must be > 0", "controller"),
}


class SpeedTrackNode(Node):
    """速度跟踪控制节点 (v2.2)。"""

    def __init__(self):
        super().__init__("speed_track_node")

        # ── 1. 声明 ROS 参数 ──
        self._declare_all_parameters()

        # ── 2. 创建纯算法控制器 ──
        self.controller = LongitudinalPIDController(config=self._read_controller_params())

        # ── 3. 注册动态参数回调 ──
        self.add_on_set_parameters_callback(self._on_param_changed)

        # ══════════════════════════════════════════════════════════
        # 4. 运行时状态变量
        # ══════════════════════════════════════════════════════════

        # ── 控制模式 ──
        self.current_mode = 0  # 默认模式 0: 刹车

        # ── /detect_result 数据缓存 ──
        self.relative_distance = 0.0
        self.relative_velocity = 0.0
        self.current_carriage_id = 0
        self.target_updated = False  # 模式 10/11 中标记是否收到新检测数据

        # ── /speed_command 动态限幅及目标缓存 ──
        self.cmd_max_forward_vel = 0.0
        self.cmd_max_backward_vel = 0.0
        self.cmd_max_acc = 0.0
        self.cmd_max_dec = 0.0
        self.target_id = 0           # 行为树指定的目标车厢号
        self.position_offset = 0.0   # 行为树下发的对齐修正值(m)

        # ── 自车速度缓存 ──
        self.ego_speed = 0.0

        # ── 当前速度指令缓存 ──
        self.current_speed_cmd = 0.0

        # ── 跟车稳定性判断 ──
        self.stable_start_time = None
        self._read_local_params()

        # ── 前馈速度滑动窗口：[(timestamp, raw_target_speed), ...] ──
        self._ff_window: list[tuple[float, float]] = []

        # ── 时间管理 ──
        self.last_time = time.time()

        # ── 模式切换时重置控制器 ──
        self._need_controller_reset = False

        # ── 巡航定点逼近标记（模式10/11检测到目标后置True，逼近完成或切模式时清除）──
        self._cruise_approaching = False

        # ══════════════════════════════════════════════════════════
        # 5. 订阅者
        # ══════════════════════════════════════════════════════════
        self.speed_command_sub = self.create_subscription(
            SpeedCommand, "/speed_command", self._speed_command_callback, 10)
        self.target_result_sub = self.create_subscription(
            DetectResult, "/detect_result", self._target_result_callback, 10)
        self.ego_speed_sub = self.create_subscription(
            ChassisInfoFb, "/chassis_info_fb", self._chassis_info_callback, 10)

        # ══════════════════════════════════════════════════════════
        # 6. 发布者
        # ══════════════════════════════════════════════════════════
        self.ctrl_speed_pub = self.create_publisher(CtrlSpeed, "/ctrl_speed", 10)
        self.speed_state_pub = self.create_publisher(UInt8, "/speed_state", 10)

        # ══════════════════════════════════════════════════════════
        # 7. 主控制循环 (50Hz)
        # ══════════════════════════════════════════════════════════
        self.timer = self.create_timer(0.02, self._main_control_loop)

        # ── 8. 日志 ──
        self._log_counter = 0
        self.get_logger().info(
            "SpeedTrackNode v2.2 initialized: mode=0 (刹车), "
            "subscribing /speed_command(SpeedCommand), /detect_result, "
            "/chassis_info_fb, publishing /ctrl_speed(CtrlSpeed) at 50Hz, "
            f"feedforward={'ON' if self.use_speed_feedforward else 'OFF'} "
            f"(window={self.feedforward_window}s, weight={self.feedforward_weight})")
        self._log_initial_params()

    # ═══════════════════════════════════════════════════════════════════════
    # 参数管理
    # ═══════════════════════════════════════════════════════════════════════

    def _decl(self, name, *, default, min, max, step, desc):
        """工厂函数：一行声明一个带范围约束的浮点参数。"""
        self.declare_parameter(
            name, default,
            ParameterDescriptor(
                floating_point_range=[FloatingPointRange(
                    from_value=min, to_value=max, step=step)],
                description=desc,
            ),
        )

    def _declare_all_parameters(self):
        """声明所有运行时参数（默认值由 YAML / launch 文件覆盖）。"""
        # ── 位置环 PID ──
        self._decl("k_p_pos", default=0.3,  min=0.0, max=5.0, step=0.01, desc="位置环 P 系数")
        self._decl("k_i_pos", default=0.04, min=0.0, max=2.0, step=0.001, desc="位置环 I 系数")
        self._decl("k_d_pos", default=0.05, min=0.0, max=2.0, step=0.01, desc="位置环 D 系数")

        # ── 限幅 ──
        self._decl("max_accel", default=1.5,  min=0.1,  max=10.0, step=0.1, desc="最大加速度 (m/s²)")
        self._decl("max_decel", default=-2.0, min=-10.0, max=-0.1, step=0.1, desc="最大减速度 (m/s²)，负值")
        self._decl("max_speed", default=5.0,  min=0.1,  max=20.0, step=0.1, desc="最大输出速度 (m/s)")
        self._decl("min_speed", default=-1.0, min=-20.0, max=-0.1, step=0.1, desc="最小输出速度 (m/s)，负值")

        # ── 业务参数 ──
        self._decl("task2_vel_offset",   default=-0.0, min=-2.0, max=0.0,  step=0.05, desc="微减速偏移 (m/s)")
        self._decl("task4_dec_valude",   default=-0.3, min=-2.0, max=0.0,  step=0.05, desc="停车减速度 (m/s²)")
        self._decl("task10_forward_vel",  default=0.5,  min=0.1,  max=5.0,  step=0.1,  desc="模式10 稳定前进速度 (m/s)")
        self._decl("task11_backward_vel", default=-0.5, min=-5.0, max=-0.1, step=0.1,  desc="模式11 稳定后退速度 (m/s)")
        self._decl("task0_brake_value",           default=50.0, min=0.0,  max=100.0,step=1.0,  desc="模式0 刹车踏板输出 (0-100)")       

        # ── 跟车稳定性判定 ──
        self._decl("stable_pos_threshold", default=0.1,  min=0.01, max=2.0, step=0.01, desc="稳定位置阈值 (m)")
        self._decl("stable_vel_threshold", default=0.1, min=0.01, max=1.0, step=0.01, desc="稳定速度阈值 (m/s)")
        self._decl("stable_duration",      default=3.0,  min=1.0,  max=10.0, step=0.5,  desc="稳定判定持续时间 (s)")
        # ── 巡航定点快速收敛 ──
        self._decl("cruise_pos_threshold", default=0.15, min=0.01, max=1.0, step=0.01, desc="模式10/11 位置接近阈值 (m)")
        self._decl("cruise_vel_threshold", default=0.5,  min=0.01, max=2.0, step=0.01, desc="模式10/11 速度接近阈值 (m/s)")
        self._decl("_integral_pos_error_limit", default=2.0, min=0.0,  max=100.0, step=1.0, desc="位置积分饱和上限 ")
        self.declare_parameter("use_speed_feedforward", True,
            ParameterDescriptor(description="是否启用速度前馈（target_speed = ego + rel_vel）。"
                                            "当感知速度有延迟/不准确时建议关闭。"))
        self._decl("feedforward_window", default=3.0,  min=0.5,  max=10.0, step=0.5, desc="前馈速度滑动平均窗口 (s)")
        self._decl("feedforward_weight", default=0.8,  min=0.0,  max=1.0,  step=0.01, desc="前馈速度权重 (0=纯PID, 1=全前馈)")

    def _read_local_params(self):
        """读取归属于 self 的业务参数到本地缓存。"""
        for name in PARAM_RULES:
            if PARAM_RULES[name][2] == "local":
                setattr(self, name, self.get_parameter(name).value)
                # task0_brake_value 是浮点声明，使用时取整
        self._task0_brake_value_int = int(self.task0_brake_value)         

    def _read_controller_params(self):
        """收集归属于 controller 的参数，返回配置字典。"""
        return {name: self.get_parameter(name).value
                for name, (_, _, owner) in PARAM_RULES.items()
                if owner == "controller"}

    def _on_param_changed(self, params):
        """动态参数回调：通用规则表校验 → 更新对应目标。"""
        controller_updates = {}
        for param in params:
            rule = PARAM_RULES.get(param.name)
            if rule is None:
                self.get_logger().warn(f"Unknown param: {param.name}")
                continue

            validator, err_msg, owner = rule
            if validator is not None and not validator(param.value):
                return SetParametersResult(successful=False, reason=err_msg)

            if owner == "controller":
                controller_updates[param.name] = param.value
            else:
                setattr(self, param.name, param.value)
                if param.name == "task0_brake_value":
                    self._task0_brake_value_int = int(param.value)

            self.get_logger().info(f"{param.name} → {param.value}")

        if controller_updates:
            self.controller.update_params(**controller_updates)

        return SetParametersResult(successful=True)

    # ═══════════════════════════════════════════════════════════════════════
    # 动态限幅 — 由 SpeedCommand 注入
    # ═══════════════════════════════════════════════════════════════════════

    def _apply_speed_command_limits(self):
        """将 SpeedCommand 中的速度/加速度限制推入 PID 控制器。

        SpeedCommand 中的值为 0 时视为未指定，不覆盖。
        """
        updates = {}
        if self.cmd_max_acc > 0:
            updates["max_accel"] = self.cmd_max_acc
        if self.cmd_max_dec < 0:
            updates["max_decel"] = self.cmd_max_dec
        if self.cmd_max_forward_vel > 0:
            updates["max_speed"] = self.cmd_max_forward_vel
        if self.cmd_max_backward_vel < 0:
            updates["min_speed"] = self.cmd_max_backward_vel
        if updates:
            self.controller.update_params(**updates)
            self.get_logger().debug(f"SpeedCommand limits applied: {updates}")

    # ═══════════════════════════════════════════════════════════════════════
    # 订阅回调
    # ═══════════════════════════════════════════════════════════════════════

    def _speed_command_callback(self, msg: SpeedCommand):
        """接收行为树下发的速度控制指令。

        提取 ctrl_mode 切换状态机模式，提取 max_* 字段作为动态限幅，
        提取 target_id 和 position 用于 ID 匹配和位置修正。
        """
        new_mode = msg.ctrl_mode

        # 缓存动态限幅（后续在状态机中应用）
        self.cmd_max_forward_vel = msg.max_forward_vel
        self.cmd_max_backward_vel = msg.max_backward_vel
        self.cmd_max_acc = msg.max_acc
        self.cmd_max_dec = msg.max_dec

        # 缓存目标车厢号和对齐修正值
        self.target_id = msg.target_id
        self.position_offset = msg.position

        if new_mode == self.current_mode:
            return

        if new_mode in (0, 1, 2, 3, 4, 10, 11):
            prev_mode = self.current_mode
            self.get_logger().info(
                f"Mode switch: {prev_mode} → {new_mode}")
            self.current_mode = new_mode
            # 模式 1/2/3 之间切换时保留积分项和控制器状态
            # （仍在跟踪同一车厢，仅控制策略微调，积分清零会导致速度突变）
            if prev_mode in (1, 2, 3) and new_mode in (1, 2, 3):
                self.get_logger().debug(
                    f"Mode {prev_mode}→{new_mode}: 保留积分项")
            else:
                self._need_controller_reset = True
                self.stable_start_time = None
                self._cruise_approaching = False
            # 动态限幅仅对跟车模式生效，巡航模式使用固定速度值(task10/11)
            if new_mode in (1, 2, 3):
                self._apply_speed_command_limits()
        else:
            self.get_logger().warn(
                f"Received invalid ctrl_mode: {new_mode} (valid: 0,1,2,3,4,10,11), ignored")

    def _target_result_callback(self, msg: DetectResult):
        """接收目标检测结果（/detect_result）。

        提取 current_distance 和 relative_velocity 用于跟车 PID。
        模式 10/11 中收到新数据时自动切换到模式 1（开始跟车）。
        """
        self.current_carriage_id = msg.current_id
        self.relative_distance = msg.current_distance
        self.relative_velocity = msg.relative_velocity
        self.target_updated = True

    def _chassis_info_callback(self, msg: ChassisInfoFb):
        """接收底盘反馈的自车绝对速度 (ChassisInfoFb, m/s)。

        ctrl_fb_velocity 是速度标量（恒正），根据 ctrl_fb_gear 决定正负：
        gear=2(倒挡) → 速度为负值，其余 → 速度为正值。
        """
        speed = msg.ctrl_fb.ctrl_fb_velocity
        gear = msg.ctrl_fb.ctrl_fb_gear
        self.ego_speed = -speed if gear == 2 else speed

    # ═══════════════════════════════════════════════════════════════════════
    # 时间工具
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_dt(self):
        """计算控制周期的时间步长，带异常保护。"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt > 0.1 or dt <= 0:
            dt = 0.02
        return dt

    # ═══════════════════════════════════════════════════════════════════════
    # 状态机
    # ═══════════════════════════════════════════════════════════════════════

    def _state_machine(self, mode, dt):
        """状态机：根据控制模式计算速度指令和车辆状态。

        返回:
            (ctrl_speed, ctrl_brake, speed_state)
        """
        # ── 模式 0: 刹车 ──
        if mode == 0:
            self.current_speed_cmd = 0.0
            return 0.0, self._task0_brake_value_int, 3

        # ── 模式 1: 开始跟车 ──
        elif mode == 1:
            speed_cmd = self._compute_mode1_follow(dt)
            self.current_speed_cmd = speed_cmd
            return speed_cmd, 0, self._compute_speed_state()

        # ── 模式 2: 微调 ──
        elif mode == 2:
            base_speed = self._compute_mode1_follow(dt)
            speed_cmd = base_speed + self.task2_vel_offset
            speed_cmd = max(-1.0, min(self.controller.max_speed, speed_cmd))
            self.current_speed_cmd = speed_cmd
            return speed_cmd, 0, self._compute_speed_state()

        # ── 模式 3: 恢复跟车 ──
        elif mode == 3:
            speed_cmd = self._compute_mode1_follow(dt)
            self.current_speed_cmd = speed_cmd
            return speed_cmd, 0, self._compute_speed_state()

        # ── 模式 4: 停车 ──
        elif mode == 4:
            if abs(self.current_speed_cmd) < self.cruise_vel_threshold:
                self.get_logger().info(
                    f"Mode 4: speed={self.current_speed_cmd:.3f}<{self.cruise_vel_threshold}, 直接刹车")
                self.current_mode = 0
                self.current_speed_cmd = 0.0
                return 0.0, self._task0_brake_value_int, 3
            speed_cmd = self._compute_mode4_stop(dt)
            self.current_speed_cmd = speed_cmd
            return speed_cmd, 0, 1

        # ── 模式 10: 前进巡航+定点 ──
        elif mode == 10:
            if self._cruise_approaching:
                pos_err = abs(self.relative_distance - self.position_offset)
                vel_err = abs(self.relative_velocity)
                if pos_err < self.cruise_pos_threshold and vel_err < self.cruise_vel_threshold:
                    self.get_logger().info(
                        f"Mode 10: 逼近完成 pos_err={pos_err:.3f} vel_err={vel_err:.3f}, 刹车")
                    self._cruise_approaching = False
                    self.current_speed_cmd = 0.0
                    return 0.0, self._task0_brake_value_int, 2
                speed_cmd = self._compute_mode1_follow(dt)
                self.current_speed_cmd = speed_cmd
                return speed_cmd, 0, 1

            # ── 未在逼近：先检查是否已在目标位置 ──
            if self.target_id == self.current_carriage_id:
                pos_err = abs(self.relative_distance - self.position_offset)
                vel_err = abs(self.relative_velocity)
                if pos_err < self.cruise_pos_threshold and vel_err < self.cruise_vel_threshold:
                    self.current_speed_cmd = 0.0
                    return 0.0, self._task0_brake_value_int, 2

            # ── 超出阈值且有新检测数据 → 开始逼近 ──
            if self.target_updated and self.target_id == self.current_carriage_id:
                self.target_updated = False
                pos_err = abs(self.relative_distance - self.position_offset)
                self.get_logger().info(
                    f"Mode 10: target acquired (id={self.current_carriage_id}), "
                    f"开始逼近 (pos_err={pos_err:.3f})")
                self._cruise_approaching = True
                self._need_controller_reset = True
                self.stable_start_time = None
                speed_cmd = self._compute_mode1_follow(dt)
                self.current_speed_cmd = speed_cmd
                return speed_cmd, 0, 1

            # ── 无目标：定速巡航 ──
            self.current_speed_cmd = self.task10_forward_vel
            return self.task10_forward_vel, 0, 1

        # ── 模式 11: 后运动+定点 ──
        elif mode == 11:
            if self._cruise_approaching:
                pos_err = abs(self.relative_distance - self.position_offset)
                vel_err = abs(self.relative_velocity)
                if pos_err < self.cruise_pos_threshold and vel_err < self.cruise_vel_threshold:
                    self.get_logger().info(
                        f"Mode 11: 逼近完成 pos_err={pos_err:.3f} vel_err={vel_err:.3f}, 刹车")
                    self._cruise_approaching = False
                    self.current_speed_cmd = 0.0
                    return 0.0, self._task0_brake_value_int, 2
                speed_cmd = self._compute_mode1_follow(dt, allow_reverse=True)
                self.current_speed_cmd = speed_cmd
                return speed_cmd, 0, 1

            # ── 未在逼近：先检查是否已在目标位置 ──
            if self.target_id == self.current_carriage_id:
                pos_err = abs(self.relative_distance - self.position_offset)
                vel_err = abs(self.relative_velocity)
                if pos_err < self.cruise_pos_threshold and vel_err < self.cruise_vel_threshold:
                    self.current_speed_cmd = 0.0
                    return 0.0, self._task0_brake_value_int, 2

            # ── 超出阈值且有新检测数据 → 开始逼近 ──
            if self.target_updated and self.target_id == self.current_carriage_id:
                self.target_updated = False
                pos_err = abs(self.relative_distance - self.position_offset)
                self.get_logger().info(
                    f"Mode 11: target acquired (id={self.current_carriage_id}), "
                    f"开始逼近 (pos_err={pos_err:.3f})")
                self._cruise_approaching = True
                self._need_controller_reset = True
                self.stable_start_time = None
                speed_cmd = self._compute_mode1_follow(dt, allow_reverse=True)
                self.current_speed_cmd = speed_cmd
                return speed_cmd, 0, 1

            # ── 无目标：定速巡航 ──
            self.current_speed_cmd = self.task11_backward_vel
            return self.task11_backward_vel, 0, 1

        # ── 未知模式：退化为刹车 ──
        else:
            self.get_logger().warn(f"Unknown mode {mode}, falling back to brake")
            self.current_speed_cmd = 0.0
            return 0.0, self._task0_brake_value_int, 3

    # ═══════════════════════════════════════════════════════════════════════
    # 模式 1: 跟车 PID 计算
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_mode1_follow(self, dt, allow_reverse: bool = False):
        """计算跟车模式的速度指令。

        控制律: speed_cmd = target_speed + position_PID(position_error)
        target_speed = ego_speed + relative_velocity（目标车厢绝对速度）
        position_error = relative_distance - position_offset

        仅当行为树 target_id 与感知端 current_carriage_id 匹配时执行 PID，
        否则维持当前速度指令不变。

        allow_reverse: True 时允许负速度（模式11后退逼近使用），
                       False 时禁止倒车（模式1/2/3 动态跟车使用）。
        """
        if self._need_controller_reset:
            self.controller.reset()
            self._ff_window.clear()
            self._need_controller_reset = False

        # ID 匹配校验：不是我们要跟的车厢则不执行 PID
        if self.target_id != self.current_carriage_id:
            return self.current_speed_cmd

        # ── 前馈速度：滑动窗口平均 + 权重缩放 ──
        raw_target = self.ego_speed + self.relative_velocity
        now = time.time()

        if self.use_speed_feedforward:
            self._ff_window.append((now, raw_target))
            # 清理超出窗口的旧数据
            cutoff = now - self.feedforward_window
            while self._ff_window and self._ff_window[0][0] < cutoff:
                self._ff_window.pop(0)
            # 计算窗口内均值
            if self._ff_window:
                avg_target = sum(v for _, v in self._ff_window) / len(self._ff_window)
            else:
                avg_target = raw_target
            target_speed = avg_target * self.feedforward_weight
        else:
            target_speed = 0.0
            self._ff_window.clear()

        position_error = self.relative_distance - self.position_offset

        speed_cmd = self.controller.update(
            target_speed=target_speed,
            ego_speed=self.ego_speed,
            position_error=position_error,
            dt=dt,
            clamp_integral_positive=(self.current_mode in (1, 2, 3)),
        )
        # 跟车模式默认禁止倒车，allow_reverse=True 时放开
        if allow_reverse:
            return max(self.controller.min_speed, min(self.controller.max_speed, speed_cmd))
        return max(0.0, speed_cmd)

    # ═══════════════════════════════════════════════════════════════════════
    # 模式 4: 减速停车
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_mode4_stop(self, dt):
        """以 task4_dec_valude 为减速度，从当前速度平缓降到 0。"""
        decel = abs(self.task4_dec_valude)
        if self.current_speed_cmd > 0:
            return max(0.0, self.current_speed_cmd - decel * dt)
        elif self.current_speed_cmd < 0:
            return min(0.0, self.current_speed_cmd + decel * dt)
        return 0.0

    # ═══════════════════════════════════════════════════════════════════════
    # 跟车稳定性判定
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_speed_state(self):
        """根据当前速度指令和跟车误差判定车辆状态。

        返回:
            1 - 开始跟车（正在调整中）
            2 - 正在共速（巡航阈值满足，持续 stable_duration 秒）
            3 - 完成停车 (|ego_speed| < 0.01)
        """
        position_error = self.relative_distance - self.position_offset
        # 模式 1/2/3 用 cruise 阈值，模式 10/11 用 stable 阈值
        if self.current_mode in (1, 2, 3):
            pos_threshold = self.stable_pos_threshold
            vel_threshold = self.stable_vel_threshold
        elif self.current_mode in (10, 11):
            pos_threshold = self.cruise_pos_threshold
            vel_threshold = self.cruise_vel_threshold
        else:
            # 模式 0/4 等不应调用此函数，兜底返回共速中的阈值
            pos_threshold = self.stable_pos_threshold
            vel_threshold = self.stable_vel_threshold
        pos_ok = abs(position_error) < pos_threshold
        vel_ok = abs(self.relative_velocity) < vel_threshold

        now = time.time()
        if pos_ok and vel_ok:
            if self.stable_start_time is None:
                self.stable_start_time = now
            elif now - self.stable_start_time >= self.stable_duration:
                return 2  # 正在共速
            return 1  # 正在计时中，尚未满足 stable_duration
        else:
            self.stable_start_time = None

        # 不满足共速条件时，再进行停车判定
        if abs(self.ego_speed) < 0.01:
            self.stable_start_time = None
            return 3

        return 1  # 开始跟车（调整中）

    # ═══════════════════════════════════════════════════════════════════════
    # 主控制循环 (50Hz)
    # ═══════════════════════════════════════════════════════════════════════

    def _main_control_loop(self):
        """主控制循环（50Hz）。"""
        dt = self._compute_dt()

        ctrl_speed, ctrl_brake, speed_state = self._state_machine(
            self.current_mode, dt)

        # ── 发布 /ctrl_speed (CtrlSpeed) ──
        cmd_msg = CtrlSpeed()
        cmd_msg.ctrl_speed = float(ctrl_speed)
        cmd_msg.ctrl_break = ctrl_brake
        # 档位固定在 0（前进），ctrl_speed 的正负号直接决定方向，
        # x1_speed_controller 对 gare=0 不做取反，原样传递给 DiffDrive
        cmd_msg.ctrl_gare = 0 if ctrl_brake == 0 else 1
        self.ctrl_speed_pub.publish(cmd_msg)

        # ── 发布 /speed_state (UInt8) ──
        state_msg = UInt8(data=speed_state)
        self.speed_state_pub.publish(state_msg)

        # ── 1Hz 状态日志 ──
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            mode_names = {
                0: "刹车", 1: "开始跟车", 2: "微调", 3: "恢复跟车",
                4: "停车", 10: "前进巡航", 11: "后运动",
            }
            state_names = {1: "跟车中", 2: "正在共速", 3: "完成停车"}
            dbg = self.controller.get_debug()
            pos_err = self.relative_distance - self.position_offset
            self.get_logger().info(
                f"[{mode_names.get(self.current_mode, '?')}] "
                f"pos_err={pos_err:+.3f} m | "
                f"pos_pid={dbg.get('pos_pid', 0):+.3f}"
                f" (p={dbg.get('p_pos', 0):+.3f} i={dbg.get('i_pos', 0):+.3f} d={dbg.get('d_pos', 0):+.3f}) | "
                f"tgt_spd={dbg.get('target_speed', 0):+.3f} m/s | "
                f"ff={'ON' if self.use_speed_feedforward else 'OFF'} "
                f"w={self.feedforward_weight:.2f} | "
                f"vel_err={self.relative_velocity:+.3f} m/s | "
                f"vel_spd={ctrl_speed:+.3f} m/s | "
                f"state={state_names.get(speed_state, '?')}({speed_state})"
            )

    def _log_initial_params(self):
        """启动时打印初始参数。"""
        c = self.controller
        self.get_logger().info(
            f"SpeedTrackNode v2.2 parameters:\n"
            f"  Pos PID: k_p_pos={c.k_p_pos}, k_i_pos={c.k_i_pos}, "
            f"k_d_pos={c.k_d_pos}\n"
            f"  Limits:  accel={c.max_accel}, decel={c.max_decel}, "
            f"max_speed={c.max_speed}, min_speed={c.min_speed}\n"
            f"  Biz:     link_dec={self.task2_vel_offset}, "
            f"stop_dec={self.task4_dec_valude}, "
            f"fwd={self.task10_forward_vel}, bwd={self.task11_backward_vel}, "
            f"brake={self._task0_brake_value_int}\n"
            f"  Stable:  pos_thr={self.stable_pos_threshold}, "
            f"vel_thr={self.stable_vel_threshold}, dur={self.stable_duration}s\n"
            f"  Dynamic reconfigure: ENABLED"
        )


# ═══════════════════════════════════════════════════════════════════════════
# ROS2 入口
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = SpeedTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
