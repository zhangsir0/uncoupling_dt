#!/usr/bin/env python3
"""纵向位置 PID 控制器 —— 纯算法实现，零 ROS/中间件依赖。

本模块只负责数学计算：接收固定输入 → 计算 → 输出速度指令。
所有 I/O、参数管理、生命周期等工程关注点由 longitudinal_pid_ros_node.py 负责。

控制律：
    speed_cmd = target_speed + position_PID(position_error)

其中：
    - target_speed: 前馈项——目标绝对速度
    - position_PID: 反馈项——对位置误差做PID组合
    - 控制律输出速度再经过加速度和速度限幅后得到最终速度指令

设计意图：
    - 首先反馈项会使得位置收敛，然后再将速度收敛到前馈速度
"""


class LongitudinalPIDController:
    """纵向位置 PID 控制器（位置环 + 加速度/速度限幅）。

    职责边界：
        - 接收：目标速度、自车速度、位置误差、控制周期 dt
        - 输出：速度指令（float）
        - 不感知 ROS、网络、文件系统

    用法示例：
        >>> ctrl = LongitudinalPIDController({"k_p_pos": 0.3, "k_i_pos": 0.01})
        >>> cmd = ctrl.update(
        ...     target_speed=2.0,
        ...     ego_speed=1.8,
        ...     position_error=0.5,
        ...     dt=0.05
        ... )
        >>> ctrl.reset()  # 重置内部状态
        >>> ctrl.update_params(k_p_pos=0.5)  # 运行时调参
    """

    # ── 默认参数 ───────────────────────────────────────────────────────────
    _DEFAULT_CONFIG = {
        # 位置环 PID
        "k_p_pos": 0.3,
        "k_i_pos": 0.01,
        "k_d_pos": 0.05,
        # 限幅参数
        "max_accel": 1.5,
        "max_decel": -2.0,
        "max_speed": 5.0,
        "min_speed": -1.0,
        # 位置积分饱和上限（内部常量，不对外暴露）
        "_integral_pos_error_limit": 1.0,
    }

    def __init__(self, config=None):
        """初始化控制器。

        参数：
            config: dict，部分或全部控制参数。未提供的键使用默认值。
                    支持的键见 _DEFAULT_CONFIG。
        """
        cfg = {**self._DEFAULT_CONFIG, **(config or {})}

        # ── 位置环 PID 参数 ──
        self.k_p_pos = cfg["k_p_pos"]
        self.k_i_pos = cfg["k_i_pos"]
        self.k_d_pos = cfg["k_d_pos"]

        # ── 限幅参数 ──
        self.max_accel = cfg["max_accel"]
        self.max_decel = cfg["max_decel"]
        self.max_speed = cfg["max_speed"]
        self.min_speed = cfg["min_speed"]

        # ── 内部常量 ──
        self._integral_pos_error_limit = cfg["_integral_pos_error_limit"]

        # ── 控制器状态（每次 reset() 后归零） ──
        self.reset()

        # ── 调试信息（存储最近一次 update() 的中间值，不改逻辑） ──
        self._debug = {}

    # ═══════════════════════════════════════════════════════════════════════
    # 公开接口
    # ═══════════════════════════════════════════════════════════════════════

    def update(self, target_speed, ego_speed, position_error, dt,
               clamp_integral_positive: bool = False):
        """单步控制更新 —— 算法的唯一入口。

        参数：
            target_speed (float):  目标速度 (m/s)
            ego_speed (float):     自车当前速度 (m/s)
            position_error (float):位置误差 (m)，target_position - ego_position
            dt (float):            控制周期 (s)
            clamp_integral_positive (bool): True 时积分下限为 0，取消负积分

        返回：
            float: 速度指令 (m/s)，范围 [min_speed, max_speed]
        """
        # 1. 位置环 PID 计算
        pos_pid, p_pos, i_pos, d_pos = self._compute_position_pid(
            position_error, dt, clamp_integral_positive)

        # 2. 组合控制律: speed_cmd = target_speed + position_PID
        raw_cmd = target_speed + pos_pid

        # 3. 加速度限制
        limited_cmd = self._apply_acceleration_limit(ego_speed, raw_cmd, dt)

        # 4. 最终限幅 [min_speed, max_speed]
        speed_cmd = max(self.min_speed, min(self.max_speed, limited_cmd))

        # ── 存储调试信息（不改控制逻辑，只供外部读取） ──
        self._debug = {
            "target_speed": target_speed,
            "ego_speed": ego_speed,
            "position_error": position_error,
            "p_pos": p_pos,
            "i_pos": i_pos,
            "d_pos": d_pos,
            "pos_pid": pos_pid,
            "raw_cmd": raw_cmd,
            "limited_cmd": limited_cmd,
            "speed_cmd": speed_cmd,
        }

        return speed_cmd

    def reset(self):
        """重置控制器内部状态。

        清除积分累积、微分历史、加速度历史。
        通常在模式切换或重新激活控制器时调用。
        """
        self.integral_pos_error = 0.0
        self.prev_pos_error = 0.0
        self.prev_cmd_speed = 0.0
        self._accel_initialized = False

    def update_params(self, **kwargs):
        """运行时更新控制参数（热更新，无需重启）。

        只更新传入的键，未传入的保持不变。

        用法：
            ctrl.update_params(k_p_pos=0.5, max_speed=4.0)
        """
        allowed = {
            "k_p_pos", "k_i_pos", "k_d_pos",
            "max_accel", "max_decel", "max_speed", "min_speed",
            "_integral_pos_error_limit",
        }
        for key, value in kwargs.items():
            if key in allowed:
                setattr(self, key, value)

    def get_state(self):
        """获取当前内部状态（用于调试/监控）。

        返回：
            dict: 包含 integral_pos_error, prev_pos_error, prev_cmd_speed
        """
        return {
            "integral_pos_error": self.integral_pos_error,
            "prev_pos_error": self.prev_pos_error,
            "prev_cmd_speed": self.prev_cmd_speed,
        }

    # ═══════════════════════════════════════════════════════════════════════
    # 内部计算方法
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_position_pid(self, position_error, dt,
                               clamp_integral_positive: bool = False):
        """位置环 PID 反馈（比例 + 积分 + 微分）。

        积分项带 ±_integral_pos_error_limit 饱和限制，防止积分饱和（windup）。
        微分项在 dt ≤ 0 时退化为 0。

        返回：
            (pos_pid, p_term, i_term, d_term)
        """
        # 比例项
        p_term = self.k_p_pos * position_error

        # 积分项（带饱和限制）
        self.integral_pos_error += position_error * dt
        lower = 0.0 if clamp_integral_positive else -self._integral_pos_error_limit
        self.integral_pos_error = max(
            lower,
            min(self._integral_pos_error_limit, self.integral_pos_error),
        )
        i_term = self.k_i_pos * self.integral_pos_error

        # 微分项
        if dt > 0:
            d_term = self.k_d_pos * (position_error - self.prev_pos_error) / dt
        else:
            d_term = 0.0

        self.prev_pos_error = position_error

        return (p_term + i_term + d_term, p_term, i_term, d_term)


    def _apply_acceleration_limit(self, current_speed, desired_speed, dt):
        """对速度指令施加加速度/减速度限制，限制计算出的速度指令的加速度在 [max_accel, max_decel] 内"""
        # 首次调用时用当前速度初始化历史值，避免冷启动跳变。
        if not self._accel_initialized:
            self.prev_cmd_speed = current_speed
            self._accel_initialized = True

        if dt > 0:
            accel = (desired_speed - self.prev_cmd_speed) / dt

            if accel > self.max_accel:
                limited = self.prev_cmd_speed + self.max_accel * dt
            elif accel < self.max_decel:
                limited = self.prev_cmd_speed + self.max_decel * dt
            else:
                limited = desired_speed
        else:
            limited = desired_speed

        self.prev_cmd_speed = limited
        return limited

    # ═══════════════════════════════════════════════════════════════════════
    # 辅助方法
    # ═══════════════════════════════════════════════════════════════════════

    def get_debug(self):
        """获取最近一次 update() 的中间计算值（用于外部调试/日志）。

        返回：
            dict: 包含位置误差、PID 各项、最终指令等全部中间值。
                  若 update() 尚未被调用则返回空 dict。
        """
        return self._debug
