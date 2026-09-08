#!/usr/bin/env python3
"""测试上游桥接节点 —— 连接 Gazebo 仿真 ↔ speed_track_node。

职责：
    - 从仿真 odometry 提取自车速度 → /chassis_info_fb (ChassisInfoFb)
    - 从仿真 odometry 计算相对位置/速度 → /detect_result (DetectResult)
    - 通过 ros2 param set 切换模式 → /speed_command (SpeedCommand)

用法：
    # 1. 启动仿真 (参考仿真包 README)
    cd <sim_ws> && source env.sh && ros2 launch train_sim train_sim.launch.py

    # 2. 启动本业务节点
    ros2 run train_control_algorithm speed_track_node

    # 3. 启动本测试桥接节点
    ros2 run train_control_algorithm test_upstream_node

    # 4. 切换模式测试
    ros2 param set /test_upstream_node target_mode 0   # 刹车
    ros2 param set /test_upstream_node target_mode 1   # 开始跟车
    ros2 param set /test_upstream_node target_mode 4   # 停车
    ros2 param set /test_upstream_node target_mode 10  # 前进巡航

    # 5. 查看 speed_track_node 输出
    ros2 topic echo /ctrl_speed
    ros2 topic echo /speed_state
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from data_interfaces.msg import DetectResult, SpeedCommand
from yhs_can_interfaces.msg import ChassisInfoFb


class TestUpstreamNode(Node):
    """测试桥接节点：仿真 ←→ speed_track_node 的中间层。"""

    def __init__(self):
        super().__init__("test_upstream_node")

        # ── 参数 ──
        self.declare_parameter("target_carriage_id", 1,
            ParameterDescriptor(description="要跟踪的目标车厢 ID (1-5)"))
        self.declare_parameter("target_mode", 0,
            ParameterDescriptor(description="控制模式: 0=刹车 1=跟车 2=微调 3=恢复 4=停车 10=前进巡航 11=后运动"))
        self.declare_parameter("ego_odom_topic", "/gz/x1/odometry",
            ParameterDescriptor(description="自车里程计话题"))
        self.declare_parameter("x1_actual_y", -380.0,
            ParameterDescriptor(description="X1 在 Gazebo 中的实际 Y 坐标 (m)，用于补偿 odom 从零计数的偏差"))

        # 读取初始值
        self._target_carriage_id = self.get_parameter("target_carriage_id").value
        self._ego_odom_topic = self.get_parameter("ego_odom_topic").value
        self._x1_actual_y = self.get_parameter("x1_actual_y").value

        # ── 自车状态 ──
        self.ego_speed = 0.0
        self.ego_position = 0.0

        # ── 目标车厢状态 ──
        self.target_speed = 0.0
        self.target_position = 0.0

        # ── 动态参数回调 ──
        self.add_on_set_parameters_callback(self._on_param_changed)

        # ══════════════════════════════════════════════════════════
        # 订阅：自车里程计
        # ══════════════════════════════════════════════════════════
        self.ego_odom_sub = self.create_subscription(
            Odometry, self._ego_odom_topic, self._ego_odom_callback, 10)

        # ══════════════════════════════════════════════════════════
        # 订阅：目标车厢里程计（动态创建）
        # ══════════════════════════════════════════════════════════
        self.target_odom_sub = None
        self._create_target_odom_sub(self._target_carriage_id)

        # ══════════════════════════════════════════════════════════
        # 发布：给 speed_track_node 的三个话题
        # ══════════════════════════════════════════════════════════
        self.speed_pub = self.create_publisher(ChassisInfoFb, "/chassis_info_fb", 10)
        self.target_result_pub = self.create_publisher(DetectResult, "/detect_result", 10)
        self.speed_command_pub = self.create_publisher(SpeedCommand, "/speed_command", 10)

        # ── 定时发布 /detect_result + /chassis_info_fb (10Hz) ──
        self.timer = self.create_timer(0.1, self._publish_sensor_data)

        # ── 启动时发一次初始模式 ──
        self._publish_speed_command(self.get_parameter("target_mode").value)

        self.get_logger().info(
            f"TestUpstreamNode ready: ego_odom={self._ego_odom_topic}, "
            f"target_carriage=#{self._target_carriage_id}, "
            f"publishing to /chassis_info_fb, /detect_result, /speed_command")

    # ═══════════════════════════════════════════════════════════════════════
    # 参数回调
    # ═══════════════════════════════════════════════════════════════════════

    def _on_param_changed(self, params):
        for param in params:
            match param.name:
                case "target_carriage_id":
                    if not (1 <= param.value <= 5):
                        return SetParametersResult(
                            successful=False, reason="target_carriage_id must be 1-5")
                    self._target_carriage_id = param.value
                    self._create_target_odom_sub(param.value)
                    self.get_logger().info(f"target_carriage_id → {param.value}")

                case "target_mode":
                    if param.value not in (0, 1, 2, 3, 4, 10, 11):
                        return SetParametersResult(
                            successful=False, reason="target_mode must be 0/1/2/3/4/10/11")
                    self._publish_speed_command(param.value)
                    self.get_logger().info(f"target_mode → {param.value}")

                case "ego_odom_topic":
                    self._ego_odom_topic = param.value
                    self.destroy_subscription(self.ego_odom_sub)
                    self.ego_odom_sub = self.create_subscription(
                        Odometry, param.value, self._ego_odom_callback, 10)
                    self.get_logger().info(f"ego_odom_topic → {param.value}")

                case "x1_actual_y":
                    self._x1_actual_y = param.value
                    self.get_logger().info(f"x1_actual_y → {param.value}")

                case _:
                    pass

        return SetParametersResult(successful=True)

    # ═══════════════════════════════════════════════════════════════════════
    # 订阅回调
    # ═══════════════════════════════════════════════════════════════════════

    def _ego_odom_callback(self, msg: Odometry):
        """从自车里程计中提取速度和位置。

        X1 的 DiffDrive odom 是机器人本地坐标系，X 轴=前进方向（对应世界 Y）。
        position.x 是累积前进距离，加上 x1_actual_y（初始世界 Y）得到世界 Y。
        """
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.ego_speed = math.sqrt(vx**2 + vy**2)
        self.ego_position = msg.pose.pose.position.x + self._x1_actual_y

    def _target_odom_callback(self, msg: Odometry):
        """从目标车厢里程计中提取速度和位置。

        车厢使用 OdometryPublisher，直接输出世界真实位姿，无需补偿。
        """
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.target_speed = math.sqrt(vx**2 + vy**2)
        self.target_position = msg.pose.pose.position.y

    # ═══════════════════════════════════════════════════════════════════════
    # 话题切换
    # ═══════════════════════════════════════════════════════════════════════

    def _create_target_odom_sub(self, carriage_id):
        """根据车厢 ID 创建对应的里程计订阅。"""
        if self.target_odom_sub is not None:
            self.destroy_subscription(self.target_odom_sub)
        topic = f"/gz/carriage_{carriage_id}/odometry"
        self.target_odom_sub = self.create_subscription(
            Odometry, topic, self._target_odom_callback, 10)
        self.get_logger().info(f"Subscribed to target odometry: {topic}")

    # ═══════════════════════════════════════════════════════════════════════
    # 发布
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_sensor_data(self):
        """定时发布 /chassis_info_fb 和 /detect_result (10Hz)。"""
        # ── /chassis_info_fb ──
        chassis_msg = ChassisInfoFb()
        chassis_msg.ctrl_fb.ctrl_fb_velocity = float(self.ego_speed)
        self.speed_pub.publish(chassis_msg)

        # ── /detect_result ──
        relative_distance = self.target_position - self.ego_position
        relative_velocity = self.target_speed - self.ego_speed

        # 每 5 秒打印一次原始位置供排查
        self.get_logger().info(
            f"[DEBUG] target_pos={self.target_position:.3f} | "
            f"ego_pos={self.ego_position:.3f}"
            f"  (odom_x + x1_actual_y={self._x1_actual_y}) | "
            f"rel_dist={relative_distance:.3f}",
            throttle_duration_sec=5.0,
        )

        detect_msg = DetectResult()
        detect_msg.current_id = self._target_carriage_id
        detect_msg.current_distance = float(relative_distance)
        detect_msg.relative_velocity = float(relative_velocity)
        self.target_result_pub.publish(detect_msg)

    def _publish_speed_command(self, mode: int):
        """发布 SpeedCommand 消息以切换 speed_track_node 的控制模式。"""
        msg = SpeedCommand()
        msg.target_gare = 4  # D档
        msg.ctrl_mode = mode
        msg.target_id = self._target_carriage_id
        msg.coupling_count = 0
        msg.position = 0.0
        # 跟车模式下设置合理的限幅
        if mode in (1, 2, 3):
            msg.max_forward_vel = 2.0
            msg.max_backward_vel = 0.5
            msg.max_acc = 1.0
            msg.max_dec = -0.5
        # 模式 10/11 不下发限幅：巡航速度由 speed_track_node 本地参数
        # task10_forward_vel / task11_backward_vel 决定，不污染控制器 max_speed

        self.speed_command_pub.publish(msg)
        mode_names = {0: "刹车", 1: "开始跟车", 2: "微调", 3: "恢复跟车",
                      4: "停车", 10: "前进巡航", 11: "后运动"}
        self.get_logger().info(
            f"SpeedCommand published: mode={mode} ({mode_names.get(mode, '?')})")


def main(args=None):
    rclpy.init(args=args)
    node = TestUpstreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
