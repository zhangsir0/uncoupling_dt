#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

# 自定义消息接口导入
from magnetic_guide_driver.msg import MagneticData
from yhs_can_interfaces.msg import ChassisInfoFb


class MagneticGuidePyController(Node):
    def __init__(self):
        super().__init__('magnetic_guide_py_controller_node')

        # ------------------- 1. 声明与加载参数 -------------------
        # 话题参数
        self.declare_parameter('topics.front_magnetic', '/magnetic_guide_data_1')
        self.declare_parameter('topics.rear_magnetic', '/magnetic_guide_data_2')
        self.declare_parameter('topics.chassis_fb', '/chassis_info_fb')
        self.declare_parameter('topics.ackermann_cmd', '/ackermann_cmd')

        # 传感器参数
        self.declare_parameter('sensor.safe_sum_threshold_front', 150.0)
        self.declare_parameter('sensor.safe_sum_threshold_rear', 150.0)
        self.declare_parameter('sensor.magnetic_diff_max_front', 500.0)
        self.declare_parameter('sensor.magnetic_diff_max_rear', 500.0)
        self.declare_parameter('sensor.magnetic_width', 0.1565)
        self.declare_parameter('sensor.L_sensor', 1.59)
        self.declare_parameter('sensor.deviation_threshold_m', 0.07)
        self.declare_parameter('sensor.front_sensor_sign', 1.0)
        self.declare_parameter('sensor.rear_sensor_sign', -1.0)
        self.declare_parameter('sensor.steer_to_front_sensor', 0.27)   # ← 新增：前传感器到转向轴距离

        # 控制参数
        self.declare_parameter('control.max_steer_angle', 0.4712)
        self.declare_parameter('control.kp_base', 0.3)
        self.declare_parameter('control.kp_speed_gain', 0.0)
        self.declare_parameter('control.ki_base', 0.02)
        self.declare_parameter('control.ki_speed_decay', 0.0)
        self.declare_parameter('control.k_soft', 0.01)                 # ← 新增：软化常数

        # 提取参数
        self.front_topic = self.get_parameter('topics.front_magnetic').value
        self.rear_topic = self.get_parameter('topics.rear_magnetic').value
        self.chassis_topic = self.get_parameter('topics.chassis_fb').value
        self.ackermann_topic = self.get_parameter('topics.ackermann_cmd').value

        self.sum_thresh_front = self.get_parameter('sensor.safe_sum_threshold_front').value
        self.sum_thresh_rear = self.get_parameter('sensor.safe_sum_threshold_rear').value
        self.diff_max_front = self.get_parameter('sensor.magnetic_diff_max_front').value
        self.diff_max_rear = self.get_parameter('sensor.magnetic_diff_max_rear').value
        self.mag_width = self.get_parameter('sensor.magnetic_width').value
        self.L_sensor = self.get_parameter('sensor.L_sensor').value
        self.dev_threshold_m = self.get_parameter('sensor.deviation_threshold_m').value
        self.front_sign = self.get_parameter('sensor.front_sensor_sign').value
        self.rear_sign = self.get_parameter('sensor.rear_sensor_sign').value
        self.steer_to_front = self.get_parameter('sensor.steer_to_front_sensor').value

        self.max_steer = self.get_parameter('control.max_steer_angle').value
        self.kp_base = self.get_parameter('control.kp_base').value
        self.kp_speed_gain = self.get_parameter('control.kp_speed_gain').value
        self.ki_base = self.get_parameter('control.ki_base').value
        self.ki_speed_decay = self.get_parameter('control.ki_speed_decay').value
        self.k_soft = self.get_parameter('control.k_soft').value

        # ------------------- 2. 状态变量 -------------------
        self.v_chassis = 0.0          # 有符号速度（关键！）
        self.e_front = 0.0
        self.e_rear = 0.0
        self.integral_term = 0.0
        self.last_time = self.get_clock().now()

        self.front_valid = False
        self.rear_valid = False

        # ------------------- 3. ROS 2 通信 -------------------
        self.sub_chassis = self.create_subscription(
            ChassisInfoFb, self.chassis_topic, self.chassis_callback, 10)
        self.sub_front_mag = self.create_subscription(
            MagneticData, self.front_topic, self.front_mag_callback, 10)
        self.sub_rear_mag = self.create_subscription(
            MagneticData, self.rear_topic, self.rear_mag_callback, 10)

        self.pub_ackermann = self.create_publisher(AckermannDriveStamped, self.ackermann_topic, 10)
        self.pub_mag_break = self.create_publisher(Bool, '/magnetic_break', 10)

        self.timer_control = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("I-Stanley Magnetic Guide Controller (Python) - 阿克曼倒挡优化版 Started.")

    def chassis_callback(self, msg: ChassisInfoFb):
        self.v_chassis = msg.ctrl_fb.ctrl_fb_velocity   # 必须是有符号速度！

    def extract_error(self, msg: MagneticData, sum_thresh: float, diff_max: float, sign: float):
        max_sum = max(msg.sum)
        if max_sum <= sum_thresh:
            max_diff_val = max(msg.diff, key=abs)
            norm_diff = 1.0 if max_diff_val > 0 else -1.0
            physical_deviation = norm_diff * (self.mag_width / 2.0) * sign
            return 0.0, False
        else:
            max_diff_val = max(msg.diff, key=abs)
            norm_diff = float(max_diff_val) / diff_max
            physical_deviation = norm_diff * (self.mag_width / 2.0) * sign
            return physical_deviation, True

    def front_mag_callback(self, msg: MagneticData):
        self.e_front, self.front_valid = self.extract_error(
            msg, self.sum_thresh_front, self.diff_max_front, self.front_sign)

    def rear_mag_callback(self, msg: MagneticData):
        self.e_rear, self.rear_valid = self.extract_error(
            msg, self.sum_thresh_rear, self.diff_max_rear, self.rear_sign)

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.02
        self.last_time = current_time

        # ------------------- Step 1: 状态估计 -------------------
        v = self.v_chassis                     # 有符号速度（正/倒车自动适配）
        speed = abs(v)
        diff = self.e_front - self.e_rear
        theta_e_raw = diff / self.L_sensor
        theta_e = math.asin(max(min(theta_e_raw, 0.999), -0.999))

        # ------------------- Step 2: 转向轴投影（倒挡优化核心） -------------------
        e_steer = self.e_front - self.steer_to_front * math.sin(theta_e)

        # ------------------- Step 3: I-Stanley 统一控制律（正/倒车通用） -------------------
        k_p = self.kp_base * (1.0 + self.kp_speed_gain * speed)
        k_i = self.ki_base * (1.0 - self.ki_speed_decay * speed)
        k_i = max(k_i, 0.0)

        cross_track_p = math.atan(k_p * e_steer / (v + self.k_soft))   # v 有符号！
        self.integral_term += e_steer * dt * k_i
        self.integral_term = max(min(self.integral_term, 0.25), -0.25)

        delta_target = theta_e + cross_track_p + self.integral_term

        # ------------------- Step 4: 安全监控与限幅 -------------------
        final_steer = max(min(delta_target, self.max_steer), -self.max_steer)

        deviation_max = max(abs(self.e_front), abs(self.e_rear))
        emergency_brake = False

        if not (self.front_valid and self.rear_valid):
            self.integral_term = 0.0
            emergency_brake = True
            self.get_logger().warn(
                f"EMERGENCY BRAKE! Deviation: F {self.e_front:.3f}m | R {self.e_rear:.3f}m | "
                f"Threshold: {self.dev_threshold_m}m | Valid: F {self.front_valid} R {self.rear_valid}",
                throttle_duration_sec=1.0)
        elif deviation_max > 0.01:
            self.get_logger().info(f"Deviation Warning: F {self.e_front:.3f}m | R {self.e_rear:.3f}m",
                                   throttle_duration_sec=3.0)

        # ------------------- Step 5: 下发指令 -------------------
        self.publish_command(final_steer, emergency_brake)

    def publish_command(self, steering_angle, is_brake):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = "base_link"
        ack_msg.drive.speed = 0.0
        ack_msg.drive.steering_angle = steering_angle
        ack_msg.drive.steering_angle_velocity = 0.0
        self.pub_ackermann.publish(ack_msg)

        msg_break = Bool()
        msg_break.data = is_brake
        self.pub_mag_break.publish(msg_break)


def main(args=None):
    rclpy.init(args=args)
    node = MagneticGuidePyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()