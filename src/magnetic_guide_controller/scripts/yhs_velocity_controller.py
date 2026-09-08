#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from yhs_can_interfaces.msg import AutoSpdCtrlCmd
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
import math

class YhsUnifiedForwarder(Node):
    def __init__(self):
        super().__init__('yhs_unified_forwarder')
        
        self.declare_parameter('ackermann_topic', '/ackermann_cmd')
        self.declare_parameter('teleop_topic', '/teleop_cmd')
        self.declare_parameter('auto_spd_topic', '/auto_spd_ctrl_cmd')
        self.declare_parameter('break_topic', '/magnetic_break')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_brake_topic', '/cmd_brake')
        self.declare_parameter('use_cmd_vel', True)  # 是否使用 cmd_vel 作为速度输入，默认为 False（优先键盘输入）
        self.declare_parameter('use_cmd_vel_flag_topic', '/use_cmd_vel_flag')
        
        ackermann_topic = self.get_parameter('ackermann_topic').value
        teleop_topic = self.get_parameter('teleop_topic').value
        auto_spd_topic = self.get_parameter('auto_spd_topic').value
        break_topic = self.get_parameter('break_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        cmd_brake_topic = self.get_parameter('cmd_brake_topic').value
        cmd_vel_flag_topic = self.get_parameter('use_cmd_vel_flag_topic').value
        self.use_cmd_vel = self.get_parameter('use_cmd_vel').value
        
        # 状态缓存
        self.current_steering = 0.0          # 来自磁导航 (/ackermann_cmd)
        
        self.current_gear = 1                # 来自键盘控制 (/teleop_cmd)
        self.current_velocity = 0.0          # 来自键盘控制
        self.current_brake = 1               # 来自键盘控制
        self.current_cmd_brake = None        # 来自 /cmd_brake，None 表示未接收到
        
        self.is_magnetic_break = False       # 来自磁导航脱轨保护 (/magnetic_break)
        
        # 订阅 1: 磁导航方向 (/ackermann_cmd)
        self.create_subscription(AckermannDriveStamped, ackermann_topic, self.ackermann_callback, 10)
        
        # 订阅 2: 键盘输入的速度与档位等 (/teleop_cmd)
        self.create_subscription(AutoSpdCtrlCmd, teleop_topic, self.teleop_callback, 10)
        
        # 订阅 3: 磁导航急停保护 (/magnetic_break)
        self.create_subscription(Bool, break_topic, self.break_callback, 10)
        
        # 订阅 4: cmd_vel 输入速度信息
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        
        # 新增: 外部制动输入（优先级高于键盘）
        self.create_subscription(UInt8, cmd_brake_topic, self.cmd_brake_callback, 10)
        
        # 订阅 5: 从键盘读取是否使用 cmd_vel 速度的标志，实现动态切换
        self.create_subscription(Bool, cmd_vel_flag_topic, self.cmd_vel_flag_callback, 10)
        
        # 发布: 统一融合给底盘驱动的话题 (/auto_spd_ctrl_cmd)
        self.pub = self.create_publisher(AutoSpdCtrlCmd, auto_spd_topic, 10)
        
        # 50 Hz 发布频率
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.get_logger().info("Unified AutoSpdCtrlCmd Forwarder initialized (Ackermann for Steering, Teleop for Vel/Gear/Brake)")

    def ackermann_callback(self, msg: AckermannDriveStamped):
        # [方向] 只提取 ackermann 的转角信息
        self.current_steering = math.degrees(msg.drive.steering_angle)

    def teleop_callback(self, msg: AutoSpdCtrlCmd):
        # [速度、档位、驻车] 从键盘提取
        self.current_gear = msg.ctrl_cmd_gear
        
        # 根据开关选择是否使用键盘速度
        if not self.use_cmd_vel:
            self.current_velocity = msg.ctrl_cmd_velocity
            
        self.current_brake = msg.ctrl_cmd_brake

    def break_callback(self, msg: Bool):
        self.is_magnetic_break = msg.data

    def cmd_vel_callback(self, msg: Twist):
        # 取 cmd_vel 的 x 向线速度
        if self.use_cmd_vel:
            if self.current_gear == 4:
                self.current_velocity = msg.linear.x
            elif self.current_gear == 2:
                self.current_velocity = -msg.linear.x

    def cmd_brake_callback(self, msg: UInt8):
        # 直接缓存外部制动命令，发布时优先使用
        self.current_cmd_brake = int(msg.data)

    def cmd_vel_flag_callback(self, msg: Bool):
        # 接收到键盘节点要求改变速度源时，根据标志位决定当前使用哪种
        if self.use_cmd_vel != msg.data:
            self.get_logger().info(f"Target velocity source switched. Use cmd_vel: {msg.data}")
            self.use_cmd_vel = msg.data

    def timer_callback(self):
        msg = AutoSpdCtrlCmd()
        
        # === 组合消息 ===
        # 方向：绝对服从磁导航节点
        msg.ctrl_cmd_steering = float(self.current_steering)
        
        # 速度与启停：优先服从磁导航急停，其次服从键盘
        if self.is_magnetic_break:
            msg.ctrl_cmd_gear = 1
            msg.ctrl_cmd_velocity = 0.0
            # 这里如果底盘制动不是 1 可根据需求改 brake
        else:
            msg.ctrl_cmd_gear = self.current_gear
            msg.ctrl_cmd_velocity = float(self.current_velocity)
        
        # 制动优先级：/cmd_brake > /teleop_cmd
        if self.current_cmd_brake is not None and self.current_cmd_brake > 0:
            msg.ctrl_cmd_brake = self.current_cmd_brake
        else:
            msg.ctrl_cmd_brake = self.current_brake
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YhsUnifiedForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
