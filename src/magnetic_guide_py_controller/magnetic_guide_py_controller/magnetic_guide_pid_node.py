#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from magnetic_guide_driver.msg import MagneticData
from yhs_can_interfaces.msg import ChassisInfoFb

class DualModeMagPIDController(Node):
    def __init__(self):
        super().__init__('magnetic_guide_pid_controller_node')

        # ------------------- 1. 声明与加载参数 -------------------
        self.declare_parameter('topics.front_magnetic', '/magnetic_guide_data_1')
        self.declare_parameter('topics.rear_magnetic', '/magnetic_guide_data_2')
        self.declare_parameter('topics.chassis_fb', '/chassis_info_fb')

        # 传感器参数
        self.declare_parameter('sensor.safe_sum_threshold', 750.0)
        self.declare_parameter('sensor.diff_normalize_range', 750.0)

        # 控制参数 (前进)
        self.declare_parameter('control.forward.kp_base', 0.2)
        self.declare_parameter('control.forward.ki_base', 0.0)
        self.declare_parameter('control.forward.kd_base', 0.01)
        self.declare_parameter('control.forward.kp_speed_decay', 0.3)
        self.declare_parameter('control.forward.ki_speed_decay', 0.1)

        # 控制参数 (后退)
        self.declare_parameter('control.reverse.kp_base', 0.26)
        self.declare_parameter('control.reverse.ki_base', 0.0)
        self.declare_parameter('control.reverse.kd_base', 0.01)
        self.declare_parameter('control.reverse.kp_speed_decay', 0.1)
        self.declare_parameter('control.reverse.ki_speed_decay', 0.1)

        # 物理限制
        self.declare_parameter('control.max_steer_angle', 0.2)
        
        # 提取话题参数
        front_topic = self.get_parameter('topics.front_magnetic').value
        rear_topic = self.get_parameter('topics.rear_magnetic').value
        chassis_topic = self.get_parameter('topics.chassis_fb').value

        self.sum_thresh = self.get_parameter('sensor.safe_sum_threshold').value
        self.diff_norm_range = self.get_parameter('sensor.diff_normalize_range').value
        self.max_steer = self.get_parameter('control.max_steer_angle').value

        # ------------------- 2. 状态变量 -------------------
        self.v_chassis = 0.0
        self.current_gear = 3     # 默认N档
        
        self.front_max_sum = 0.0
        self.front_diff = 0.0

        self.rear_max_sum = 0.0
        self.rear_diff = 0.0
        
        # PID 变量
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        # ------------------- 3. ROS 2 通信 -------------------
        self.create_subscription(ChassisInfoFb, chassis_topic, self.chassis_callback, 10)
        self.create_subscription(MagneticData, front_topic, self.front_mag_callback, 10)
        self.create_subscription(MagneticData, rear_topic, self.rear_mag_callback, 10)

        self.pub_mag_steer = self.create_publisher(Float32, '/ctrl_mag_steer', 10)
        self.pub_mag_stop  = self.create_publisher(Bool, '/ctrl_mag_stop', 10)

        # 50Hz 控制循环
        self.create_timer(0.02, self.control_loop)
        self.get_logger().info("DualMode Mag PID Controller Started.")

    def chassis_callback(self, msg: ChassisInfoFb):
        self.current_gear = msg.ctrl_fb.ctrl_fb_gear
        if hasattr(msg.ctrl_fb, 'ctrl_fb_velocity'):
            self.v_chassis = msg.ctrl_fb.ctrl_fb_velocity
        elif hasattr(msg.ctrl_fb, 'ctrl_fb_linear_x'):
            self.v_chassis = msg.ctrl_fb.ctrl_fb_linear_x

    def parse_magnetic_data(self, msg: MagneticData):
        max_sum = max(msg.sum)
        # 获取绝对值最大的 diff
        max_diff_val = max(msg.diff, key=abs)
        return float(max_sum), float(max_diff_val)

    def front_mag_callback(self, msg: MagneticData):
        self.front_max_sum, self.front_diff = self.parse_magnetic_data(msg)

    def rear_mag_callback(self, msg: MagneticData):
        self.rear_max_sum, self.rear_diff = self.parse_magnetic_data(msg)

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: dt = 0.02
        self.last_time = current_time

        # 是否触发脱轨急停
        emergency_brake = False

        msg_stop = Bool()

        # ------------------- 2. 状态机与数据路由 -------------------
        if self.current_gear == 4 or self.current_gear == 3 or self.current_gear == 1: # D 或 N档 按前进处理
            is_forward = True
            active_sum = self.front_max_sum
            active_diff = self.front_diff
            # 极性：前面传感器车辆靠右diff为负，靠左diff为正
            current_error_raw =  active_diff
            
            kp_base = self.get_parameter('control.forward.kp_base').value
            ki_base = self.get_parameter('control.forward.ki_base').value
            kd = self.get_parameter('control.forward.kd_base').value
            kp_decay = self.get_parameter('control.forward.kp_speed_decay').value
            ki_decay = self.get_parameter('control.forward.ki_speed_decay').value
            
        elif self.current_gear == 2: # R档
            is_forward = False
            active_sum = self.rear_max_sum
            active_diff = self.rear_diff
            # 极性：后向传感器相反，倒车时由于物理同轴方向，需注意其符号
            current_error_raw = active_diff
            
            kp_base = self.get_parameter('control.reverse.kp_base').value
            ki_base = self.get_parameter('control.reverse.ki_base').value
            kd = self.get_parameter('control.reverse.kd_base').value
            kp_decay = self.get_parameter('control.reverse.kp_speed_decay').value
            ki_decay = self.get_parameter('control.reverse.ki_speed_decay').value
                 
            
        else:  # P档驻车：转向清零，不触发脱轨停止
            self.publish_steering(0.0)                                    
            msg_stop.data = False                                         
            self.pub_mag_stop.publish(msg_stop)                           
            return

        # ------------------- 1. 安全监控模块 -------------------
        if active_sum <= self.sum_thresh:
            emergency_brake = True
            msg_stop.data = True
            self.pub_mag_stop.publish(msg_stop)
            self.integral = 0.0

            self.get_logger().warn(
                f"EMERGENCY BRAKE! active_sum ({active_sum}) <= thresh ({self.sum_thresh}) | front_diff: {self.front_diff} | rear_diff: {self.rear_diff}.",
                throttle_duration_sec=1.0
            )

            self.publish_stop()
            return
        else:
            msg_stop.data = False
            self.pub_mag_stop.publish(msg_stop)

        # ------------------- 3. 动态增益调度模块 -------------------
        speed = abs(self.v_chassis)
        kp_current = kp_base * (1.0 - kp_decay * speed)
        ki_current = ki_base * (1.0 - ki_decay * speed)
        
        # 限制不为负
        kp_current = max(kp_current, 0.0)
        ki_current = max(ki_current, 0.0)

        # 归一化处理 diff 到 [-1.0, 1.0] 的偏差空间，或按实际调整
        normalized_diff = current_error_raw / self.diff_norm_range
        normalized_diff = max(min(normalized_diff, 1.0), -1.0)
        
        # 车身前向偏角误差 (假设车体偏右，e_raw为负，要向左修正则需要正的转向角)
        error = normalized_diff if is_forward else -normalized_diff

        # ------------------- 4. 核心 PID 运算模块 -------------------
        self.integral += error * dt
        # 积分限幅
        self.integral = max(min(self.integral, 10.0), -10.0)
        
        derivative = (error - self.last_error) / dt
        self.last_error = error

        p_out = kp_current * error
        i_out = ki_current * self.integral
        d_out = kd * derivative

        target_steer = p_out + i_out + d_out
        
        # 倒车阿克曼转向极性：因为后轮的纠正与前轮是相反效应。
        if not is_forward:
            target_steer = -target_steer

        # ------------------- 5. 硬件安全限幅 -------------------
        final_steer = max(min(target_steer, self.max_steer), -self.max_steer)

        self.publish_steering(final_steer)
        
        # ------------------- 6. 状态日志输出 -------------------
        self.get_logger().info(
            f"Mode:{'FWD' if is_forward else 'REV'} | "
            f"Spd:{speed:.2f} | "
            f"FSum:{self.front_max_sum:.0f} RSum:{self.rear_max_sum:.0f} | "
            f"FDiff:{self.front_diff:.1f} RDiff:{self.rear_diff:.1f} |  "  
            f"Err_norm:{error:.3f} | "
            f"Kp:{kp_current:.3f} Ki:{ki_current:.3f} | "
            f"P:{p_out:.3f} I:{i_out:.3f} D:{d_out:.3f} | "
            f"Steer:{final_steer:.3f}",
        )

    def publish_steering(self, steering_angle):
        """发布 /ctrl_mag_steer (Float32, 度)，供底盘仲裁节点 L1 转向覆盖。"""
        steer_deg = Float32()
        steer_deg.data = float(math.degrees(steering_angle))
        self.pub_mag_steer.publish(steer_deg)

    def publish_stop(self):
        """发布停止指令：转向清零 + 触发磁导航脱轨停止。"""
        # 清零 /ctrl_mag_steer
        steer_zero = Float32()
        steer_zero.data = 0.0
        self.pub_mag_steer.publish(steer_zero)

        # 发布 /ctrl_mag_stop = True，通知底盘仲裁节点磁导航已停止
        stop_msg = Bool()
        stop_msg.data = True
        self.pub_mag_stop.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DualModeMagPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
