#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yhs_can_interfaces.msg import AutoSpdCtrlCmd
from std_msgs.msg import Bool
import sys
import select
import termios
import tty

msg = """
Reading from the keyboard and Publishing to combined topic!
---------------------------
1. Set Gear First:
   p : P Gear (Park)
   r : R Gear (Reverse)
   n : N Gear (Neutral)
   d : D Gear (Drive)

2. Move (Arrow Keys):
   Up/Down (or W/X) : Accelerate / Decelerate
   Left/Right       : Steer Left / Steer Right

3. Brake Logic:
   space      : Brake = 100
   s          : Brake = 0

4. Velocity Mode Switch:
   c          : Use cmd_vel for speed
   k          : Use keyboard for speed

q/z : increase/decrease max speeds/steering steps by 10%
CTRL-C to quit
"""

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    
    # Handle Arrow keys which start with \x1b (ESC)
    if key == '\x1b':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        tty.setcbreak(sys.stdin.fileno())
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            key += ch2 + ch3

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = rclpy.create_node('teleop_keyboard_node')
    
    # 发布往这个中间话题，供 yhs_velocity_controller.py 融合并读取速度和档位、刹车
    pub = node.create_publisher(AutoSpdCtrlCmd, '/teleop_cmd', 10)
    # 提供速度来源选项切换的话题
    pub_mode = node.create_publisher(Bool, '/use_cmd_vel_flag', 10)

    speed_step = 0.1    # 每次加减速步进 (m/s)
    turn_step = 2.0     # 每次转向角度步进 (degrees)
    
    abs_speed = 0.0
    steering_angle = 0.0
    current_gear = 1    # 1:P, 2:R, 3:N, 4:D
    gear_str = 'N'
    current_brake = 100   # 刹车状态 (默认0)
    use_cmd_vel = True    # 默认值跟随转发节点，由cmd控制

    status = 0

    try:
        print(msg)
        print(f"currently:\tGear {gear_str}\tBrake {current_brake}\tspeed {abs_speed:.2f}\tturn {steering_angle:.2f}\tMode {'cmd_vel' if use_cmd_vel else 'keyboard'} ")
        while True:
            key = getKey(settings)
            
            # Gear selection
            if key == 'p':
                current_gear = 1
                gear_str = 'P'
            elif key == 'r':
                current_gear = 2
                gear_str = 'R'
            elif key == 'n':
                current_gear = 3
                gear_str = 'N'
            elif key == 'd':
                current_gear = 4
                gear_str = 'D'
            
            # Movement (Arrow keys or W/X equivalents)
            elif key == '\x1b[A' or key == 'w': # UP
                abs_speed += speed_step
            elif key == '\x1b[B' or key == 'x': # DOWN
                abs_speed -= speed_step
                if abs_speed < 0:
                    abs_speed = 0.0
            elif key == '\x1b[D': # LEFT
                steering_angle -= turn_step
            elif key == '\x1b[C': # RIGHT
                steering_angle += turn_step

            # Brake control
            elif key == ' ': # SPACE
                current_brake = 100
                abs_speed = 0.0 # 踩死刹车时，也可顺便将目标速度置零以防意外
            elif key == 's':
                current_brake = 0
                
            # Mode Switch control
            elif key == 'c':
                use_cmd_vel = True
            elif key == 'k':
                use_cmd_vel = False
                
            elif key in speedBindings.keys():
                speed_step = speed_step * speedBindings[key][0]
                turn_step = turn_step * speedBindings[key][1]
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key == '\x03': # CTRL+C
                break

            # 打包发送至转发节点
            cmd_msg = AutoSpdCtrlCmd()
            
            cmd_msg.ctrl_cmd_gear = current_gear
            cmd_msg.ctrl_cmd_velocity = float(abs_speed)
            cmd_msg.ctrl_cmd_brake = int(current_brake)
            
            # 同时发送控制模式标志
            mode_msg = Bool()
            mode_msg.data = use_cmd_vel
            pub_mode.publish(mode_msg)
            
            pub.publish(cmd_msg)
            print(f"\rcurrently:\tGear {gear_str}\tBrake {current_brake}\tspeed {abs_speed:.2f}\tturn {steering_angle:.2f}\tMode {'cmd_vel' if use_cmd_vel else 'keyboard'}    ", end='')

    except Exception as e:
        print(e)
        
    finally:
        cmd_msg = AutoSpdCtrlCmd()
        cmd_msg.ctrl_cmd_gear = 1 # p
        cmd_msg.ctrl_cmd_velocity = 0.0
        cmd_msg.ctrl_cmd_brake = 100 # 当节点退出时给出强刹车
        pub.publish(cmd_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
