"""
铁路驼峰摘钩机器人 行为树执行器 launch 文件

用法:
  # 基本启动 (自动 tick, 50Hz)
  ros2 launch uncoupling_robot_bt bt_executor.launch.py

  # 单步调试模式
  ros2 launch uncoupling_robot_bt bt_executor.launch.py step_mode:=true

  # 自定义参数
  ros2 launch uncoupling_robot_bt bt_executor.launch.py tick_rate:=10.0 groot_port:=1668

  # 禁止 Groot2 (纯离线运行)
  ros2 launch uncoupling_robot_bt bt_executor.launch.py enable_groot:=false

参数说明:
  bt_xml_path   - 行为树 XML 文件路径 (默认: config/uncoupling_robot_bt.xml)
  tick_rate     - tick 频率 Hz (默认: 50)
  step_mode     - 单步调试模式 (默认: false)
  groot_port    - Groot2 ZMQ 端口 (默认: 1667)
  enable_groot  - 是否启动 Groot2 桥接 (默认: true)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 声明参数 ──────────────────────────────────────────────
    bt_xml_path = LaunchConfiguration('bt_xml_path')
    tick_rate = LaunchConfiguration('tick_rate')
    step_mode = LaunchConfiguration('step_mode')
    groot_port = LaunchConfiguration('groot_port')
    enable_groot = LaunchConfiguration('enable_groot')

    declare_bt_xml_path = DeclareLaunchArgument(
        'bt_xml_path',
        default_value='config/uncoupling_robot_bt.xml',
        description='行为树 XML 文件路径')

    declare_tick_rate = DeclareLaunchArgument(
        'tick_rate',
        default_value='50.0',
        description='Tick 频率 (Hz)')

    declare_step_mode = DeclareLaunchArgument(
        'step_mode',
        default_value='false',
        description='单步调试模式')

    declare_groot_port = DeclareLaunchArgument(
        'groot_port',
        default_value='1667',
        description='Groot2 ZMQ 端口')

    declare_enable_groot = DeclareLaunchArgument(
        'enable_groot',
        default_value='true',
        description='启用 Groot2 连接')

    # ── MPC Controller Node ────────────────────────────────────
    mpc_controller = Node(
        package='uncoupling_robot_bt',
        executable='mpc_controller_node',
        name='mpc_controller_node',
        output='screen',
        parameters=[{
            'max_velocity': 1.0,
            'min_velocity': 0.0,
            'max_acceleration': 1.0,
            'position_weight': 10.0,
            'velocity_weight': 1.0,
            'convergence_threshold': 0.05,
            'control_rate': 50.0,
            'target_gap_id': 3,
        }],
    )

    # ── BT Executor Node ──────────────────────────────────────
    bt_executor = Node(
        package='uncoupling_robot_bt',
        executable='bt_executor_node',
        name='bt_executor_node',
        output='screen',
        parameters=[{
            'bt_xml_path': bt_xml_path,
            'tick_rate': tick_rate,
            'step_mode': step_mode,
            'groot_port': groot_port,
            'enable_groot': enable_groot,
        }],
        # 模拟时钟 (rosbag 回放时使用)
        # use_sim_time=True,
    )

    return LaunchDescription([
        declare_bt_xml_path,
        declare_tick_rate,
        declare_step_mode,
        declare_groot_port,
        declare_enable_groot,
        mpc_controller,
        bt_executor,
    ])
