"""
底盘控制仲裁节点 launch 文件

用法:
  # 默认参数启动
  ros2 launch chassis_arbitration chassis_arbitration.launch.py

  # 自定义参数文件
  ros2 launch chassis_arbitration chassis_arbitration.launch.py params_file:=/path/to/params.yaml

参数说明:
  params_file - ROS2 参数文件路径 (默认: params/chassis_arbitration.yaml)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('chassis_arbitration')

    # ── 声明参数 ──────────────────────────────────────────────
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'chassis_arbitration.yaml'),
        description='ROS2 参数文件路径')

    # ── 底盘控制仲裁节点 ──────────────────────────────────────
    chassis_arbitration = Node(
        package='chassis_arbitration',
        executable='chassis_arbitration_node',
        name='chassis_arbitration_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_declare,
        chassis_arbitration,
    ])
