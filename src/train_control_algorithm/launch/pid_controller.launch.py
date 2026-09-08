#!/usr/bin/env python3
"""启动 speed_track_node + test_upstream_node。

用法:
    ros2 launch train_control_algorithm pid_controller.launch.py
    ros2 launch train_control_algorithm pid_controller.launch.py \
        target_carriage_id:=2 x1_actual_y:=-380.0

运行时动态切换模式:
    ros2 param set /test_upstream_node target_mode 0   # 刹车
    ros2 param set /test_upstream_node target_mode 1   # 跟车
    ros2 param set /test_upstream_node target_mode 4   # 停车
    ros2 param set /test_upstream_node target_mode 10  # 前进巡航
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_PKG = "train_control_algorithm"
_PKG_SHARE = get_package_share_directory(_PKG)
_DEFAULT_PARAMS = os.path.join(_PKG_SHARE, "config", "speed_track_params.yaml")


def generate_launch_description():
    # ── test_upstream_node 参数 ──
    target_carriage_id = LaunchConfiguration("target_carriage_id", default="1")
    target_mode = LaunchConfiguration("target_mode", default="0")
    ego_odom_topic = LaunchConfiguration("ego_odom_topic", default="/gz/x1/odometry")
    x1_actual_y = LaunchConfiguration("x1_actual_y", default="-380.0")

    return LaunchDescription([
        DeclareLaunchArgument("target_carriage_id", default_value="1",
                              description="要跟踪的目标车厢 ID (1-5)"),
        DeclareLaunchArgument("target_mode", default_value="0",
                              description="初始控制模式: 0=刹车 1=跟车 4=停车 10=前进巡航 11=后运动"),
        DeclareLaunchArgument("ego_odom_topic", default_value="/gz/x1/odometry",
                              description="自车里程计话题"),
        DeclareLaunchArgument("x1_actual_y", default_value="-380.0",
                              description="X1 在 Gazebo 中的实际 Y 坐标 (m)"),

        # ── speed_track_node ──
        Node(
            package=_PKG,
            executable="speed_track_node",
            name="speed_track_node",
            output="screen",
            parameters=[_DEFAULT_PARAMS],
        ),

        # ── test_upstream_node ──
        Node(
            package=_PKG,
            executable="test_upstream_node",
            name="test_upstream_node",
            output="screen",
            parameters=[{
                "target_carriage_id": target_carriage_id,
                "target_mode": target_mode,
                "ego_odom_topic": ego_odom_topic,
                "x1_actual_y": x1_actual_y,
            }],
        ),
    ])
