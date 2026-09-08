import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'magnetic_guide_controller'
    config_file_name = 'controller_params.yaml'
    
    # 配置文件路径
    config_dir = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        config_file_name
    )

    # 声明可配置的启动参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config_dir,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Controller 节点
    magnetic_controller_node = Node(
        package=pkg_name,
        executable='magnetic_guide_controller_node',
        name='magnetic_guide_controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        magnetic_controller_node
    ])