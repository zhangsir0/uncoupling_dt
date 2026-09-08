import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('magnetic_guide_py_controller'),
        'config',
        'magnetic_guide_params.yaml'
    )

    yhs_pkg = 'yhs_can_control'
    magnetic_driver_pkg = 'magnetic_guide_driver'
    can_driver_pkg = 'can_driver'

    magnetic_guide_pid_controller_node = Node(
        package='magnetic_guide_py_controller',
        executable='magnetic_guide_pid_node',
        name='magnetic_guide_pid_controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

        # Include yhs_can_control launch file
    yhs_launch_file = os.path.join(
        get_package_share_directory(yhs_pkg),
        'launch',
        'yhs_can_control.launch.py'
    )
    
    yhs_can_control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yhs_launch_file)
    )

    # Include magnetic_guide_driver launch file
    magnetic_driver_launch_file = os.path.join(
        get_package_share_directory(magnetic_driver_pkg),
        'launch',
        'magnetic_guide_driver.launch.py'
    )
    
    magnetic_driver_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(magnetic_driver_launch_file)
    )

    # Include can_driver launch xml file
    can_driver_launch_file = os.path.join(
        get_package_share_directory(can_driver_pkg),
        'launch',
        'can_driver_node_socket.launch.xml'
    )
    
    can_driver_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(can_driver_launch_file)
    )
    # Velocity controller node
    velocity_controller_node = Node(
        package='magnetic_guide_controller',
        executable='yhs_velocity_controller.py',
        name='yhs_velocity_controller',
        output='screen',
        emulate_tty=True
    )
    return LaunchDescription([
        magnetic_guide_pid_controller_node,
        velocity_controller_node, # (如果您之前有定义这个节点请解开注释并补充定义)
        yhs_can_control_include,
        magnetic_driver_include,
        can_driver_include
    ])