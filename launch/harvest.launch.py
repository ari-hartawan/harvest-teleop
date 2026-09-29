import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('harvest_teleop')
    config_file = os.path.join(pkg_share, 'config', 'kinova_gen3_7dof_params.yaml') 

    arm_mode = context.perform_substitution(LaunchConfiguration('arm_mode'))
    port_right = LaunchConfiguration('port_right') 
    port_left = LaunchConfiguration('port_left') 

    nodes_to_start = []

    # ==================================================================
    # LENGAN KANAN TELEOP
    # ==================================================================
    if arm_mode in ['both', 'right_only']:
        teleop_right_node = Node(
            package='harvest_teleop',
            executable='teleop_node',
            name='harvest_teleop_core_node',
            namespace='right_', 
            parameters=[config_file, {'hardware.port_name': port_right}],
            output='screen', 
            emulate_tty=True 
        )
        nodes_to_start.append(teleop_right_node)

    # ==================================================================
    # LENGAN KIRI TELEOP
    # ==================================================================
    if arm_mode in ['both', 'left_only']:
        teleop_left_node = Node(
            package='harvest_teleop',
            executable='teleop_node',
            name='harvest_teleop_core_node',
            namespace='left_', 
            parameters=[config_file, {'hardware.port_name': port_left}],
            output='screen', 
            emulate_tty=True 
        )
        nodes_to_start.append(teleop_left_node)

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm_mode', default_value='both', choices=['both', 'right_only', 'left_only']),
        
        DeclareLaunchArgument(
            'port_right', 
            default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14113015-if00',
            description='Persistent USB port for RIGHT master device'
        ),
        DeclareLaunchArgument(
            'port_left', 
            default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14029857-if00',
            description='Persistent USB port for LEFT master device'
        ),
        
        OpaqueFunction(function=launch_setup)
    ])