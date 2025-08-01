"""
@file manual_control_launch.py
@brief Launch file for manual control of the MiR robot.

This launch file sets up nodes for manual control, including twist multiplexer, keyboard/joystick teleop, and a custom manual mode node.

@section dependencies Dependencies
- twist_mux
- teleop_twist_keyboard
- joy
- teleop_twist_joy
- mir_manual_navigation

@section parameters Launch Parameters
- namespace (string, default=""): Namespace for launched nodes.
- use_sim_time (bool, default=false): Use simulation time if true.

@section nodes Nodes
- twist_mux: Prioritizes control sources (keyboard/joystick).
- teleop_twist_keyboard: Keyboard teleoperation.
- joy_node: Reads joystick input.
- teleop_twist_joy: Converts joystick input to Twist messages.
- manual_mode: Custom manual navigation logic.

@section usage Usage
ros2 launch mir_manual_navigation manual_control_launch.py [parameters]
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for manual control nodes."""
    manual_navigation_dir = get_package_share_directory('mir_manual_navigation')

    # Launch configurations with defaults
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Paths to configuration files
    twist_config = os.path.join(manual_navigation_dir, 'config', 'twist_mux.yaml')
    joy_config = os.path.join(manual_navigation_dir, 'config', 'joy_config.yaml')
    joy_initialization = os.path.join(manual_navigation_dir, 'config', 'joy_init.yaml')

    # Declare launch arguments (for overriding defaults)
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all topics/TF prefixes'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Nodes
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_config, {'use_sim_time': use_sim_time}],
        namespace=namespace,
    )

    keyboard_teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        namespace=namespace,
        prefix='xterm -e',  # Open in a new terminal window
        output='screen',
        remappings=[('cmd_vel', 'cmd_vel_keyb')],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_initialization, {'use_sim_time': use_sim_time}],
        namespace=namespace,
    )

    twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[joy_config, {'use_sim_time': use_sim_time}],
        namespace=namespace,
        remappings=[('cmd_vel', 'cmd_vel_joy')],
    )

    mir_restapi_server_node = Node(
        package='mir_restapi',
        executable='mir_restapi_server',
        name='mir_restapi_server',
        parameters=[{'use_sim_time': use_sim_time}],
        namespace=namespace,
        output='screen',
    )
    controller_node = Node(
        package='mir_restapi',
        executable='mir_control_node',
        name='mir_control_node',
        parameters=[{'use_sim_time': use_sim_time}],
        namespace=namespace,
        prefix='xterm -e',
        output='screen',
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        twist_mux_node,
        keyboard_teleop_node,
        joy_node,
        twist_joy_node,
        mir_restapi_server_node,
        controller_node,
    ])