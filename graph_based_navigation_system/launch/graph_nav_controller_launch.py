"""
@file graph_nav_controller_launch.py
@brief Launch file for the Graph Navigation Controller node.

This launch file:
- Starts only the graph_nav_controller node from graph_based_navigation_system.
- It is meant to be run alongside the main navigation stack (graph_navigation_launch.py).

@section dependencies Dependencies
- graph_based_navigation_system package (with graph_nav_controller.py executable)

@section parameters Launch Parameters
- use_sim_time (bool, default=false): Use simulation clock if true.

@section usage Usage
ros2 launch graph_based_navigation_system graph_nav_controller_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # === Declare arguments ===
    declare_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        )
    ]

    # === Graph Navigation Controller Node ===
    graph_nav_node = Node(
        package='graph_based_navigation_system',
        executable='graph_nav_controller.py',
        name='graph_nav_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        *declare_arguments,
        graph_nav_node
    ])
