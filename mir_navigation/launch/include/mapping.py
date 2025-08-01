"""
@file mapping.py
@brief Launch file for SLAM mapping with the MiR robot.

This launch file sets up the SLAM Toolbox for mapping with the MiR robot.
It configures the necessary parameters and launches the SLAM node.

@section dependencies Dependencies
- slam_toolbox

@section parameters Launch Parameters
- use_sim_time (bool, default=false): Use simulation time if true.
- slam_params_file (string): Full path to the ROS2 parameters file for slam_toolbox.

@section nodes Nodes
- slam_toolbox: Asynchronous SLAM Toolbox node for mapping.

@section usage Usage
Include this launch file in a main launch file or run directly:
ros2 launch mir_navigation mapping.py slam_params_file:=/path/to/slam_params.yaml
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for SLAM mapping.

    @return LaunchDescription object containing all nodes and parameters.
    """
    slam_configuration = get_package_share_directory('mir_navigation') 
    slam_params_file = os.path.join(slam_configuration, 'config','mir_mapping_async.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ld = LaunchDescription()

    # Declare the use_sim_time parameter
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use real time if false'
    )

    # Configure the SLAM Toolbox node
    launch_mapping = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # Add all actions to the launch description
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(launch_mapping)

    return ld