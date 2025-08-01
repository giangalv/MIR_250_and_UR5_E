"""
@file mir_full_launch.py
@brief Main launch file for MiR robot including driver, navigation, and manual control.

This launch file combines:
- MiR driver and visualization
- SLAM mapping functionality
- Manual control interface

@section dependencies Dependencies
- mir_driver
- mir_navigation
- mir_manual_navigation

@section parameters Launch Parameters
- namespace (string, default=""): Namespace for all topics.
- use_sim_time (bool, default=false): Use simulation time if true.
- navigation_enabled (bool, default=false): Enable navigation stack.
- slam_params_file (string): Path to SLAM configuration file.
- rviz_config_file (string): Path to RViz configuration file.

@section included_launches Included Launch Files
- mir_launch.py: Driver and visualization
- mapping.py: SLAM configuration
- manual_control_launch.py: Manual teleoperation

@section usage Usage
ros2 launch mir_navigation mir_full_launch.py [parameters]
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """Generate launch description for full MiR system."""
    # Package directories
    mir_driver_dir = get_package_share_directory('mir_driver')
    mir_nav_dir = get_package_share_directory('mir_navigation')
    mir_manual_nav_dir = get_package_share_directory('mir_manual_navigation')

    # Launch configurations with defaults
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=PathJoinSubstitution([mir_nav_dir, 'config', 'mir_mapping_async.yaml'])
    )
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file',
        default=PathJoinSubstitution([mir_nav_dir, 'rviz', 'mir_mapping_manual.rviz']) # mir_mapping.rviz
    )

    # Declare launch arguments
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

    declare_navigation = DeclareLaunchArgument(
        'navigation_enabled',
        default_value='false',
        description='Enable navigation stack for mapping'
    )

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([mir_nav_dir, 'config', 'mir_mapping_async.yaml']),
        description='SLAM configuration parameters'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([mir_nav_dir, 'rviz', 'mir_mapping_manual.rviz']), # mir_mapping.rviz
        description='RViz configuration file'
    )

    # Include other launch files
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_driver_dir, 'launch', 'mir_launch.py')
        ),
        launch_arguments={
            'rviz_config_file': rviz_config_file,
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }.items()
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_nav_dir, 'launch', 'include', 'mapping.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            'namespace': namespace
        }.items()
    )

    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_manual_nav_dir, 'launch', 'manual_control_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_navigation,
        declare_slam_params,
        declare_rviz_config,
        
        driver_launch,
        mapping_launch,
        manual_control_launch
    ])