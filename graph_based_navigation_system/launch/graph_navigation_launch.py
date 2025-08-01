"""
@file graph_navigation_launch.py
@brief Launch file for MiR navigation (Nav2 + AMCL) with driver and manual control,
       prepared for running Graph-Based Navigation separately.

This launch file:
- Loads MiR driver
- Initializes AMCL localization
- Starts Nav2 navigation stack
- Launches manual control interface (from mir_manual_navigation)
- Prepares the environment for running graph_nav_controller in a separate terminal

@section dependencies Dependencies
- mir_navigation: Nav2 configurations and AMCL
- mir_driver: MiR robot interface
- mir_manual_navigation: Manual control launch file

@section parameters Launch Parameters
- use_sim_time (bool, default=false): Use simulation time if true
- map (string): Path to map.yaml (absolute or relative to mir_navigation/maps)

@section usage Usage
ros2 launch graph_based_navigation_system graph_navigation_launch.py map:=path/to/map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    mir_nav_dir = get_package_share_directory('mir_navigation')
    mir_driver_dir = get_package_share_directory('mir_driver')
    mir_manual_nav_dir = get_package_share_directory('mir_manual_navigation')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_arg = LaunchConfiguration('map')

    def resolve_map_path(context):
        """Resolve map path relative to mir_navigation/maps or as absolute."""
        map_value = context.launch_configurations.get('map', '')
        candidate_rel_path = os.path.join(mir_nav_dir, 'maps', map_value)

        if map_value == '':
            return [LogInfo(msg='[GRAPH NAV] No map specified. Launch may fail.'), SetLaunchConfiguration('map_file', '')]

        if os.path.isfile(candidate_rel_path):
            return [SetLaunchConfiguration('map_file', candidate_rel_path)]
        elif os.path.isfile(map_value):
            return [SetLaunchConfiguration('map_file', map_value)]
        else:
            return [
                LogInfo(msg=f'[GRAPH NAV] Map file "{map_value}" not found. Launch may fail.'),
                SetLaunchConfiguration('map_file', '')
            ]

    # === Declare arguments ===
    declare_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            description='Path to the map YAML file (relative to mir_navigation/maps or absolute)'
        )
    ]

    # === Include MiR driver ===
    mir_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_driver_dir, 'launch', 'mir_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # === Include AMCL ===
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_nav_dir, 'launch', 'include', 'amcl.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': use_sim_time
        }.items()
    )

    # === Include Nav2 stack ===
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_nav_dir, 'launch', 'include', 'navigation.py')
        ),
        launch_arguments={
            'map_subscribe_transient_local': 'true',
            'use_sim_time': use_sim_time
        }.items()
    )

    # === Include Manual Control ===
    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_manual_nav_dir, 'launch', 'manual_control_launch.py')
        )
    )

    return LaunchDescription([
        *declare_arguments,
        OpaqueFunction(function=resolve_map_path),
        mir_driver_launch,
        amcl_launch,
        nav2_launch,
        manual_control_launch
    ])
