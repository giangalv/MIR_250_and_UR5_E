"""
@file graph_navigation_launch.py
@brief Launch file for MiR navigation (Nav2 + AMCL) with driver and manual control,
       prepared for running Graph-Based Navigation separately.

This launch file:
- Loads MiR driver
- Initializes AMCL localization
- Starts Nav2 navigation stack
- Launches manual control interface (from mir_manual_navigation)
- Launches RViz for visualization
- Prepares the environment for running graph_nav_controller in a separate terminal

@section dependencies Dependencies
- mir_navigation: Nav2 configurations and AMCL
- mir_driver: MiR robot interface
- mir_manual_navigation: Manual control launch file

@section parameters Launch Parameters
- use_sim_time (bool, default=false): Use simulation time if true
- map (string): Path to map.yaml (absolute or relative to mir_navigation/maps)

@section usage Usage
ros2 launch graph_based_navigation_system graph_navigation_launch.py map:=test_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetLaunchConfiguration, LogInfo, TimerAction, GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    # === Package directories ===
    mir_driver_dir = get_package_share_directory('mir_driver')
    mir_manual_nav_dir = get_package_share_directory('mir_manual_navigation')
    mir_graph_nav_dir = get_package_share_directory('graph_based_navigation_system')
    mir_navigation_dir = get_package_share_directory('mir_navigation')

    # === Launch configurations ===
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace', default='')
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file',
        default=PathJoinSubstitution([mir_graph_nav_dir, 'config', 'graph_based_nav.rviz'])
    )

    # === Resolve map file ===
    def resolve_map_path(context):
        """Resolve map path relative to mir_navigation/maps or as absolute."""
        map_value = context.launch_configurations.get('map', '')

        default_map_path = os.path.join(mir_graph_nav_dir, 'maps', 'test_TER_1_with_nodes_edges.yaml')
        candidate_rel_path = os.path.join(mir_graph_nav_dir, 'maps', map_value)

        if map_value == '':
            return [
                LogInfo(msg='[GRAPH NAV] No map specified. Falling back to default map.'),
                SetLaunchConfiguration('map_file', default_map_path)
            ]
        elif os.path.isfile(candidate_rel_path):
            return [
                LogInfo(msg=f'[GRAPH NAV] Using map (relative): {candidate_rel_path}'),
                SetLaunchConfiguration('map_file', candidate_rel_path)
            ]
        elif os.path.isfile(map_value):
            return [
                LogInfo(msg=f'[GRAPH NAV] Using map (absolute): {map_value}'),
                SetLaunchConfiguration('map_file', map_value)
            ]
        else:
            return [
                LogInfo(msg=f'[GRAPH NAV] Map file "{map_value}" not found. Using fallback default.'),
                SetLaunchConfiguration('map_file', default_map_path)
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
            default_value='',
            description='Path to the map YAML file (relative to mir_navigation/maps or absolute)'
        )
    ]

    # === MiR driver ===    
    mir_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_driver_dir, 'launch', 'mir_launch.py')
        ),
        launch_arguments={
            'rviz_config_file': rviz_config_file,
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }.items()
    )

    # === AMCL ===
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_navigation_dir, 'launch', 'include', 'amcl.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': use_sim_time
        }.items()
    )

    # === Nav2 navigation stack === TO BE CHANGED OR THINK ABOUT HOW TO INTEGRATE WITH THE DYNAMIC GRAPH NAVIGATION SYSTEM
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_navigation_dir, 'launch', 'include', 'navigation.py')
        ),
        launch_arguments={
            'map_subscribe_transient_local': 'true',
            'use_sim_time': use_sim_time
        }.items()
    )

    # === Manual control ===
    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_manual_nav_dir, 'launch', 'manual_control_launch.py')
        )
    )

    # === Grouped dependent launches (delayed) ===
    dependent_launches = GroupAction([
        LogInfo(msg='[GRAPH NAV] Launching AMCL, Nav2 stack, Manual Control, and RViz...'),
        manual_control_launch,
        amcl_launch,
        nav2_launch,
    ])

    # === Delayed start to allow MiR driver to initialize ===
    delayed_start = TimerAction(
        period=30.0,
        actions=[dependent_launches]
    )

    # === Final LaunchDescription ===
    return LaunchDescription([
        *declare_arguments,
        OpaqueFunction(function=resolve_map_path),
        mir_driver_launch,
        delayed_start
    ])
