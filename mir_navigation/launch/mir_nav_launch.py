import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetLaunchConfiguration, TimerAction, LogInfo, GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    mir_driver_dir = get_package_share_directory('mir_driver')
    mir_nav_dir = get_package_share_directory('mir_navigation')
    mir_manual_nav_dir = get_package_share_directory('mir_manual_navigation')

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file',
        default=PathJoinSubstitution([mir_nav_dir, 'rviz', 'mir_nav.rviz'])
    )

    def resolve_map_path(context):
        map_arg = context.launch_configurations.get('map', '')
        if not map_arg:
            return [SetLaunchConfiguration('map_file', '')]
        rel_path = os.path.join(mir_nav_dir, 'maps', map_arg)
        if os.path.isfile(rel_path):
            return [SetLaunchConfiguration('map_file', rel_path)]
        elif os.path.isfile(map_arg):
            return [SetLaunchConfiguration('map_file', map_arg)]
        return [SetLaunchConfiguration('map_file', '')]

    declare_arguments = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace for all nodes and topics'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map',
            description='Relative path to map in mir_navigation/maps or absolute path to map.yaml'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([mir_nav_dir, 'rviz', 'mir_nav.rviz']),
            description='Full path to RViz configuration file'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([mir_nav_dir, 'config', 'mir_mapping_async.yaml']),
            description='Full path to SLAM parameters file'),
    ]

    mir_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_driver_dir, 'launch', 'mir_launch.py')
        ),
        launch_arguments={
            'rviz_config_file': rviz_config_file,
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }.items()
    )

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_nav_dir, 'launch', 'include', 'amcl.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': use_sim_time
        }.items()
    )

    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_nav_dir, 'launch', 'include', 'navigation.py')
        ),
        launch_arguments={
            'map_subscribe_transient_local': 'true',
            'use_sim_time': use_sim_time
        }.items()
    )

    manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_manual_nav_dir, 'launch', 'manual_control_launch.py')
        )
    )

    # Group the dependent launch files together
    dependent_launches = GroupAction([
        LogInfo(msg="Launching AMCL, Nav2 stack, and Manual Control..."),
        amcl,
        navigation_stack,
        manual_control
    ])

    # Delay to give the MiR driver time to initialize 
    delayed_start = TimerAction(
        period=30.0,  # seconds
        actions=[dependent_launches]
    )

    return LaunchDescription([
        *declare_arguments,
        OpaqueFunction(function=resolve_map_path),
        mir_driver,
        delayed_start
    ])
