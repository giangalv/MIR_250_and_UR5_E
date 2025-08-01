"""
@file navigation.py
@brief Launch file for real-robot navigation with the MiR robot.

This launch file configures the Nav2 stack for physical MiR robot operation with:
- Real-time clock (no simulation time)
- Default behavior trees for physical navigation
- Optimized parameters for real-world operation
- Safety-focused velocity smoothing
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get package directory
    mir_nav_dir = get_package_share_directory('mir_navigation')

    # Configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    log_level = LaunchConfiguration('log_level')

    # Real-robot behavior tree path
    default_bt_xml_path = os.path.join(
        mir_nav_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')

    # Lifecycle nodes to manage
    lifecycle_nodes = [
        'bt_navigator',
        'controller_server',
        #'map_saver',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'waypoint_follower',
        'velocity_smoother'
        #'map_server',
        #'robot_state_publisher'
    ]

    # Parameter substitutions for real-robot
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'default_nav_to_pose_bt_xml': default_bt_xml_path,
        'default_nav_through_poses_bt_xml': default_bt_xml_path,
        'controller_server.controller_frequency': '20.0',  
        'planner_server.expected_planner_frequency': '20.0', # '1.0'
        'behavior_server.bt_loop_duration': '100'
    }

    # Configure parameters
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True)

    # Launch arguments
    declare_args = [
        # Core configuration
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'),
            
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
            
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(mir_nav_dir, 'config', 'mir_nav_params.yaml'), 
            description='Full path to the ROS2 parameters file'),
            
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
            
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use composed bringup if True'),
            
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel_nav',
            description='Output command velocity topic'),
            
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'),
    ]

    # Node definitions
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('cmd_vel', cmd_vel_topic)],
        respawn=True)

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level])

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level])

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('cmd_vel', cmd_vel_topic)])

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level])

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level])

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('cmd_vel', cmd_vel_topic),
            ('cmd_vel_smoothed', cmd_vel_topic)])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': lifecycle_nodes}])

    # Create launch description
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # Add declared arguments
    for arg in declare_args:
        ld.add_action(arg)

    # Add nodes
    ld.add_action(controller_server)
    ld.add_action(smoother_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)
    ld.add_action(lifecycle_manager)

    return ld