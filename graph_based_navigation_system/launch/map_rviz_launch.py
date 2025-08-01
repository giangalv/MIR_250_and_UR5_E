import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_yaml_path = os.path.join(
        get_package_share_directory('graph_based_navigation_system'),
        'maps',
        'test_TER_1_with_nodes_edges.yaml'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('graph_based_navigation_system'),
        'config',
        'graph_based_nav.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map',
            default_value=map_yaml_path,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=rviz_config_path,
            description='Full path to RViz2 config file'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'yaml_filename': LaunchConfiguration('map')}
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen',
        ),

        # Use ExecuteProcess to call ros2 lifecycle set ...
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                    output='screen'
                )
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
