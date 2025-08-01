from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'graph_based_navigation_system'
    params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mir_nav_params.yaml'
    )

    map_file = os.path.join(
        get_package_share_directory(package_name),
        'maps',
        'test_TER_1.yaml'  # Update with your actual map file
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the ROS2 parameters file.'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map')}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])
