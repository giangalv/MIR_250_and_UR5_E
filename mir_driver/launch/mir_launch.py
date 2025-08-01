import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    mir_description_dir = get_package_share_directory('mir_description')
    mir_driver_dir = get_package_share_directory('mir_driver')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')

    # Declare launch arguments (for overriding defaults)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all topics/TF prefixes'
    )

    # Nodes
    mir_bridge_node = Node(
        package='mir_driver',
        executable='mir_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tf_prefix': namespace
        }],
        namespace=namespace,
        output='screen'
    )

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper_cmd_vel_mir',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': ''  # Empty frame_id
        }],
        remappings=[
            ('cmd_vel_in', 'cmd_vel_out'),
            ('cmd_vel_out', 'cmd_vel_stamped'),
        ],
        namespace=namespace,
    )

    # Include other launch files
    robot_displays_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'mir_250_launch.py')
        )
    )

    additions_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_driver_dir, 'launch', 'additions_launch.py')
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        mir_bridge_node,
        twist_stamper_node,
        robot_displays_launcher,
        additions_launcher,
    ])