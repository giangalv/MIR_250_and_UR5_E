
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sensors_launcher_mir_250',
            executable='dual_laser_merger_node',
            name='dual_laser_merger',
            output='screen',
        ),
        Node(
            package='sensors_launcher_mir_250',
            executable='camera_right_changer_node',
            name='camera_right_changer',
            output='screen',
        ),
        Node(
            package='sensors_launcher_mir_250',
            executable='camera_left_changer_node',
            name='camera_left_changer',
            output='screen',
        )
    ])