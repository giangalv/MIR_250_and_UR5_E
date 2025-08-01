"""
@file mir_launch.py
@brief Launch file for MiR robot with teleoperation and RViz2 visualization.

This launch file sets up the Rviz2 visualization and the foxglove visualization

@section dependencies Dependencies
- rviz2
- mir_description
- foxglove_bridge

@section parameters Launch Parameters
- namespace (string, default=""): Namespace to push all topics into.
- use_sim_time (bool, default=false): Use simulation time if true.
- rviz_config_file (string, default="mir_visu_full.rviz"): Path to the RViz configuration file.

@section nodes Nodes
- rviz2: Visualization tool for monitoring the MiR robot state.
- foxglove_bridge: Visualization tool for monitoring the MiR robot state.

@section usage Usage
ros2 launch mir_description mir_launch.py [parameters]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Generate a ROS2 launch description.

    This function sets up the launch description for running RViz2 and Foxglove bridge for a MiR robot.
    It declares necessary launch arguments and starts the required nodes.

    @return LaunchDescription: The launch description containing the defined nodes and arguments.
    """
    
    # Get the package share directory for mir_description
    mir_description_dir = get_package_share_directory('mir_description')
    scan_merger_dir = get_package_share_directory('sensors_launcher_mir_250')
    
    # Define launch configurations
    use_sim_time_standard = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time') 

    rviz_config_file_standard = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(mir_description_dir, 'rviz', 'mir_manual_control.rviz'),
        description='Define RViz config file to be used'
    )
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    standard_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace to use'
    )
    name_space = LaunchConfiguration('namespace')

    laser_merger_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_merger_dir, 'launch', 'sensors_launcher.launch.py')),
        launch_arguments={
            'namespace': name_space,
            'use_sim_time': use_sim_time
        }.items()
    )

    cloud_merger_launcher = Node(
        package='mir_manual_navigation',
        executable='cloud_transformation',
        name='cloud_transformation',
    )

    rviz_launcher = Node(
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    foxglove_path = os.path.join(get_package_share_directory('foxglove_bridge'), 
                                  'launch', 'foxglove_bridge_launch.xml')
    
    foxglove_launcher = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(foxglove_path)
    )
    

    return LaunchDescription([
        use_sim_time_standard,
        rviz_config_file_standard,
        standard_namespace,
        laser_merger_launcher,
        #cloud_merger_launcher,
        rviz_launcher,
        #foxglove_launcher,
    ])