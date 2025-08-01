#!/usr/bin/env python3
## @class TopicConfig
#  @brief Configuration for a ROS2 topic.
#
#  This class holds the configuration for a ROS2 topic, including its name,
#  type, and any filters or quality of service profiles that should be applied.

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_system_default,
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)

import time
import copy
import sys
from collections.abc import Iterable

from rclpy_message_converter import message_converter

from geometry_msgs.msg import TwistStamped, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float64
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from mir_msgs.msg import RobotMode, RobotState, Encoders, UserPrompt, Proximity, Pendant
from sdc21x0.msg import StampedEncoders

import mir_driver.rosbridge

import actionlib_msgs.msg
import visualization_msgs.msg


tf_prefix = ''

qos_tf = QoSProfile(
    depth=100,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE  
)

qos_tf_static = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,  
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL  
)

qos_sensors = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

qos_commands = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

qos_odometry = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class TimeFilter():
    def __init__(self):
        self.prev_time = 0
    def test_time(self, time):
        result = time < self.prev_time
        self.prev_time = time 
        return result
    
class TopicConfig(object):
    ## @brief Constructor for TopicConfig.
    #  @param topic The name of the topic.
    #  @param topic_type The type of the topic.
    #  @param topic_renamed Optional renamed topic for ROS2.
    #  @param latch Whether the topic should be latched.
    #  @param dict_filter Optional filter function for the topic's dictionary.
    #  @param qos_profile Optional quality of service profile for the topic.
    def __init__(self, topic, topic_type, topic_renamed=None, latch=False, dict_filter=None,
                 qos_profile=None):
        self.topic = topic
        if (topic_renamed):
            self.topic_ros2_name = topic_renamed
        else:
            self.topic_ros2_name = topic
        self.topic_type = topic_type
        self.latch = latch
        self.dict_filter = dict_filter
        if qos_profile is not None:
            self.qos_profile = qos_profile
        else:
            self.qos_profile = qos_profile_system_default


## @brief Filter function, standard constructor for header dictionaries.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _header_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    return filtered_msg_dict

## @brief Filter function for odometry messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _odom_dict_filter(msg_dict,  to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    filtered_msg_dict['child_frame_id'] = tf_prefix + \
        filtered_msg_dict['child_frame_id'].strip('/')
    return filtered_msg_dict

def _imu_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    # Change the frame ID to "laser"
    filtered_msg_dict['header']['frame_id'] = 'imu_frame' 
    return filtered_msg_dict

## @brief Filter function for transform messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _tf_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)

    for transform in filtered_msg_dict['transforms']:
        transform['child_frame_id'] = tf_prefix + transform['child_frame_id'].strip('/')
        transform['header'] = _convert_ros_header(transform['header'], to_ros2)
    return filtered_msg_dict

## @brief Filter function for laser scan messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _laser_scan_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    # Change the frame ID to "laser"
    #filtered_msg_dict['header']['frame_id'] = 'laser'
    return filtered_msg_dict

def _initialpose_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    return filtered_msg_dict

## @brief Filter function for robot mode messages
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _robot_mode_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['robot_mode'] = filtered_msg_dict.pop('robotMode')
    filtered_msg_dict['robot_mode_string'] = filtered_msg_dict.pop('robotModeString')
    return filtered_msg_dict

## @brief Filter function for robot state messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _robot_state_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['robot_state'] = filtered_msg_dict.pop('robotState')
    filtered_msg_dict['robot_state_string'] = filtered_msg_dict.pop('robotStateString')
    return filtered_msg_dict

## @brief Filter function for marker dictionaries.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _marker_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    filtered_msg_dict['lifetime'] = _convert_ros_time(filtered_msg_dict['lifetime'], to_ros2)
    return filtered_msg_dict

## @brief Filter function for map messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _map_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(
        filtered_msg_dict['header'], to_ros2)
    filtered_msg_dict['info']['map_load_time'] = _convert_ros_time(
        filtered_msg_dict['info']['map_load_time'], to_ros2)
    print('called dict filter')
    return filtered_msg_dict

## @brief Filter function for occupancy grid messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _occupancy_grid_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    filtered_msg_dict['info'] = _map_meta_data_dict_filter(filtered_msg_dict['info'], to_ros2)
    return filtered_msg_dict

## @brief Filter function for map metadata messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _map_meta_data_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['map_load_time'] = _convert_ros_time(filtered_msg_dict['map_load_time'], to_ros2)
    return filtered_msg_dict

## @brief Filter function for goal status array messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _goal_status_array_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    filtered_msg_dict['status_list'] = [
        _goal_status_dict_filter(status, to_ros2) for status in filtered_msg_dict['status_list']
    ]
    return filtered_msg_dict

## @brief Filter function for individual goal status messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _goal_status_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['goal_id']['stamp'] = _convert_ros_time(filtered_msg_dict['goal_id']['stamp'], to_ros2)
    return filtered_msg_dict

## @brief Filter function for diagnostic array messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _diagnostic_array_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    filtered_msg_dict['status'] = [
        _diagnostic_status_dict_filter(status, to_ros2) for status in filtered_msg_dict['status']
    ]
    return filtered_msg_dict

## @brief Filter function for individual diagnostic status messages.
#  @param msg_dict The message dictionary to filter.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The filtered message dictionary.
def _diagnostic_status_dict_filter(msg_dict, to_ros2):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['level'] = bytes([filtered_msg_dict['level'] % 256])
    return filtered_msg_dict

## @brief Convert ROS header between ROS1 and ROS2 formats.
#  @param header_msg_dict The header message dictionary to convert.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The converted header dictionary.
def _convert_ros_header(header_msg_dict, to_ros2):
    header_dict = copy.deepcopy(header_msg_dict)
    header_dict['stamp'] = _convert_ros_time(header_dict['stamp'], to_ros2)
    if to_ros2:
        del header_dict['seq']
        frame_id = header_dict['frame_id'].strip('/')
        header_dict['frame_id'] = tf_prefix + frame_id
    else:  # to ros1
        header_dict['seq'] = 0
        # remove tf_prefix to frame_id

    return header_dict

## @brief Convert ROS time between ROS1 and ROS2 formats.
#  @param time_msg_dict The time message dictionary to convert.
#  @param to_ros2 Boolean indicating the direction of conversion.
#  @return The converted time dictionary.
def _convert_ros_time(time_msg_dict, to_ros2):
    time_dict = copy.deepcopy(time_msg_dict)
    if to_ros2:
        # Conversion from MiR (ros1) to sys (ros2)
        time_dict['nanosec'] = time_dict.pop('nsecs')
        time_dict['sec'] = time_dict.pop('secs')
    else:
        # Conversion from sys (ros2) to MiR (ros1)
        time_dict['nsecs'] = time_dict.pop('nanosec')
        time_dict['secs'] = time_dict.pop('sec')

    return time_dict

####################################################################

## @brief Prepend tf_prefix to frame_id in a message dictionary.
#  @param msg_dict The message dictionary to modify.
#  @return The modified message dictionary.
def _prepend_tf_prefix_dict_filter(msg_dict):
    # filtered_msg_dict = copy.deepcopy(msg_dict)
    if not isinstance(msg_dict, dict):   # can happen during recursion
        return
    for (key, value) in msg_dict.items():
        if key == 'header':
            try:
                # prepend frame_id
                frame_id = value['frame_id'].strip('/')
                if (frame_id != 'map'):
                    # prepend tf_prefix, then remove leading '/' (e.g., when tf_prefix is empty)
                    value['frame_id'] = frame_id.strip('/')
                else:
                    value['frame_id'] = frame_id

            except TypeError:
                pass   # value is not a dict
            except KeyError:
                pass   # value doesn't have key 'frame_id'
        elif isinstance(value, dict):
            _prepend_tf_prefix_dict_filter(value)
        elif isinstance(value, Iterable):    # an Iterable other than dict (e.g., a list)
            for item in value:
                _prepend_tf_prefix_dict_filter(item)
    return msg_dict

## @brief Remove tf_prefix from frame_id in a message dictionary.
#  @param msg_dict The message dictionary to modify.
#  @return The modified message dictionary.
def _remove_tf_prefix_dict_filter(msg_dict):
    # filtered_msg_dict = copy.deepcopy(msg_dict)
    if not isinstance(msg_dict, dict):   # can happen during recursion
        return
    for (key, value) in msg_dict.items():
        if key == 'header':
            try:
                # remove frame_id
                s = value['frame_id'].strip('/')
                if s.find(tf_prefix) == 0:
                    # strip off tf_prefix, then strip leading '/'
                    value['frame_id'] = (s[len(tf_prefix):]).strip('/')
            except TypeError:
                pass   # value is not a dict
            except KeyError:
                pass   # value doesn't have key 'frame_id'
        elif isinstance(value, dict):
            _remove_tf_prefix_dict_filter(value)
        elif isinstance(value, Iterable):    # an Iterable other than dict (e.g., a list)
            for item in value:
                _remove_tf_prefix_dict_filter(item)
    return msg_dict


## @var PUB_TOPICS and SUB_TOPICS
#  @brief List of topics to publish to ROS2 and subscribe from the MiR.
#
#  This list contains configurations for topics that are published to ROS2
#  and subscribed from the MiR, including optional filters and quality of
#  service profiles.
PUB_TOPICS = [
    #TopicConfig('camera_floor_left/floor', PointCloud2, dict_filter=_header_dict_filter), # WORKING
    TopicConfig('camera_floor_left/obstacles', PointCloud2, dict_filter=_header_dict_filter, qos_profile=qos_sensors), # WORKING
    #TopicConfig('camera_floor_right/floor', PointCloud2, dict_filter=_header_dict_filter), # WORKING
    TopicConfig('camera_floor_right/obstacles', PointCloud2, dict_filter=_header_dict_filter, qos_profile=qos_sensors), # WORKING
    TopicConfig('b_scan', LaserScan, dict_filter=_laser_scan_filter, qos_profile=qos_sensors), # WORKING
    TopicConfig('f_scan', LaserScan, dict_filter=_laser_scan_filter, qos_profile=qos_sensors), # WORKING
    TopicConfig('imu_data', Imu, dict_filter=_imu_dict_filter, qos_profile=qos_sensors), # WORKING
    TopicConfig('MC/encoders', StampedEncoders, dict_filter=_header_dict_filter, qos_profile=qos_sensors), # WORKING

    TopicConfig('odom', Odometry, dict_filter=_odom_dict_filter, qos_profile=qos_odometry), # WORKING

    TopicConfig('tf', TFMessage, dict_filter=_tf_dict_filter, topic_renamed='/tf', qos_profile=qos_tf), # WORKING

    TopicConfig('tf_static', TFMessage, dict_filter=_tf_dict_filter, topic_renamed='/tf_static_starter', qos_profile=qos_tf_static), # WORKING
    
    #TopicConfig('robot_mode', RobotMode, dict_filter=_robot_mode_dict_filter), # WORKING
    #TopicConfig('robot_pose', Pose), # it is to respect /map frame
    #TopicConfig('robot_state', RobotState, dict_filter=_robot_state_dict_filter), # WORKING

    #TopicConfig('light_cmd', String), 
    #TopicConfig('PB/proximity', Proximity),

    # TopicConfig('initialpose', PoseWithCovarianceStamped, dict_filter=_initialpose_dict_filter), NOT WORK Qos problem
    # TopicConfig('amcl_pose', mir_msgs.msg.LocalizationPose),

    # TopicConfig('MissionController/prompt_user', UserPrompt),

    # TopicConfig('PB/bms_status', mir_msgs.msg.BMSData),
    # TopicConfig('PB/bms_status_bmz2', mir_msgs.msg.BMSData),
    # TopicConfig('PB/charging_status', mir_msgs.msg.ChargingState),
    # TopicConfig('PB/gpio/input', mir_msgs.msg.Gpio),
    # TopicConfig('PB/gpio/output', mir_msgs.msg.Gpio),
    # TopicConfig('PB/gpio/output/feedback', mir_msgs.msg.Gpio),
    # TopicConfig('PB/gpio/stop_button', String),
    # TopicConfig('PB/motor_status', mir_msgs.msg.PowerBoardMotorStatus),
    # TopicConfig('PB/pallet_lifter_status', mir_msgs.msg.PalletLifterStatus),
    # TopicConfig('PB/pendant', Pendant),
    

    # TopicConfig('SickPLC/safety_sensor_data', mir_sensor_interfaces.msg.SafetySensorDataList),
    # TopicConfig('active_mapping_guid', std_msgs.msg.String),
    
    # TopicConfig('cerberi/safety_cerberus/velocity_profile', driving_interfaces_msgs.msg.VelocityProfile),
    # TopicConfig('check_pose_area/polygon', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('client_count', std_msgs.msg.Int32),
    # TopicConfig('cmd_vel', geometry_msgs.msg.TwistStamped),
    # TopicConfig('connected_clients', rosbridge_msgs.msg.ConnectedClients),

    # TopicConfig('data_events/docking_offsets', mir_data_msgs.msg.DockingOffsetEvent),
    # TopicConfig('data_events/elevator_floors', mir_data_msgs.msg.ElevatorFloorEvent),
    # TopicConfig('data_events/elevators', mir_data_msgs.msg.ElevatorEvent),
    # TopicConfig('data_events/footprints', mir_data_msgs.msg.FootprintEvent),
    # TopicConfig('data_events/maps', mir_data_msgs.msg.MapEvent),
    # TopicConfig('data_events/mission_groups', mir_data_msgs.msg.MissionGroupEvent),
    # TopicConfig('data_events/positions', mir_data_msgs.msg.PositionEvent),
    # TopicConfig('data_events/registers', mir_data_msgs.msg.PLCRegisterEvent),
    # TopicConfig('data_events/sessions', mir_data_msgs.msg.SessionEvent),
    # TopicConfig('data_events/setting_advanceds', mir_data_msgs.msg.SettingEvent),
    # TopicConfig('data_events/settings', mir_data_msgs.msg.SettingEvent),
    # TopicConfig('data_events/sounds', mir_data_msgs.msg.SoundEvent),
    # TopicConfig('data_events/user_groups', mir_data_msgs.msg.UserGroupEvent),
    # TopicConfig('data_events/users', mir_data_msgs.msg.UserEvent),
    # TopicConfig('data_events/zones', mir_data_msgs.msg.AreaEventEvent),

    # TopicConfig('diagnostics', DiagnosticArray),
    # TopicConfig('diagnostics_agg', DiagnosticArray),
    # TopicConfig('diagnostics_toplevel_state', DiagnosticStatus),

    # TopicConfig('embedded_comm_interface/can_embedded_receive_velocity_cmds', std_msgs.msg.Bool),
    # TopicConfig('embedded_comm_interface/embedded/robot_body/errors_interface', embedded_interfaces_msgs.msg.EmbeddedErrors),
    # TopicConfig('global_regulated_cmd_vel', geometry_msgs.msg.TwistStamped),

    # TopicConfig('imu1_debug_data', sensor_msgs.msg.Imu),
    # TopicConfig('imu2_debug_data', sensor_msgs.msg.Imu),

    # TopicConfig('internal_ios/status', mir_msgs.msg.IOs),
    # TopicConfig('lifecycle/camera_floor_left/driver/status', life_cycle_mgmt_interfaces.msg.UpgradeStatus),
    # TopicConfig('lifecycle/camera_floor_right/driver/status', life_cycle_mgmt_interfaces.msg.UpgradeStatus),
    # TopicConfig('lifecycle/embedded/status', life_cycle_mgmt_interfaces.msg.UpgradeStatus),

    
    # TopicConfig('localization_score', Float64),
    # TopicConfig('map', OccupancyGrid),
    # TopicConfig('map_metadata', MapMetaData),

    # TopicConfig('marker_tracking_node/feedback', mir_marker_tracking.msg.MarkerTrackingActionFeedback),
    # TopicConfig('marker_tracking_node/laser_line_extract/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('marker_tracking_node/laser_line_extract/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('marker_tracking_node/laser_line_extract/visualization_marker', visualization_msgs.msg.MarkerArray),
    # TopicConfig('marker_tracking_node/result', mir_marker_tracking.msg.MarkerTrackingActionResult),
    # TopicConfig('marker_tracking_node/status', actionlib_msgs.msg.GoalStatusArray),

    # TopicConfig('mir_amcl/selected_points', PointCloud2),
    # TopicConfig('mir_log', rosgraph_msgs.msg.Log),
    # TopicConfig('mir_safety_warning/interface_component/light_control_interface/light_cmd_topic', std_msgs.msg.String),
    # TopicConfig('mir_sound/sound_event', mir_msgs.msg.SoundEvent),
    # TopicConfig('mir_status_msg', std_msgs.msg.String),
    # TopicConfig('mirspawn/node_events', mirSpawn.msg.LaunchItem),
    # TopicConfig('mirwebapp/grid_map_metadata', mir_msgs.msg.LocalMapStat),
    # TopicConfig('mirwebapp/laser_map_metadata', mir_msgs.msg.LocalMapStat),
    # TopicConfig('mirwebapp/laser_map_pointcloud', PointCloud2),
    # TopicConfig('mirwebapp/obstacle_map_pointcloud', mir_msgs.msg.ObstacleCloud),
    # TopicConfig('mirwebapp/web_path', mir_msgs.msg.WebPath),
    # TopicConfig('mission/ready', mir_mission_interface.msg.Ready),

    # TopicConfig('move_base/cancel', actionlib_msgs.msg.GoalID),
    # TopicConfig('move_base/feedback', mir_nav_interface.msg.MirMoveBaseActionFeedback),
    # TopicConfig('move_base/goal', mir_nav_interface.msg.MirMoveBaseActionGoal),
    # TopicConfig('move_base/result', mir_nav_interface.msg.MirMoveBaseActionResult),
    # TopicConfig('move_base/status', actionlib_msgs.msg.GoalStatusArray),

    # TopicConfig('move_base_node/MIRPlannerROS/local_plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/MIRPlannerROS/visualization_marker', visualization_msgs.msg.MarkerArray),
    # TopicConfig('move_base_node/SBPLLatticeLocalPlanner/initial_smooth_plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/SBPLLatticeLocalPlanner/plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/SBPLLatticeLocalPlanner/sbpl_lattice_planner_stats', sbpl_lattice_planner.msg.SBPLLatticePlannerStats),
    # TopicConfig('move_base_node/SBPLLatticeLocalPlanner/sbpl_plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/SBPLLatticeLocalPlanner/visualization_marker', visualization_msgs.msg.MarkerArray),
    # TopicConfig('move_base_node/SBPLLatticePlanner/initial_smooth_plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/SBPLLatticePlanner/plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/SBPLLatticePlanner/sbpl_lattice_planner_stats', sbpl_lattice_planner.msg.SBPLLatticePlannerStats),
    # TopicConfig('move_base_node/SBPLLatticePlanner/sbpl_plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/SBPLLatticePlanner/visualization_marker', visualization_msgs.msg.MarkerArray),
    # TopicConfig('move_base_node/UI_plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/current_goal', geometry_msgs.msg.PoseStamped),
    # TopicConfig('move_base_node/global_move/len_to_goal', std_msgs.msg.Float64),
    # TopicConfig('move_base_node/local_costmap/costmap_data', mir_msgs.msg.CostmapData),
    # TopicConfig('move_base_node/local_costmap/obstacles', mir_msgs.msg.ObstacleCloud),
    # TopicConfig('move_base_node/local_costmap/robot_footprint', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('move_base_node/local_costmap/safety_zone', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('move_base_node/plan/local_plan', mir_msgs.msg.robot_state_path),
    # TopicConfig('move_base_node/plan/local_trajectory', mir_msgs.msg.TrajectoryPath),
    # TopicConfig('move_base_node/plan/plan_bank_pick', mir_msgs.msg.robot_state_path),
    # TopicConfig('move_base_node/plan/velocity_regulator_plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/plan_handler/active_global_plan', mir_msgs.msg.robot_state_path),
    # TopicConfig('move_base_node/plan_handler/latest_path_added_to_plan_bank', mir_msgs.msg.robot_state_path),
    # TopicConfig('move_base_node/plan_handler/replan_end_poses', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/time_to_coll', std_msgs.msg.Float64),
    # TopicConfig('move_base_node/traversal_time_marker', visualization_msgs.msg.Marker),
    # TopicConfig('move_base_node/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('move_base_node/zone_handler/zones', mir_nav_interface.msg.ZoneList),

    # TopicConfig('move_base_simple/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('moving_state', mir_msgs.msg.MovingState),
    # TopicConfig('navigation_map', mir_msgs.msg.NavigationMap),

    # TopicConfig('odom_enc', nav_msgs.msg.Odometry),
    # TopicConfig('odom_imu1', nav_msgs.msg.Odometry),
    # TopicConfig('odom_imu2', nav_msgs.msg.Odometry),

    # TopicConfig('one_way_map', nav_msgs.msg.OccupancyGrid),
    # TopicConfig('param_manager_update', mir_param_interface.msg.RosParamListUpdate),
    # TopicConfig('param_update', std_msgs.msg.String),
    # TopicConfig('particlevizmarker', visualization_msgs.msg.MarkerArray),
    # TopicConfig('resource_tracker/acquisition', mir_msgs.msg.ResourcesAcquisition),
    # TopicConfig('resource_tracker/needed_resources', mir_msgs.msg.ResourcesState),
    # TopicConfig('resource_tracker/pose_acquisition', geometry_msgs.msg.Pose),


    # TopicConfig('robot_status', mir_msgs.msg.RobotStatus),

    # TopicConfig('rosout', rosgraph_msgs.msg.Log),
    # TopicConfig('rosout_agg', rosgraph_msgs.msg.Log),

    # TopicConfig('safety_status', mir_msgs.msg.SafetyStatus),

    # TopicConfig('scan_filter/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('scan_footpr_filter', sensor_msgs.msg.LaserScan),
    # TopicConfig('scan_raw', sensor_msgs.msg.LaserScan),
    
    # TopicConfig('session_importer_node/info', mirSessionImporter.msg.SessionImportInfo),
    # TopicConfig('skid_detection_cusum', mir_msgs.msg.SkidDetectionStampedFloat),
    # TopicConfig('skid_detection_likelihood', mir_msgs.msg.SkidDetectionStampedFloat),
    # TopicConfig('skid_detection_measurements', mir_msgs.msg.SkidDetectionDiff),
    # TopicConfig('stall_detection', std_msgs.msg.Bool),

    # TopicConfig('traffic_management/envelopes', visualization_msgs.msg.Marker),
    # TopicConfig('traffic_map', nav_msgs.msg.OccupancyGrid),

    # TopicConfig('wifi_diagnostics', diagnostic_msgs.msg.DiagnosticArray),
    # TopicConfig('wifi_diagnostics/cur_ap', mir_wifi_msgs.msg.APInfo),
    # TopicConfig('wifi_diagnostics/roam_events', mir_wifi_msgs.msg.WifiRoamEvent),
    # TopicConfig('wifi_diagnostics/wifi_ap_interface_stats', mir_wifi_msgs.msg.WifiInterfaceStats),
    # TopicConfig('wifi_diagnostics/wifi_ap_rssi', mir_wifi_msgs.msg.APRssiStats),
    # TopicConfig('wifi_diagnostics/wifi_ap_time_stats', mir_wifi_msgs.msg.APTimeStats),
    # TopicConfig('wifi_watchdog/ping', mir_wifi_msgs.msg.APPingStats)
]

# topics we want to subscribe to from ROS2 (and publish to the MiR)
SUB_TOPICS = [
    TopicConfig('cmd_vel', TwistStamped, 'cmd_vel_stamped', qos_profile=qos_commands),
    # TopicConfig('initialpose', geometry_msgs.msg.PoseWithCovarianceStamped),
    # TopicConfig('light_cmd', std_msgs.msg.String),
    # TopicConfig('mir_cmd', std_msgs.msg.String),
    # TopicConfig('move_base/cancel', actionlib_msgs.msg.GoalID),

    # really mir_actions/MirMoveBaseActionGoal:
    # TopicConfig('move_base/goal', move_base_msgs.msg.MoveBaseActionGoal,
    #   dict_filter=_move_base_goal_dict_filter),
    #TopicConfig('robot_state', mir_msgs.msg.RobotState, 'robot_state_ordering'),
]

## @class PublisherWrapper
#  @brief A wrapper class for handling the publication of topics to ROS2.
class PublisherWrapper(object):
    ## Constructor
    #  @param topic_config Configuration for the topic.
    #  @param nh Node handle for ROS2.
    def __init__(self, topic_config, nh):
        self.time_filter = TimeFilter()
        self.node_handle = nh
        self.topic_config = topic_config
        self.robot = nh.robot
        self.connected = False
        self.sub = nh.create_subscription(
            msg_type=topic_config.topic_type,
            topic=topic_config.topic,
            callback=self.callback,
            qos_profile=topic_config.qos_profile
        )
        self.pub = nh.create_publisher(
            msg_type=topic_config.topic_type,
            topic=topic_config.topic_ros2_name,
            qos_profile=topic_config.qos_profile
        )

        nh.get_logger().info("Publishing topic '%s' [%s]" %
                             (topic_config.topic_ros2_name, topic_config.topic_type.__module__))
        # latched topics must be subscribed immediately
        # if topic_config.latch:
        self.peer_subscribe(None, None, None, nh)

    ## Method to handle peer subscription.
    #  @param topic_name Name of the topic.
    #  @param topic_publish Publish function for the topic.
    #  @param peer_publish Peer publish function.
    #  @param nh Node handle for ROS2.
    def peer_subscribe(self, topic_name, topic_publish, peer_publish, nh):
        if not self.connected:
            self.connected = True
            nh.get_logger().info("Starting to stream messages on topic '%s'" %
                                 self.topic_config.topic)
            self.robot.subscribe(
                topic=('/' + self.topic_config.topic), callback=self.callback)
    
    ## Callback method for handling incoming messages.
    #  @param msg_dict Dictionary containing the message data.
    def callback(self, msg_dict):
        if not isinstance(msg_dict, dict):  # Defensive guard for recursion
            return

        msg_dict = _prepend_tf_prefix_dict_filter(msg_dict)

        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict, to_ros2=True)

        try:
            msg = message_converter.convert_dictionary_to_ros_message(
                self.topic_config.topic_type, msg_dict
            )
            self.pub.publish(msg)

        except rclpy.exceptions.ROSInterruptException:
            # ROS shutdown in progress
            pass

        except RuntimeError as e:
            if "destruction was requested" in str(e):
                # Publisher was destroyed during shutdown
                pass
            else:
                self.node_handle.get_logger().warn(
                    f"RuntimeError while publishing on topic '{self.topic_config.topic_ros2_name}': {e}"
                )

        except Exception as e:
            self.node_handle.get_logger().warn(
                f"Error while publishing on topic '{self.topic_config.topic_ros2_name}': {e}"
            )

## @class SubscriberWrapper
#  @brief A wrapper class for handling the subscription of topics from ROS2.
class SubscriberWrapper(object):
    ## Constructor
    #  @param topic_config Configuration for the topic.
    #  @param nh Node handle for ROS2.
    def __init__(self, topic_config, nh):
        self.topic_config = topic_config
        self.robot = nh.robot
        self.sub = nh.create_subscription(
            msg_type=topic_config.topic_type,
            topic=topic_config.topic_ros2_name,
            callback=self.callback,
            qos_profile=topic_config.qos_profile
        )

        nh.get_logger().info("Subscribing to topic '%s' [%s]" % (
            topic_config.topic, topic_config.topic_type.__module__))

    ## Callback method for handling incoming messages.
    #  @param msg The message received from the topic.
    def callback(self, msg):
        if msg is None:
            return

        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        msg_dict = _remove_tf_prefix_dict_filter(msg_dict)

        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict, to_ros2=False)
        self.robot.publish('/' + self.topic_config.topic, msg_dict)

## @class MiR250BridgeNode
#  @brief A ROS2 node for bridging communication between ROS2 and the MiR robot.
class MiR250BridgeNode(Node):
    def __init__(self):
        super().__init__('mir_bridge')

        self.mir_bridge_ready = False  # state
        self.srv_mir_ready = self.create_service(
            Trigger, 'mir_bridge_ready', self.mir_bridge_ready_poll_callback)

        try:
            # Define the hostname and port of the MiR
            hostname = self.declare_parameter(
                'hostname', '<robot-ip>').value
        except KeyError:
            self.get_logger().fatal('Parameter "hostname" is not set!')
            sys.exit(-1)
        port = self.declare_parameter('port', 9090).value
        assert isinstance(port, int), 'port parameter must be an integer'

        global tf_prefix
        self.declare_parameter('tf_prefix', '')
        tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value.strip('/')
        if tf_prefix != "":
            tf_prefix = tf_prefix + '/'

        self.get_logger().info('Trying to connect to %s:%i...' % (hostname, port))
        self.robot = mir_driver.rosbridge.RosbridgeSetup(hostname, port)

        i = 1
        while not self.robot.is_connected():
            if not rclpy.ok():
                #sys.exit(0)
                self.get_logger().info("ROS shutdown detected during connection wait.")
                return
            if self.robot.is_errored():
                self.get_logger().fatal('Connection eMiR250BridgeNode to %s:%i, giving up!'
                                        % (hostname, port))
                #sys.exit(-1)
                rclpy.shutdown()
                return
            if i % 10 == 0:
                self.get_logger().warn('Still waiting for connection to %s:%i...'
                                       % (hostname, port))
            i += 1
            time.sleep(1)
        self.get_logger().info('Connected to %s:%i...' % (hostname, port))

        topics = self.get_topics()
        
        published_topics = [topic_name for (topic_name, _, has_publishers, _)
                            in topics if has_publishers]
        subscribed_topics = [topic_name for (topic_name, _, _, has_subscribers)
                             in topics if has_subscribers]

        for pub_topic in PUB_TOPICS:
            PublisherWrapper(pub_topic, self)
            if ('/' + pub_topic.topic) not in published_topics:
                self.get_logger().warn(
                    "Topic '%s' is NOT published by the MiR!" % pub_topic.topic)

        for sub_topic in SUB_TOPICS:
            SubscriberWrapper(sub_topic, self)
            if ('/' + sub_topic.topic) not in subscribed_topics:
                self.get_logger().warn(
                    "Topic '%s' is NOT yet subscribed to by the MiR!" % sub_topic.topic)
        
        self.mir_bridge_ready = True

    ## Method to retrieve topics from the MiR robot.
    #  @return A list of topics with their types and subscription/publishing status.
    def get_topics(self):
        srv_response = self.robot.callService('/rosapi/topics', msg={})
        topic_names = sorted(srv_response['topics'])
        topics = []

        for topic_name in topic_names:
            srv_response = self.robot.callService(
                "/rosapi/topic_type", msg={'topic': topic_name})
            topic_type = srv_response['type']

            srv_response = self.robot.callService(
                "/rosapi/publishers", msg={'topic': topic_name})
            has_publishers = True if len(
                srv_response['publishers']) > 0 else False

            srv_response = self.robot.callService(
                "/rosapi/subscribers", msg={'topic': topic_name})
            has_subscribers = True if len(
                srv_response['subscribers']) > 0 else False

            topics.append(
                [topic_name, topic_type, has_publishers, has_subscribers])

        #print('PUBLISHERS:')
        for (topic_name, topic_type, has_publishers, has_subscribers) in topics:
            if has_publishers:
                #print((' * %s [%s]' % (topic_name, topic_type)))
                pass

        #print('\nSUBSCRIBERS:')
        for (topic_name, topic_type, has_publishers, has_subscribers) in topics:
            if has_subscribers:
                #print((' * %s [%s]' % (topic_name, topic_type)))
                pass
        return topics

    ## Callback method for checking the readiness of the MiR bridge.
    #  @param request The service request.
    #  @param response The service response.
    #  @return The response indicating the readiness of the bridge.
    def mir_bridge_ready_poll_callback(self, request, response):
        self.get_logger().info('Checked for readiness')
        response.success = self.mir_bridge_ready
        response.message = ""
        return response
    
    def destroy_node(self):
        self.robot.closing()
        super().destroy_node()


## Main function to initialize and run the MiR250BridgeNode.
def main(args=None):
    rclpy.init(args=args)
    node = MiR250BridgeNode()
    node.get_logger().info("╔═══════════════════════════════╗")
    node.get_logger().info("║    MiR Bridge Node STARTED    ║")
    node.get_logger().info("╚═══════════════════════════════╝")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("╔═══════════════════════════════╗")
        print("║ MiR Bridge Node SHUTTING DOWN ║")
        print("╚═══════════════════════════════╝")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
