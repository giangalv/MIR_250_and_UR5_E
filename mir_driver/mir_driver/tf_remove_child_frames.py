#!/usr/bin/env python3

## @package tf_remove_child_frames
#  This module provides a ROS2 node to remove specified child frames from TF messages.

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, qos_profile_system_default

## @class remove_child_frames_node
#  @brief A ROS2 node to filter out specified child frames from TF and TF static messages.
class remove_child_frames_node(Node):

    ## Constructor
    #  Initializes the node and sets up publishers and subscriptions.
    def __init__(self):
        super().__init__('tf_remove_child_frames')
        remove_frames = self.declare_parameter('remove_frames', []).value
        tf_pub = self.create_publisher(
            msg_type=TFMessage,
            topic='tf_out',
            qos_profile=QoSProfile(depth=1)
        )

        ## Callback function for TF messages.
        #  Filters out transforms with child frames specified in the remove_frames parameter.
        #  @param msg The incoming TFMessage.
        def tf_cb(msg):
            msg.transforms = [t for t in msg.transforms
                              if t.child_frame_id.lstrip('/') not in remove_frames]
            if len(msg.transforms) > 0:
                tf_pub.publish(msg)

        self.create_subscription(
            msg_type=TFMessage,
            topic="tf",
            callback=tf_cb,
            qos_profile=qos_profile_system_default
        )

        tf_static_pub = self.create_publisher(
            msg_type=TFMessage,
            topic='tf_static_out',
            qos_profile=QoSProfile(depth=1)
            # latch
        )

        ## Callback function for TF static messages.
        #  Filters out static transforms with child frames specified in the remove_frames parameter.
        #  @param msg The incoming TFMessage.
        def tf_static_cb(msg):
            msg.transforms = [t for t in msg.transforms
                              if t.child_frame_id.lstrip('/') not in remove_frames]
            if len(msg.transforms) > 0:
                tf_static_pub.publish(msg)

        self.create_subscription(
            msg_type=TFMessage,
            topic="tf_static",
            callback=tf_static_cb,
            qos_profile=qos_profile_system_default
        )

## Main function to initialize and run the node.
#  @param args Command line arguments (optional).
def main(args=None):
    rclpy.init(args=args)
    node = remove_child_frames_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
