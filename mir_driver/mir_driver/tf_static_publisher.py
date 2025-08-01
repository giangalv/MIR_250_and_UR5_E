#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import subprocess
from ament_index_python.packages import get_package_share_directory
import os


class TFStaticRepublisher(Node):
    def __init__(self):
        super().__init__('tf_static_republisher')
        
        # QoS for tf_static: reliable and transient local
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static_starter', self.tf_static_callback, qos_profile)
        
        self.tf_pub = self.create_publisher(TFMessage, '/tf_static', qos_profile)

        self.cached_transforms = []
        self.published = False

        self.add_virtual_laser_static_transform()

    def add_virtual_laser_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = Time(sec=0, nanosec=0)
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'virtual_laser_link'

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.2

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.cached_transforms.append(transform)

    def tf_static_callback(self, msg):
        if self.published:
            return  # Already published once

        for transform in msg.transforms:
            # Exclude transforms involving 'map' or 'odom'
            if transform.header.frame_id in ['map', 'odom'] or transform.child_frame_id in ['map', 'odom']:
                continue
            
            # Only add if not already present
            if not any(t.child_frame_id == transform.child_frame_id and t.header.frame_id == transform.header.frame_id
                       for t in self.cached_transforms):
                transform.header.stamp = Time(sec=0, nanosec=0)
                self.cached_transforms.append(transform)

        self.tf_pub.publish(TFMessage(transforms=self.cached_transforms))
        self.published = True
        self.get_logger().info(f"Published {len(self.cached_transforms)} static transforms to /tf_static.")


def main():
    rclpy.init()
    node = TFStaticRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()