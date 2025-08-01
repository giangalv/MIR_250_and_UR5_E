#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
from tf2_ros import Buffer, TransformListener


class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')

        # Use BEST_EFFORT reliability for sensor data
        qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        qos_sensors = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # TF2 setup for frame transformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_left = None  # Store full message
        self.latest_right = None  # Store full message

        self.subscription_left = self.create_subscription(
            PointCloud2, 
            '/camera_floor_left/obstacles', 
            self.left_callback, 
            qos)
        
        self.subscription_right = self.create_subscription(
            PointCloud2, 
            '/camera_floor_right/obstacles', 
            self.right_callback, 
            qos)

        self.publisher = self.create_publisher(
            PointCloud2, 
            '/camera_floor/obstacles', 
            20)

        self.timer = self.create_timer(0.1, self.publish_merged)  # Publish at ~10 Hz

    def left_callback(self, msg: PointCloud2):
        self.latest_left = msg  # Store full message

    def right_callback(self, msg: PointCloud2):
        self.latest_right = msg  # Store full message

    def publish_merged(self):
        if self.latest_left is None and self.latest_right is None:
            return  # Wait until at least one message is received

        # Extract points from messages if available
        left_points = np.array([
            [p[0], p[1], p[2]] for p in pc2.read_points(self.latest_left, field_names=("x", "y", "z"), skip_nans=True)
        ]) if self.latest_left else np.empty((0, 3))

        right_points = np.array([
            [p[0], p[1], p[2]] for p in pc2.read_points(self.latest_right, field_names=("x", "y", "z"), skip_nans=True)
        ]) if self.latest_right else np.empty((0, 3))

        '''
        # Determine frame_id based on most recent available message
        if self.latest_left:
            frame_id = self.latest_left.header.frame_id
        elif self.latest_right:
            frame_id = self.latest_right.header.frame_id
        else:
            frame_id = "odom"  # Fallback
        '''
        # Use a fixed frame_id for simplicity
        frame_id = "base_link"

        # Merge or use available points
        if left_points.size == 0 and right_points.size == 0:
            # Publish an empty PointCloud2 message
            empty_msg = self.create_empty_pointcloud2(frame_id)
            self.publisher.publish(empty_msg)
        else:
            combined_points = np.vstack((left_points, right_points)) if left_points.size and right_points.size else (
                left_points if left_points.size else right_points
            )
            merged_msg = self.numpy_to_pointcloud2(combined_points, frame_id=frame_id)
            self.publisher.publish(merged_msg)
    
    def create_empty_pointcloud2(self, frame_id="base_link"):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Return an empty PointCloud2 message
        return PointCloud2(
            header=header,
            height=1,
            width=0,
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=0,
            data=[],
            is_dense=True
        )

    def numpy_to_pointcloud2(self, points, frame_id="map"): # "odom"
        """Converts a Nx3 numpy array to a ROS2 PointCloud2 message with proper timestamp."""
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()  # Use current time
        header.frame_id = frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg = pc2.create_cloud(header, fields, points)
        msg.is_dense = True # I trust the data
        msg.height = 1
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    try:        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
