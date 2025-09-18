#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import os
import tf2_ros
import tf_transformations

class InitialPositionNode(Node):
    def __init__(self):
        super().__init__('initial_position_node')

        # File to save/load the last position
        self.file_path = os.path.join(os.path.expanduser("~"), ".ros", "mir_position.txt")

        # Publisher for the current pose
        self.pose_pub = self.create_publisher(Pose2D, 'robot_pose2d', 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Load last saved position at startup
        self.last_pose = self.load_position()
        if self.last_pose:
            self.get_logger().info(
                f"Using saved position at startup -> x: {self.last_pose.x:.2f}, y: {self.last_pose.y:.2f}, yaw: {self.last_pose.theta:.2f} rad"
            )
        else:
            self.get_logger().info("No previous position found, starting from (0,0,0)")
            self.last_pose = Pose2D(x=0.0, y=0.0, theta=0.0)

        # Timer to update pose from TF and publish (1 Hz)
        self.publish_timer = self.create_timer(1.0, self.update_and_publish_pose)

        # Timer to periodically save the pose (every 5 seconds)
        self.save_timer = self.create_timer(5.0, self.save_position)

    def load_position(self):
        """Load position from file once."""
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, "r") as f:
                    line = f.readline().strip()
                    if line:
                        x, y, theta = map(float, line.split())
                        return Pose2D(x=x, y=y, theta=theta)
            except Exception as e:
                self.get_logger().warn(f"Failed to load position: {e}")
        return None

    def save_position(self):
        """Save the last known position to file."""
        try:
            os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
            with open(self.file_path, "w") as f:
                f.write(f"{self.last_pose.x} {self.last_pose.y} {self.last_pose.theta}\n")
            #self.get_logger().info(f"Position saved -> x: {self.last_pose.x:.2f}, y: {self.last_pose.y:.2f}, yaw: {self.last_pose.theta:.2f} rad")
        except Exception as e:
            self.get_logger().error(f"Failed to save position: {e}")

    def update_and_publish_pose(self):
        """Update pose from TF if available, then publish."""
        try:
            # Look up map -> base_link transform (global localization pose)
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )

            # Update last_pose with TF data
            self.last_pose.x = x
            self.last_pose.y = y
            self.last_pose.theta = yaw

            self.get_logger().debug(
                f"Updated pose from TF: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            )

        except Exception:
            # If TF fails, keep using the loaded pose
            self.get_logger().debug("TF not available yet, using loaded pose.")
            
        # Publish the current pose (from TF if available, else loaded)
        self.pose_pub.publish(self.last_pose)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected, shutting down...")
    finally:
        node.save_position()  # ensure final save
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
