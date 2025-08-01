import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Wait for TF transformation
        self.timer = self.create_timer(1.0, self.publish_initial_pose)

    def publish_initial_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            quat = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

            self.get_logger().info(f"Initial Pose: x={x}, y={y}, yaw={yaw}")

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation = quat
            msg.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                                   0, 0.25, 0, 0, 0, 0,
                                   0, 0, 0.25, 0, 0, 0,
                                   0, 0, 0, 0.0685, 0, 0,
                                   0, 0, 0, 0, 0.0685, 0,
                                   0, 0, 0, 0, 0, 0.0685
                                ]
            '''
            -> 0.25 for (x, y, z) → Moderate uncertainty in position (±0.5m).
            -> 0.0685 for (roll, pitch, yaw) → Small uncertainty in orientation (~±5°).
            -> 0 in other places → No cross-correlation between position and orientation.

            pose.covariance = [
                XX,  XY,  XZ,  XRoll, XPitch, XYaw,
                XY,  YY,  YZ,  YRoll, YPitch, YYaw,
                XZ,  YZ,  ZZ,  ZRoll, ZPitch, ZYaw,
                XRoll, YRoll, ZRoll, RollRoll, RollPitch, RollYaw,
                XPitch, YPitch, ZPitch, RollPitch, PitchPitch, PitchYaw,
                XYaw, YYaw, ZYaw, RollYaw, PitchYaw, YawYaw
            ]

            This tells AMCL not to trust the given pose much, allowing it to adjust more.
            msg.pose.covariance = [ 
                1.0, 0, 0, 0, 0, 0,
                0, 1.0, 0, 0, 0, 0,
                0, 0, 1.0, 0, 0, 0,
                0, 0, 0, 0.5, 0, 0,
                0, 0, 0, 0, 0.5, 0,
                0, 0, 0, 0, 0, 0.5
            ]
            ''' 
            self.publisher_.publish(msg)
            self.get_logger().info("Published Initial Pose!")
            self.timer.cancel()  # Stop after publishing once

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
