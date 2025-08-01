#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sdc21x0.msg import StampedEncoders

class EncoderToJointState(Node):
    def __init__(self):
        super().__init__('encoder_to_joint_state')
        self.sub = self.create_subscription(
            StampedEncoders,
            '/MC/encoders',
            self.encoder_callback,
            10
        )
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = ['left_wheel_joint', 'right_wheel_joint']
        self.ticks_to_rad = (2 * 3.1415926535) / 1000.0 # It depends on the Encoder

    def encoder_callback(self, msg):
        joint_state = JointState()
        joint_state.header.stamp = msg.header.stamp  
        joint_state.name = self.joint_names
        joint_state.position = [
            msg.encoders.left_wheel * self.ticks_to_rad,
            msg.encoders.right_wheel * self.ticks_to_rad
        ]
        self.pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown() 


if __name__ == '__main__':
    main()