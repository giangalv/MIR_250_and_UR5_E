#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_msgs.msg import TFMessage
import threading

class TFRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')
        
        # Parameters
        self.declare_parameter('publish_frequency', 10.0)
        self.publish_frequency = self.get_parameter('publish_frequency').value
        
        # TF infrastructure
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Thread-safe storage
        self.latest_transforms = []
        self.lock = threading.Lock()
        self._shutdown = False
        
        # Subscriber
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf_unfiltered',
            self.tf_callback,
            10)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.timer_callback)
        
        self.get_logger().info(f"TF Republisher started at {self.publish_frequency}Hz")

    def tf_callback(self, msg):
        if self._shutdown:
            return
            
        with self.lock:
            self.latest_transforms = msg.transforms

    def timer_callback(self):
        if self._shutdown or not self.latest_transforms:
            return
            
        with self.lock:
            tf_message = TFMessage()
            tf_message.transforms = self.latest_transforms
            self.tf_broadcaster.sendTransform(tf_message.transforms)

    def shutdown(self):
        self._shutdown = True
        self.get_logger().info("Shutting down TF Republisher...")
        self.timer.cancel()
        self.tf_subscription.destroy()

def main():
    rclpy.init()
    node = TFRepublisher()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.remove_node(node)
            node.destroy_node()
        except:
            pass

if __name__ == '__main__':
    main()