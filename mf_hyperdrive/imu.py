import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import time
import math


class TestPublisher(Node):

    def __init__(self):
        super().__init__('imu')
        self.publisher_ = self.create_publisher(PoseStamped, 'pose', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.start_time = time.perf_counter()

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = math.sin(time.perf_counter() - self.start_time) 
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher_.publish(msg)
#        self.get_logger().info(f"Z position: {str(msg.pose.position.z)}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher = TestPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

