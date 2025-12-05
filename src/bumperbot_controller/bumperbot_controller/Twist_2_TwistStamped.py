#!/usr/bin/env python3

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.sub = self.create_subscription(Twist, '/key_vel', self.cb, 10)
        self.pub = self.create_publisher(TwistStamped, 'bumperbot_controller/cmd_vel', 10)

    def cb(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = msg
        self.pub.publish(stamped)

def main():
    rclpy.init()

    twist_to_stamped = TwistToStamped()
    rclpy.spin(twist_to_stamped)
    
    twist_to_stamped.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()