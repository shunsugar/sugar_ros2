#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleListenerString(Node):
    def __init__(self):
        super().__init__("simple_listener_string")
        self.string_sub_ = self.create_subscription(String, "/string_msg", self.listener_callback, 10)
        self.subscriptions

    def listener_callback(self, msg):
        self.get_logger().info("Subscribe: '%s'" % msg.data)


def main():
    rclpy.init()
    simple_listener_string = SimpleListenerString()
    rclpy.spin(simple_listener_string)
    simple_listener_string.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
