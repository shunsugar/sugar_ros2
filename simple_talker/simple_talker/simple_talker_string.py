#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleTalkerString(Node):
    def __init__(self):
        super().__init__("simple_talker_string")
        self.string_pub_ = self.create_publisher(String, "/string_msg", 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Hello world! %d" % self.i
        self.string_pub_.publish(msg)
        self.get_logger().info("Publish: '%s'" % msg.data)
        self.i += 1


def main():
    rclpy.init()
    simple_talker_string = SimpleTalkerString()
    rclpy.spin(simple_talker_string)
    simple_talker_string.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
