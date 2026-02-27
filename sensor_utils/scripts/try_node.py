#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import math


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class YawEstimator(Node):
    def __init__(self):
        super().__init__("yaw_estimator")

        # Parameters
        self.declare_parameter("imu_topic", "/imu/spresense")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gnss_topic", "/odom/UM982/initxy")

        # Publishers
        self.pub_yaw_gnss_ori = self.create_publisher(
            Float64, "/yaw/gnss_orientation", 10
        )
        self.pub_yaw_gnss_diff = self.create_publisher(Float64, "/yaw/gnss_diff", 10)

        self.pub_yaw_imu_int = self.create_publisher(Float64, "/yaw/imu_integrated", 10)
        # self.pub_yaw_imu_w = self.create_publisher(Float64, "/yaw/imu_angular_velocity", 10)

        self.pub_yaw_wheel_ori = self.create_publisher(
            Float64, "/yaw/wheel_orientation", 10
        )
        self.pub_yaw_wheel_int = self.create_publisher(
            Float64, "/yaw/wheel_integrated", 10
        )

        # Subscribers
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").get_parameter_value().string_value,
            self.imu_callback,
            500,
        )

        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            self.odom_callback,
            50,
        )

        self.create_subscription(
            Odometry,
            self.get_parameter("gnss_topic").get_parameter_value().string_value,
            self.gnss_callback,
            10,
        )

        # Internal states
        self.prev_imu_time = None
        self.imu_yaw = 0.0

        self.prev_wheel_time = None
        self.wheel_yaw = 0.0

        self.prev_gnss_x = None
        self.prev_gnss_y = None

        self.get_logger().info("Yaw estimator (6 outputs) started")

    # -------------------------
    # IMU
    # -------------------------
    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # publish raw angular velocity
        w = msg.angular_velocity.z
        # self.pub_yaw_imu_w.publish(Float64(data=w))

        if self.prev_imu_time is None:
            self.prev_imu_time = t
            return

        dt = t - self.prev_imu_time
        self.prev_imu_time = t

        self.imu_yaw += w * dt
        self.imu_yaw = normalize_angle(self.imu_yaw)

        self.pub_yaw_imu_int.publish(Float64(data=self.imu_yaw))

    # -------------------------
    # Wheel Odometry
    # -------------------------
    def odom_callback(self, msg: Odometry):
        # orientation yaw
        yaw_ori = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pub_yaw_wheel_ori.publish(Float64(data=yaw_ori))

        # angular velocity integration
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        w = msg.twist.twist.angular.z

        if self.prev_wheel_time is None:
            self.prev_wheel_time = t
            return

        dt = t - self.prev_wheel_time
        self.prev_wheel_time = t

        self.wheel_yaw += w * dt
        self.wheel_yaw = normalize_angle(self.wheel_yaw)

        self.pub_yaw_wheel_int.publish(Float64(data=self.wheel_yaw))

    # -------------------------
    # GNSS Odometry
    # -------------------------
    def gnss_callback(self, msg: Odometry):
        # ① orientation from dual antenna
        yaw_ori = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pub_yaw_gnss_ori.publish(Float64(data=yaw_ori))

        # ② position difference
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.prev_gnss_x is None:
            self.prev_gnss_x = x
            self.prev_gnss_y = y
            return

        dx = x - self.prev_gnss_x
        dy = y - self.prev_gnss_y

        if math.hypot(dx, dy) < 1e-3:
            return

        yaw_diff = math.atan2(dy, dx)
        self.pub_yaw_gnss_diff.publish(Float64(data=yaw_diff))

        self.prev_gnss_x = x
        self.prev_gnss_y = y


def main():
    rclpy.init()
    node = YawEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
