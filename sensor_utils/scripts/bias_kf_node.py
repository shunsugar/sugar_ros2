#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
import math


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    half = yaw * 0.5
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return math.atan2(t3, t4)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class EKFLocalizer(Node):
    """
    EKF with state: [x, y, theta, b_omega]
    - b_omega: IMU angular velocity bias
    """

    def __init__(self):
        super().__init__("ekf_localizer")

        self.declare_parameter("imu_topic", "/imu/spresense")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gnss_topic", "/odom/UM982/initxy")
        self.declare_parameter("fused_odom_topic", "/odom/kf")

        # Subscribers
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").get_parameter_value().string_value,
            self.imu_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("gnss_topic").get_parameter_value().string_value,
            self.gnss_callback,
            10,
        )

        # Publisher
        self.fused_pub = self.create_publisher(
            Odometry,
            self.get_parameter("fused_odom_topic").get_parameter_value().string_value,
            10,
        )

        # State: [x, y, theta, b_omega]
        self.x = np.array([0.0, 0.0, 0.0, 0.0])
        self.P = np.eye(4) * 0.1

        # Process noise: [x, y, theta, b_omega]
        self.Q = np.diag([0.05, 0.05, (0.01) ** 2, 1e-6])

        # GNSS measurement noise: [x, y, theta]
        self.R_gnss = np.diag([0.5, 0.5, (2 * math.pi / 180) ** 2])

        # IMU bookkeeping
        self.imu_yaw = 0.0
        self.prev_imu_stamp = None

        # Odometry bookkeeping
        self.prev_odom_stamp = None

        self.get_logger().info("EKF localizer with bias initialized.")

    def imu_callback(self, msg: Imu):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.prev_imu_stamp is None:
            self.prev_imu_stamp = t
            return
        dt = t - self.prev_imu_stamp
        self.prev_imu_stamp = t
        if dt <= 0.0 or dt > 1.0:
            return

        wz = msg.angular_velocity.z
        # Bias-corrected yaw integration
        self.imu_yaw += (wz - self.x[3]) * dt
        self.imu_yaw = normalize_angle(self.imu_yaw)

    def odom_callback(self, msg: Odometry):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.prev_odom_stamp is None:
            self.prev_odom_stamp = t
            return
        dt = t - self.prev_odom_stamp
        self.prev_odom_stamp = t
        if dt <= 0.0 or dt > 1.0:
            return

        v = msg.twist.twist.linear.x

        theta = self.x[2]
        b_omega = self.x[3]

        # Prediction
        theta_pred = theta + (self.imu_yaw - theta)  # bias-corrected
        x_pred = self.x[0] + v * dt * math.cos(theta) * 0.45
        y_pred = self.x[1] + v * dt * math.sin(theta) * 0.45
        b_pred = b_omega  # random walk

        self.x = np.array([x_pred, y_pred, theta_pred, b_pred])

        # Jacobian
        F = np.eye(4)
        F[0, 2] = -v * dt * math.sin(theta)
        F[1, 2] = v * dt * math.cos(theta)
        F[2, 3] = -dt  # theta depends on bias

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

        self.publish_fused_odom(msg.header.stamp)

    def gnss_callback(self, msg: Odometry):
        if msg.pose.covariance[0] < 28:
            return

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        # Measurement
        z = np.array([px, py, yaw])

        # Measurement matrix: H = [x, y, theta] part
        H = np.zeros((3, 4))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        # Innovation
        y_innov = z - self.x[:3]
        y_innov[2] = normalize_angle(y_innov[2])

        S = H @ self.P @ H.T + self.R_gnss
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y_innov
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(4) - K @ H) @ self.P

        self.publish_fused_odom(msg.header.stamp)

    def publish_fused_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"

        msg.pose.pose.position.x = self.x[0]
        msg.pose.pose.position.y = self.x[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self.x[2])

        self.fused_pub.publish(msg)


def main():
    rclpy.init()
    node = EKFLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

