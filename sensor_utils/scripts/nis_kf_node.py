#!/usr/bin/env python3
import rclpy
import numpy as np
import math
import tf2_ros

from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped


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
    EKF with state X = [x, y, theta]
    - IMU callback integrates angular velocity to maintain imu_yaw
    - Odometry callback provides linear velocity for prediction
    - GNSS callback provides position + heading for update
    """

    def __init__(self):
        super().__init__("ekf_localizer_nis")

        self.declare_parameter("imu_topic", "/imu/spresense")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gnss_topic", "/odom/UM982/initxy")
        self.declare_parameter("fused_odom_topic", "/odom/kf_nis")
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", False)

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
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("gnss_topic").get_parameter_value().string_value,
            self.gnss_callback,
            10,
        )

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter("fused_odom_topic").get_parameter_value().string_value,
            10,
        )
        self.odom_msg = Odometry()

        self.publish_odom = (
            self.get_parameter("publish_odom").get_parameter_value().bool_value
        )
        self.publish_tf = (
            self.get_parameter("publish_tf").get_parameter_value().bool_value
        )

        self.t = TransformStamped()
        self.br = tf2_ros.TransformBroadcaster(self)

        # State: [x, y, theta]
        self.x = np.array([0.0, 0.0, 0.0])
        self.P = np.eye(3) * 0.1

        # Process noise: [x, y, theta]
        self.Q = np.diag([0.025, 0.025, (0.01) ** 2])
        # GNSS measurement noise: [x, y, theta]
        self.R_gnss = np.diag([0.01, 0.01, (2 * math.pi / 180) ** 2])

        self.SAT_NUM_THRESHOLD = 28
        self.NIS_CHI2_THRESHOLD = 7.815

        self.WAIT_SEC_SAT_NUM = 15.0
        # self.WAIT_SEC_NIS_DIFF = 5.0

        self.MAX_UPDATE_DIST = 0.2

        self.SCALE_FACTOR = 0.45

        self.imu_yaw = 0.0
        self.prev_imu_stamp = None
        self.prev_odom_stamp = None

        self.gnss_resume_time = 0.0

        self.get_logger().info("EKF localizer initialized.")

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
        self.imu_yaw += wz * dt
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
        wz_odom = msg.twist.twist.angular.z

        # Use IMU yaw rate if available
        delta_yaw = normalize_angle(self.imu_yaw - self.x[2])
        if abs(delta_yaw) < 1e-6:
            delta_yaw = wz_odom * dt

        # Prediction step
        theta_pred = self.x[2] + delta_yaw
        theta_pred = normalize_angle(theta_pred)
        dx = v * dt * math.cos(self.x[2]) * self.SCALE_FACTOR
        dy = v * dt * math.sin(self.x[2]) * self.SCALE_FACTOR

        self.x[0] += dx
        self.x[1] += dy
        self.x[2] = theta_pred

        # Jacobian F
        F = np.eye(3)
        F[0, 2] = -v * dt * math.sin(self.x[2])
        F[1, 2] = v * dt * math.cos(self.x[2])

        # Covariance propagation
        self.P = F @ self.P @ F.T + self.Q

        self.publish_fused_odom(msg.header.stamp)

    def gnss_callback(self, msg: Odometry):
        stamp = msg.header.stamp
        curr_stamp = stamp.sec + stamp.nanosec * 1e-9

        # Check wait time after bad GNSS
        if curr_stamp < self.gnss_resume_time:
            return

        # Check the number of satellites
        sat_num = msg.pose.covariance[0]
        if sat_num < self.SAT_NUM_THRESHOLD:
            self.gnss_resume_time = curr_stamp + self.WAIT_SEC_SAT_NUM
            #self.get_logger().info(f"GNSS update skipped due to low satellite count: {sat_num}")
            return

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        z = np.array([px, py, yaw])

        H = np.eye(3)
        y_innov = z - self.x
        y_innov[2] = normalize_angle(y_innov[2])

        S = H @ self.P @ H.T + self.R_gnss

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            #self.get_logger().warn("Singular matrix encountered in GNSS update step.")
            return

        # Check NIS
        nis = y_innov.T @ S_inv @ y_innov
        if nis > self.NIS_CHI2_THRESHOLD:
            # self.gnss_resume_time = curr_stamp + self.WAIT_SEC_NIS_NUM
            #self.get_logger().warn("GNSS measurement rejected by NIS check.")
            return

        K = self.P @ H.T @ S_inv

        corr = K @ y_innov

        # Clamp large updates
        dist_corr = math.sqrt(corr[0] ** 2 + corr[1] ** 2)
        if dist_corr >= self.MAX_UPDATE_DIST:
            scale = self.MAX_UPDATE_DIST / dist_corr
            corr *= scale
            #self.get_logger().info(f"Update clamped! {dist_corr:.2f} -> {self.MAX_UPDATE_DIST}m")

        # State update
        self.x = self.x + corr
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P

        self.publish_fused_odom(msg.header.stamp)

    def publish_fused_odom(self, stamp):
        if self.publish_odom:
            self.odom_msg.header.stamp = stamp
            self.odom_msg.header.frame_id = "odom"
            self.odom_msg.child_frame_id = "base_footprint"
            self.odom_msg.pose.pose.position.x = self.x[0]
            self.odom_msg.pose.pose.position.y = self.x[1]
            self.odom_msg.pose.pose.position.z = 0.0
            self.odom_msg.pose.pose.orientation = yaw_to_quaternion(self.x[2])
            self.odom_pub.publish(self.odom_msg)

        if self.publish_tf:
            self.t.header.stamp = stamp
            self.t.header.frame_id = "odom"
            self.t.child_frame_id = "base_footprint"
            self.t.transform.translation.x = self.x[0]
            self.t.transform.translation.y = self.x[1]
            self.t.transform.translation.z = 0.0
            self.t.transform.rotation = yaw_to_quaternion(self.x[2])
            self.br.sendTransform(self.t)


def main():
    rclpy.init()
    node = EKFLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

