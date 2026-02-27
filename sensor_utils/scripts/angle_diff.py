#!/usr/bin/env python3
import rclpy
import numpy as np
import math
import tf2_ros
import csv
import os

from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

# デバッグ用に追加
from std_msgs.msg import Float32


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
        super().__init__("ekf_localizer_diff")

        self.declare_parameter("imu_topic", "/imu/spresense")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gnss_topic", "/odom/UM982/initxy")
        self.declare_parameter("fused_odom_topic", "/odom/kf_diff")
        self.declare_parameter("publish_odom", False)
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
        # --- 追加: デバッグ用Publisher（これで効果をグラフ化できます） ---
        # self.debug_pub = self.create_publisher(Float32, "/debug/vector_angle_diff", 10)

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
        self.R_gnss = np.diag([3.0, 3.0, (2 * math.pi / 180) ** 2])

        self.SAT_NUM_THRESHOLD = 28
        self.VEC_DIFF_DEG_THRESHOLD = 10.0

        self.WAIT_SEC_SAT_NUM = 15.0
        self.WAIT_SEC_VEC_DIFF = 5.0

        self.MAX_UPDATE_DIST = 0.2

        self.SCALE_FACTOR = 0.45

        # --- 追加: 信頼度減衰のためのパラメータ ---
        self.current_gnss_covariance_scale = 1.0
        # 0.95の場合、毎回5%ずつしか信頼度が回復しない（1.0に戻るのに時間がかかる）
        # 10HzのGNSSなら、約2秒で ~0.3倍まで減衰
        self.RECOVERY_DECAY_RATE = 0.9

        self.imu_yaw = 0.0
        self.prev_imu_stamp = None
        self.prev_odom_stamp = None

        self.prev_gnss_stamp = None
        self.prev_gnss_pos = None
        self.prev_pred_pos = np.array([0.0, 0.0])

        self.gnss_resume_time = 0.0

        self.csv_filename = "angle_diff_log.csv"
        self.f = open(self.csv_filename, "w", newline="")
        self.writer = csv.writer(self.f)
        self.writer.writerow(["timestamp", "angle_diff_deg"])

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

        """
        # Check wait time
        if curr_stamp < self.gnss_resume_time:
            return

        # 1. Satellite Count Check
        sat_num = msg.pose.covariance[0]
        if sat_num < self.SAT_NUM_THRESHOLD:
            self.gnss_resume_time = curr_stamp + self.WAIT_SEC_SAT_NUM
            self.prev_gnss_pos = None
            self.get_logger().info(f"Low Sat: {sat_num}")
            return
        """

        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        # Initialization check
        if self.prev_gnss_pos is None:
            self.prev_gnss_stamp = curr_stamp
            self.prev_gnss_pos = pos
            self.prev_pred_pos = np.array([self.x[0], self.x[1]])
            return

        dt = curr_stamp - self.prev_gnss_stamp
        if dt <= 0.0 or dt > 10.0:
            self.prev_gnss_stamp = curr_stamp
            self.prev_gnss_pos = pos
            return

        # --- 改善点1: 幾何学的整合性の計算とRの適応的拡大 ---

        dp_gnss_vec = pos - self.prev_gnss_pos
        dp_pred_vec = np.array([self.x[0], self.x[1]]) - self.prev_pred_pos
        norm_pred = np.linalg.norm(dp_pred_vec)

        # 瞬時的なインフレーション係数の計算
        instant_inflation = 1.0

        if norm_pred > 0.2:  # 移動量が小さいときは方向計算が不安定なのでスキップ
            cross = dp_gnss_vec[0] * dp_pred_vec[1] - dp_gnss_vec[1] * dp_pred_vec[0]
            dot = dp_gnss_vec[0] * dp_pred_vec[0] + dp_gnss_vec[1] * dp_pred_vec[1]
            angle_rad = math.atan2(abs(cross), dot)
            angle_deg = math.degrees(angle_rad)

            # ログ保存
            self.writer.writerow([curr_stamp, angle_deg])
            self.f.flush()

            # デバッグ値をPublish（rqt_plotなどで確認してください）
            # debug_msg = Float32()
            # debug_msg.data = float(angle_deg)
            # self.debug_pub.publish(debug_msg)

            # 閾値判定（完全な外れ値は除去）
            # if angle_deg > self.VEC_DIFF_DEG_THRESHOLD:  # 例: 20度以上ならGate
            #    self.get_logger().warn(f"Geometric Reject: {angle_deg:.1f} deg")
            #    self.current_gnss_covariance_scale = 100.0
            #    self.gnss_resume_time = curr_stamp + 2.0
            #    return

            # 角度差による瞬時スケーリング
            if angle_deg > 15:
                angle_deg = 15
            instant_inflation = 1.0 + (angle_deg**2) * 2.0

            # 【対策1】 指数関数的にペナルティを与える
            # angle_deg が 0に近い -> 1.0
            # angle_deg が 5度 -> 1 + exp(4.0) ≈ 55
            # angle_deg が 10度 -> 1 + exp(8.0) ≈ 3000 (ほぼ更新停止)
            # instant_inflation = 1.0 + math.exp(angle_deg * 0.5)

        # --- 改善点: 信頼度の平滑化（ヒステリシス） ---
        # 1. 前回のスケールを少し減衰させる (例: 50.0 -> 49.0)
        decayed_scale = self.current_gnss_covariance_scale * self.RECOVERY_DECAY_RATE

        # 2. 今回計算した瞬時値と比較し、大きい方を採用する
        #    これにより、「急激な悪化」には即座に反応し、「回復」はゆっくりになる
        self.current_gnss_covariance_scale = max(instant_inflation, decayed_scale)

        # 下限処理（1.0未満にはならないように）
        if self.current_gnss_covariance_scale < 1.0:
            self.current_gnss_covariance_scale = 1.0

        # 【対策2】 信頼度が極端に低い場合は更新をスキップする
        # Rを極大にして計算するよりも、ここで止めたほうが計算負荷も低く確実です
        INFLATION_CUTOFF_THRESHOLD = 200.0  # チューニング箇所
        if self.current_gnss_covariance_scale > INFLATION_CUTOFF_THRESHOLD:
            # self.get_logger().warn(f"Updates skipped due to high uncertainty: factor {self.current_gnss_covariance_scale:.1f}")
            self.prev_gnss_stamp = curr_stamp
            self.prev_gnss_pos = pos
            return  # ここで帰る

        # Rに適用
        R_curr = self.R_gnss.copy()
        R_curr *= self.current_gnss_covariance_scale

        if self.current_gnss_covariance_scale > 2.0:
            # self.get_logger().info(f"R scaled by x{self.current_gnss_covariance_scale:.2f}")
            pass

        self.prev_gnss_stamp = curr_stamp
        self.prev_gnss_pos = pos

        # --- EKF Update Step ---

        px = pos[0]
        py = pos[1]
        # Orientation from GNSS is often unreliable, consider ignoring theta update or giving huge variance
        yaw_gnss = quaternion_to_yaw(msg.pose.pose.orientation)
        z = np.array([px, py, yaw_gnss])

        H = np.eye(3)
        y_innov = z - self.x
        y_innov[2] = normalize_angle(y_innov[2])

        # S = H P H^T + R_adaptive
        S = H @ self.P @ H.T + R_curr

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        # --- 改善点2: NIS (Normalized Innovation Squared) Check ---
        # Gate threshold for 3 degrees of freedom (Chi-square distribution)
        # 95%: 7.81, 99%: 11.34
        nis = y_innov.T @ S_inv @ y_innov
        NIS_THRESHOLD = 7.81

        if nis > NIS_THRESHOLD:
            self.current_gnss_covariance_scale = max(
                self.current_gnss_covariance_scale, 5.0
            )
            self.get_logger().warn(f"NIS Reject: {nis:.2f}")
            # NIS棄却時は更新しない（外れ値）
            return

        # Compute Kalman Gain
        K = self.P @ H.T @ S_inv
        corr = K @ y_innov

        # 【対策3】 信頼度に応じて最大補正距離を制限する
        # 信頼度が低い(scaleが大きい)ときは、補正リミットを小さくする
        # scale=1.0 -> limit=0.2m
        # scale=100 -> limit=0.002m
        dynamic_limit = self.MAX_UPDATE_DIST / (
            self.current_gnss_covariance_scale**0.3
        )  # 平方根などでマイルドに

        # 安全のため下限を設定 (例: 1cm以下にはしない)
        dynamic_limit = max(dynamic_limit, 0.01)

        # --- Clamp logic (Safety Net) ---
        dist_corr = math.sqrt(corr[0] ** 2 + corr[1] ** 2)
        if dist_corr >= dynamic_limit:
            # クランプ処理は最後の安全策として残す
            scale = dynamic_limit / dist_corr
            corr *= scale
            # self.get_logger().info(f"Clamp: {dist_corr:.2f}->{self.MAX_UPDATE_DIST}")

        # Update State
        self.x = self.x + corr
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P

        self.prev_pred_pos = np.array([self.x[0], self.x[1]])

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

    def destroy_node(self):
        self.f.close()
        self.get_logger().info("CSV file closed.")
        super().destroy_node()


def main():
    rclpy.init()
    node = EKFLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

