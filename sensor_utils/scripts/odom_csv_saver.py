#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os


class OdomCsvSaver(Node):
    def __init__(self):
        super().__init__("odom_csv_saver")

        self.target_topics = [
            "/odom/wheel_spimu",
            "/odom/UM982/initxy",
            "/fusion/odom",
            "/odom/kf_yaw",
            "/odom/kf_sat",
            "/odom/kf_vec",
            "/odom/kf_nis",
            "/odom/kf_my",
            "/odom/kf_my2",
            "/odom_fast",
        ]

        self.csv_file = "topic_odom.csv"

        # CSVヘッダーの定義（ご指定の10項目）
        self.header = [
            "topic_name",
            "time_stamp",
            "pose_x",
            "pose_y",
            "pose_z",
            "orientation_w",
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "pose_covariance_0",  # 共分散行列の最初の要素(xの分散)
        ]

        # ファイルの初期化
        self.init_csv_file()

        # サブスクライバーの動的生成
        self.subs = []
        for topic in self.target_topics:
            # lambdaを使って、コールバック時にトピック名を渡せるようにする
            sub = self.create_subscription(
                Odometry, topic, lambda msg, t=topic: self.listener_callback(msg, t), 10
            )
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to: {topic}")

    def init_csv_file(self):
        """CSVファイルを作成し、ヘッダーを書き込む"""
        if os.path.exists(self.csv_file):
            os.remove(self.csv_file)  # 既存ファイルがあれば削除して新規作成

        with open(self.csv_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(self.header)

    def listener_callback(self, msg, topic_name):
        """共通のデータ処理・保存関数"""

        # タイムスタンプ (秒.ナノ秒)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # データの抽出
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        cov0 = msg.pose.covariance[0]  # 共分散の0番目の要素

        # 書き込む行データ
        row = [topic_name, timestamp, p.x, p.y, p.z, o.w, o.x, o.y, o.z, cov0]

        # CSVへ追記
        # ※頻繁なファイルオープンは低速ですが、データ破損リスク低減のためここでは都度書き込みます
        with open(self.csv_file, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCsvSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

