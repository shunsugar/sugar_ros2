#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import csv
import os


class YawCsvSaver(Node):
    def __init__(self):
        super().__init__("yaw_csv_saver")

        # 保存対象の yaw トピック
        self.target_topics = [
            "/yaw/gnss_orientation",
            "/yaw/gnss_diff",
            "/yaw/imu_integrated",
            # "/yaw/imu_angular_velocity",
            "/yaw/wheel_orientation",
            "/yaw/wheel_integrated",
        ]

        # 出力CSVファイル
        self.csv_file = "yaw_topics.csv"

        # CSVヘッダー
        self.header = [
            "topic_name",
            "time_stamp",
            "yaw",  # [rad]
        ]

        # CSV初期化
        self.init_csv_file()

        # Subscribers
        self.subs = []
        for topic in self.target_topics:
            sub = self.create_subscription(
                Float64,
                topic,
                lambda msg, t=topic: self.listener_callback(msg, t),
                100,
            )
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to: {topic}")

    def init_csv_file(self):
        """CSVファイルを新規作成"""
        if os.path.exists(self.csv_file):
            os.remove(self.csv_file)

        with open(self.csv_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(self.header)

    def listener_callback(self, msg: Float64, topic_name: str):
        """
        共通コールバック
        """
        # ROS time（受信時刻）
        now = self.get_clock().now().nanoseconds * 1e-9

        row = [
            topic_name,
            now,
            msg.data,
        ]

        # CSVへ追記
        with open(self.csv_file, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = YawCsvSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

