#include "laserscan_to_costmap.hpp"

LaserScanToCostmap::LaserScanToCostmap() : Node("laserscan_to_costmap")
{
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/merged_scan", qos_profile, std::bind(&LaserScanToCostmap::laserScanCallback, this, std::placeholders::_1));

    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    // コストマップのパラメータを初期化
    this->declare_parameter<double>("map_resolution", 0.1); // 1セルの大きさ[m]
    this->declare_parameter<int>("map_width", 100);        // マップの幅[セル]
    this->declare_parameter<int>("map_height", 100);       // マップの高さ[セル]
    this->declare_parameter<double>("origin_x", -5.0);     // マップの原点X座標
    this->declare_parameter<double>("origin_y", -5.0);     // マップの原点Y座標

    this->get_parameter("map_resolution", map_resolution_);
    this->get_parameter("map_width", map_width_);
    this->get_parameter("map_height", map_height_);
    this->get_parameter("origin_x", origin_x_);
    this->get_parameter("origin_y", origin_y_);

    // OccupancyGridの基本設定
    costmap_.header.frame_id = "map";
    costmap_.info.resolution = map_resolution_;
    costmap_.info.width = map_width_;
    costmap_.info.height = map_height_;
    costmap_.info.origin.position.x = origin_x_;
    costmap_.info.origin.position.y = origin_y_;
    costmap_.info.origin.orientation.w = 1.0;
    costmap_.data.resize(map_width_ * map_height_, -1); // -1は未知セル
}

void LaserScanToCostmap::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    // コストマップをリセット（初期化）
    std::fill(costmap_.data.begin(), costmap_.data.end(), 0); // 0は移動可能セル

    // LaserScanデータをコストマップに変換
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
        float range = scan_msg->ranges[i];
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

        // 有効なデータのみ処理
        if (range >= scan_msg->range_min && range <= scan_msg->range_max)
        {
            // 世界座標系での点の位置を計算
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);

            // マップ座標系に変換
            int map_x = static_cast<int>((x - origin_x_) / map_resolution_);
            int map_y = static_cast<int>((y - origin_y_) / map_resolution_);

            // マップの範囲内か確認
            if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_)
            {
                // コストマップデータを更新（障害物は100）
                costmap_.data[map_y * map_width_ + map_x] = 100;
            }
        }
    }

    // コストマップをパブリッシュ
    costmap_.header.stamp = this->get_clock()->now();
    costmap_pub_->publish(costmap_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanToCostmap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

