#include "pcloud_to_lscan.hpp"

PointCloudToLaserScan::PointCloudToLaserScan() : Node("pcloud_to_lscan")
{
  input_topic_ = this->declare_parameter<std::string>("in_topic", "/filtered_points");
  output_topic_ = this->declare_parameter<std::string>("out_topic", "/scan");

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, 10, std::bind(&PointCloudToLaserScan::pointCloudCallback, this, std::placeholders::_1));
  laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
}

void PointCloudToLaserScan::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  sensor_msgs::msg::LaserScan scan;
  scan.header = msg->header;
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 180.0;
  scan.time_increment = 0.0;
  scan.scan_time = 0.1;
  scan.range_min = 0.0;
  scan.range_max = 100.0;
  scan.ranges.resize(360, std::numeric_limits<float>::infinity());

  for (auto& point : cloud->points)
  {
    float angle = std::atan2(point.y, point.x);
    int index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);
    float range = std::sqrt(point.x * point.x + point.y * point.y);
    if (range >= scan.range_min && range <= scan.range_max)
    {
      scan.ranges[index] = range;
    }
  }

  laserscan_pub_->publish(scan);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToLaserScan>());
  rclcpp::shutdown();
  return 0;
}

