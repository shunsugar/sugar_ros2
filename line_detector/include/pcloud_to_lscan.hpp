#ifndef PCLOUD_TO_LSCAN_HPP
#define PCLOUD_TO_LSCAN_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudToLaserScan : public rclcpp::Node
{
public:
  PointCloudToLaserScan();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;

  std::string input_topic_;
  std::string output_topic_;
};

#endif // PCLOUD_TO_LSCAN_HPP

