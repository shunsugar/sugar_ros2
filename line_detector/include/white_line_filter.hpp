#ifndef WHITE_LINE_FILTER_HPP
#define WHITE_LINE_FILTER_HPP

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class WhiteLineFilter : public rclcpp::Node
{
public:
  WhiteLineFilter();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr white_line_pub_;

  std::string input_topic_;
  std::string output_topic_;
};

#endif // WHITE_LINE_FILTER_HPP

