#include "white_line_filter.hpp"

WhiteLineFilter::WhiteLineFilter() : Node("white_line_filter")
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/lidar_points");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/filtered_points");

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, 10, std::bind(&WhiteLineFilter::pointCloudCallback,
    this, std::placeholders::_1));
  white_line_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "White Line Filter Node Started.");
}

void WhiteLineFilter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert PointCloud2 to PCL data type
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  // Filter by reflection intensity using PassThrough filter
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("intensity");
  pass.setFilterLimits(30.0, std::numeric_limits<float>::max());
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pass.filter(*filtered_cloud);

  // Convert filtered PCL data to PointCloud2 and publish
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*filtered_cloud, output);
  output.header = msg->header;
  white_line_pub_->publish(output);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhiteLineFilter>());
  rclcpp::shutdown();
  return 0;
}

