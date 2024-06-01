#include "imu_node.hpp"

ImuNode::ImuNode() : Node("imu_node")
{
  this->declare_parameter<std::string>("imu_topic", "imu");
  this->get_parameter("imu_topic", imu_topic_);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  livox_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                   "/livox/imu", 10, std::bind(&ImuNode::computeOrientationData, this, std::placeholders::_1));
}

void ImuNode::computeOrientationData(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // RCLCPP_INFO(rclcpp::get_logger(imu_topic_), "Subscribed /livox/imu");
  
  publishMsg(msg);
}

void ImuNode::publishMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // RCLCPP_INFO(rclcpp::get_logger(imu_topic_), "Published %s", imu_topic_.c_str());
  imu_pub_->publish(*msg);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
