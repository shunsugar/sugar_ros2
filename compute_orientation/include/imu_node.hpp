#ifndef IMU_NODE_HPP_
#define IMU_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuNode : public rclcpp::Node
{
public:
  ImuNode();
  void computeOrientationData(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publishMsg(const sensor_msgs::msg::Imu::SharedPtr msg);

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr livox_imu_sub_;

  sensor_msgs::msg::Imu _imu_msg;
  std::string imu_topic_;
  std::vector<float> quaternion_;
};

# endif
