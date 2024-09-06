#ifndef IMU_NODE_GYRO_HPP_
#define IMU_NODE_GYRO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <array>
#include <cmath>

class ImuNode : public rclcpp::Node
{
public:
  ImuNode();

private:
  void computeOrientation(const sensor_msgs::msg::Imu::SharedPtr msg);
  void multiplyQuaternion(const std::array<double, 4>& q1,
                          const std::array<double, 4>& q2,
                          std::array<double, 4>& q_out);
  void normalizeQuaternion(std::array<double, 4>& q);
  void publishMsg(const sensor_msgs::msg::Imu::SharedPtr msg);

  std::string imu_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr livox_imu_sub_;

  std::array<double, 4> q_;
};

# endif // IMU_NODE_GYRO_HPP_
