#ifndef IMU_NODE_HPP_
#define IMU_NODE_HPP_

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
  void multiplyQuaternion(double w1, double x1, double y1, double z1,
                          double w2, double x2, double y2, double z2,
                          double &w_out, double &x_out, double &y_out, double &z_out);
  void normalizeQuaternion(double &w, double &x, double &y, double &z);
  void publishMsg(const sensor_msgs::msg::Imu::SharedPtr msg);

  std::string imu_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr livox_imu_sub_;

  double q_w_, q_x_, q_y_, q_z_;
};

# endif // IMU_NODE_HPP_
