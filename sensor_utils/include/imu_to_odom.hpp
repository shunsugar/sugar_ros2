#ifndef IMU_TO_ODOM_H
#define IMU_TO_ODOM_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>

class ImuToOdom : public rclcpp::Node
{
public:
  ImuToOdom();

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  rclcpp::Time last_time_;
  geometry_msgs::msg::Vector3 velocity_;
  geometry_msgs::msg::Point position_;
};

#endif // IMU_TO_ODOM_H

