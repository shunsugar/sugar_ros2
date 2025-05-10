#ifndef WHEEL_IMU_ODOM_H
#define WHEEL_IMU_ODOM_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomFusionNode : public rclcpp::Node
{
public:
  OdomFusionNode();

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string imu_topic_;
  std::string odom_topic_;
  std::string fused_odom_topic_;
  std::string odom_header_frame_;
  std::string odom_child_frame_;
  std::string TF_header_frame_;
  std::string TF_child_frame_;
  bool publish_odom_;
  bool publish_TF_;
  bool debug_;

  double yaw_;
  double x_;
  double y_;
};

#endif // WHEEL_IMU_ODOM_H

