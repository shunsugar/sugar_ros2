#include "imu_to_odom.hpp"

ImuToOdom::ImuToOdom() : Node("imu_to_odom")
{
  input_topic_ = this->declare_parameter<std::string>("in_topic", "/imu");
  output_topic_ = this->declare_parameter<std::string>("out_topic", "/odom/imu");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    input_topic_, 10, std::bind(&ImuToOdom::imuCallback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

  last_time_ = this->get_clock()->now();

  velocity_.x = velocity_.y = velocity_.z = 0.0;
  position_.x = position_.y = position_.z = 0.0;
}

void ImuToOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  rclcpp::Time current_time = msg->header.stamp;
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // 速度の更新（v = v + a * dt）
  velocity_.x += msg->linear_acceleration.x * dt;
  velocity_.y += msg->linear_acceleration.y * dt;
  velocity_.z += msg->linear_acceleration.z * dt;

  // 位置の更新（p = p + v * dt）
  position_.x += velocity_.x * dt;
  position_.y += velocity_.y * dt;
  position_.z += velocity_.z * dt;

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = frame_id_;

  odom_msg.pose.pose.position = position_;  // ドリフトを含む
  odom_msg.pose.pose.orientation = msg->orientation;
  odom_msg.twist.twist.linear = velocity_;  // ドリフトを含む
  odom_msg.twist.twist.angular = msg->angular_velocity;

  odom_pub_->publish(odom_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToOdom>());
    rclcpp::shutdown();
    return 0;
}

