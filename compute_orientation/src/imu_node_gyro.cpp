#include "imu_node_gyro.hpp"

ImuNode::ImuNode() : Node("imu_node"), q_{1.0, 0.0, 0.0, 0.0}
{
  this->declare_parameter<std::string>("imu_topic", "/imu");
  this->get_parameter("imu_topic", imu_topic_);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  livox_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                   "/livox/imu", 10, std::bind(&ImuNode::computeOrientation, this, std::placeholders::_1));
}

void ImuNode::computeOrientation(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // RCLCPP_INFO(rclcpp::get_logger(imu_topic_), "Subscribed /livox/imu");

  // Obtain angular_velocity data
  double omega_x = msg->angular_velocity.x;
  double omega_y = msg->angular_velocity.y;
  double omega_z = msg->angular_velocity.z;

  // Calculate the time derivative of a quaternion
  double dt = 0.01;
  std::array<double, 4> q_dot;
  std::array<double, 4> omega_q = {0.0, omega_x, omega_y, omega_z};
  multiplyQuaternion(q_, omega_q, q_dot);
  q_dot[0] *= 0.5;
  q_dot[1] *= 0.5;
  q_dot[2] *= 0.5;
  q_dot[3] *= 0.5;

  // Update quaternion
  q_[0] += q_dot[0] * dt;
  q_[1] += q_dot[1] * dt;
  q_[2] += q_dot[2] * dt;
  q_[3] += q_dot[3] * dt;

  // Normalize quaternion
  normalizeQuaternion(q_);

  // Publish message
  msg->orientation.w = q_[0];
  msg->orientation.x = q_[1];
  msg->orientation.y = q_[2];
  msg->orientation.z = q_[3];
  publishMsg(msg);
}

void ImuNode::multiplyQuaternion(const std::array<double, 4>& q1,
                                 const std::array<double, 4>& q2,
                                 std::array<double, 4>& q_out)
{
  q_out[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
  q_out[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
  q_out[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
  q_out[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

void ImuNode::normalizeQuaternion(std::array<double, 4>& q)
{
  double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;
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
