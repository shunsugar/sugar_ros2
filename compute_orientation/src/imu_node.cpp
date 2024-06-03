#include "imu_node.hpp"

ImuNode::ImuNode() : Node("imu_node"), q_w_(1.0), q_x_(0.0), q_y_(0.0), q_z_(0.0)
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

  // Obtain angular velocity data
  double omega_x = msg->angular_velocity.x;
  double omega_y = msg->angular_velocity.y;
  double omega_z = msg->angular_velocity.z;

  // Calculate the time derivative of a quaternion
  double dt = 0.01;
  double q_dot_w, q_dot_x, q_dot_y, q_dot_z;
  multiplyQuaternion(q_w_, q_x_, q_y_, q_z_, 0, omega_x, omega_y, omega_z, q_dot_w, q_dot_x, q_dot_y, q_dot_z);
  q_dot_w *= 0.5;
  q_dot_x *= 0.5;
  q_dot_y *= 0.5;
  q_dot_z *= 0.5;

  // Update quaternion
  q_w_ += q_dot_w * dt;
  q_x_ += q_dot_x * dt;
  q_y_ += q_dot_y * dt;
  q_z_ += q_dot_z * dt;

  // Normalize quaternion
  normalizeQuaternion(q_w_, q_x_, q_y_, q_z_);

  // Publish message
  msg->orientation.w = q_w_;
  msg->orientation.x = q_x_;
  msg->orientation.y = q_y_;
  msg->orientation.z = q_z_;
  publishMsg(msg);
}

void ImuNode::multiplyQuaternion(double w1, double x1, double y1, double z1,
                                 double w2, double x2, double y2, double z2,
                                 double &w_out, double &x_out, double &y_out, double &z_out)
{
  w_out = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  x_out = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
  y_out = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
  z_out = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

void ImuNode::normalizeQuaternion(double &w, double &x, double &y, double &z)
{
  double norm = std::sqrt(w * w + x * x + y * y + z * z);
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
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
