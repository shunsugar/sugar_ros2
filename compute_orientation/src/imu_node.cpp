#include "imu_node.hpp"

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

  double dt = 0.01;

  // Obtain angular_velocity data
  double omega_x = msg->angular_velocity.x;
  double omega_y = msg->angular_velocity.y;
  double omega_z = msg->angular_velocity.z;
  std::array<double, 4> omega_q = {0.0, omega_x, omega_y, omega_z};

  // Calculate the time derivative of a quaternion
  std::array<double, 4> q_dot;
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

  // Obtain linear_acceleration data
  double alpha_x = msg->linear_acceleration.x;
  double alpha_y = msg->linear_acceleration.y;
  double alpha_z = msg->linear_acceleration.z;
  std::array<double, 4> alpha_q = {0.0, alpha_x, alpha_y, alpha_z};

  // Rotate linear acceleration vector
  rotateAccelerationVector(q_, alpha_q);
	Vector3 alpha_v = {alpha_q[1], alpha_q[2], alpha_q[3]};

  // Calculate unit vector and rotation angle
	Vector3 g = {0, 0, 1};
	Vector3 n;
	double theta;
  calculateRotationParams(alpha_v, g, n, theta);

	// Calculate quaternion to compensate for errors
  cal(n, theta, q_alpha);

  // Compensate for quaternion
  multiplyQuaternion(q_alpha, q_, q_);

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

void ImuNode::rotateAccelerationVector(const std::array<double, 4>& q, std::array<double, 4> alpha)
{
  // q_inverse = q_bar / |q|^2
  std::array<double, 4> q_inverse = {q[0], -q[1], -q[2], -q[3]};
  double magnitude_q = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  q_inverse[0] /= magnitude_q;
  q_inverse[1] /= magnitude_q;
  q_inverse[2] /= magnitude_q;
  q_inverse[3] /= magnitude_q;

  // alpha = q x alpha x q_inverse
  multiplyQuaternion(q, alpha, alpha);
	multiplyQuaternion(alpha, q_inverse, alpha);
}

void ImuNode::calculateRotationParams(const alpha, const g, Vector3 n, double theta)
{
  // alpha x g
  crossProduct(alpha, g, n);
  double magnitude_v = [0]
  n[0] /= magnitude_v;
}

void ImuNode::crossProduct()
{
  
}

void ImuNode::compensateQuaternion(const std::array<double, 4>& q, std::array<double, 4> alpha)
{
  Vector3 a = accel;
  double aNorm = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  for (int i = 0; i < 3; i++) {
    a[i] /= aNorm;
  }
  Vector3 g = {0, 0, 1};
  Vector3 aError = {a[1]*g[2] - a[2]*g[1], a[2]*g[0] - a[0]*g[2], a[0]*g[1] - a[1]*g[0]};
  double aErrorNorm = std::sqrt(aError[0]*aError[0] + aError[1]*aError[1] + aError[2]*aError[2]);
  Quaternion q_a = {std::cos(alpha * aErrorNorm / 2), std::sin(alpha * aErrorNorm / 2) * aError[0] / aErrorNorm, std::sin(alpha * aErrorNorm / 2) * aError[1] / aErrorNorm, std::sin(alpha * aErrorNorm / 2) * aError[2] / aErrorNorm};
  return quaternionMultiply(q_a, q);
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
