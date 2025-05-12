#include "wheel_imu_odom.hpp"

OdomFusionNode::OdomFusionNode() : Node("wheel_imu_odom")
{
  imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
  fused_odom_topic_ = this->declare_parameter<std::string>("fused_odom_topic", "/odom/wheel_imu");
  
  odom_header_frame_ = this->declare_parameter<std::string>("odom_header_frame", "odom");
  odom_child_frame_ = this->declare_parameter<std::string>("odom_child_frame", "base_footprint");
  TF_header_frame_ = this->declare_parameter<std::string>("TF_header_frame", "odom");
  TF_child_frame_ = this->declare_parameter<std::string>("TF_child_frame", "base_footprint");
  
  scale_factor_ = this->declare_parameter<double>("scale_factor", 0.45);
  publish_odom_ = this->declare_parameter<bool>("publish_odom", true);
  publish_TF_ = this->declare_parameter<bool>("publish_TF", true);
  debug_ = this->declare_parameter<bool>("debug", false);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10, std::bind(&OdomFusionNode::imuCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&OdomFusionNode::odomCallback, this, std::placeholders::_1));
  fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(fused_odom_topic_, 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  
  yaw_ = 0.0;
  position_x_ = 0.0;
  position_y_ = 0.0;
  prev_time_ = this->now();
  
  RCLCPP_INFO(this->get_logger(), "Initialized wheel_imu_odom_node.");
  RCLCPP_INFO(this->get_logger(), "publish_odom: %s", publish_odom_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "publish_TF: %s", publish_TF_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "debug: %s", debug_ ? "true" : "false");
}

void OdomFusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  orientation_x_ = msg->orientation.x;
  orientation_y_ = msg->orientation.y;
  orientation_z_ = msg->orientation.z;
  orientation_w_ = msg->orientation.w;
  ang_vel_x_ = msg->angular_velocity.x;
  ang_vel_y_ = msg->angular_velocity.y;
  ang_vel_z_ = msg->angular_velocity.z;
  yaw_ = getYawFromQuaternion(msg->orientation);
}

void OdomFusionNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time current_time = msg->header.stamp;
  double dt = (current_time - prev_time_).seconds();
  prev_time_ = current_time;

  double dx = msg->twist.twist.linear.x * dt * scale_factor_;  // best: 0.45
  position_x_ += dx * cos(yaw_);
  position_y_ += dx * sin(yaw_);

  // Construct TF
  if (publish_TF_)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = TF_header_frame_;
    t.child_frame_id = TF_child_frame_;
    t.transform.translation.x = position_x_;
    t.transform.translation.y = position_y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = orientation_x_;
    t.transform.rotation.y = orientation_y_;
    t.transform.rotation.z = orientation_z_;
    t.transform.rotation.w = orientation_w_;
    tf_broadcaster_->sendTransform(t);
  }

  // Construct Odom message
  if (publish_odom_)
  {
    nav_msgs::msg::Odometry fused_msg;
    fused_msg.header.stamp = this->get_clock()->now();
    fused_msg.header.frame_id = odom_header_frame_;
    fused_msg.child_frame_id = odom_child_frame_;
    fused_msg.pose.pose.position.x = position_x_;
    fused_msg.pose.pose.position.y = position_y_;
    fused_msg.pose.pose.position.z = 0.0;
    fused_msg.pose.pose.orientation.x = orientation_x_;
    fused_msg.pose.pose.orientation.y = orientation_y_;
    fused_msg.pose.pose.orientation.z = orientation_z_;
    fused_msg.pose.pose.orientation.w = orientation_w_;
    fused_msg.twist.twist.linear.x = msg->twist.twist.linear.x;
    fused_msg.twist.twist.linear.y = 0.0;
    fused_msg.twist.twist.angular.x = ang_vel_x_;
    fused_msg.twist.twist.angular.y = ang_vel_y_;
    fused_msg.twist.twist.angular.z = ang_vel_z_;
    fused_odom_pub_->publish(fused_msg);
  }
  
  // Debugging Feature
  if (debug_)
  {
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, yaw: %f", position_x_, position_y_, yaw_);
  }
}

double OdomFusionNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFusionNode>());
  rclcpp::shutdown();
  return 0;
}

