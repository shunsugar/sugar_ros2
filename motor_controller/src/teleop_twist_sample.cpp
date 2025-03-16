#include "teleop_twist_sample.hpp"


TeleopTwistSample::TeleopTwistSample() : Node("teleop_twist_sample"), toggle_(true)
{
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), 
    std::bind(&TeleopTwistSample::publish_twist, this));
}

void TeleopTwistSample::publish_twist()
{
    auto twist_msg = geometry_msgs::msg::Twist();
    if (toggle_) {
        twist_msg.linear.x = 0.50;
        twist_msg.angular.z = 0.0;
    } else {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.39;
    }
    toggle_ = !toggle_;
    twist_pub_->publish(twist_msg);
    RCLCPP_INFO(this->get_logger(), "\nPublish: linear.x = %.2f, angular.z = %.2f", 
    twist_msg.linear.x, twist_msg.angular.z);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopTwistSample>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
