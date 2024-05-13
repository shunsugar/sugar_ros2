#include "motor_listener.hpp"


MotorListener::MotorListener() : Node("motor_listener")
{
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>
    ("/ctrl_cmd_vel", 10, std::bind(&MotorListener::cmd_vel_callback, this, std::placeholders::_1));
}

void MotorListener::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("motor_listener"), 
    "Received cmd_vel: linear.x=%.2f, angular.z=%.2f", msg->linear.x, msg->angular.z);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
