#include "motor_controller.hpp"


MotorController::MotorController() : Node("motor_controller")
{
    ctrl_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/ctrl_cmd_vel", 10);
    ctrl_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
    ("/cmd_vel", 10, std::bind(&MotorController::control_callback, this, std::placeholders::_1));
}

void MotorController::control_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    auto ctrl_pub_msg = geometry_msgs::msg::Twist();
    RCLCPP_INFO(rclcpp::get_logger("motor_controller"), 
    "Received cmd_vel: linear.x=%.2f, angular.z=%.2f", msg->linear.x, msg->angular.z);
    ctrl_pub_msg.linear.x = msg->linear.x * 2.0;
    ctrl_pub_msg.angular.z = msg->angular.z / 2.0;
    ctrl_pub_->publish(ctrl_pub_msg);
    RCLCPP_INFO(rclcpp::get_logger("motor_controller"), 
    "Sent ctrl_cmd_vel: linear.x=%.2f, angular.z=%.2f", ctrl_pub_msg.linear.x, ctrl_pub_msg.angular.z);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
