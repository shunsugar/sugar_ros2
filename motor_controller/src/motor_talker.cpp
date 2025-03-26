#include "motor_talker.hpp"


MotorTalker::MotorTalker() : Node("motor_talker")
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MotorTalker::publish_cmd_vel, this));
}

void MotorTalker::publish_cmd_vel()
{
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    cmd_vel_msg.linear.x = 0.4;
    cmd_vel_msg.angular.z = 0.2;
    publisher_->publish(cmd_vel_msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTalker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
