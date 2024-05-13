#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"


class MotorController : public rclcpp::Node
{
public:
    MotorController();

private:
    void control_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ctrl_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ctrl_sub_;
};

#endif
