#ifndef MOTOR_LISTENER_H_
#define MOTOR_LISTENER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"


class MotorListener : public rclcpp::Node
{
public:
    MotorListener();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

#endif
