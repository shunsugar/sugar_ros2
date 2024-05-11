#ifndef MOTOR_TALKER_H_
#define MOTOR_TALKER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"


class MotorTalker : public rclcpp::Node
{
public:
    MotorTalker();

private:
    void publish_cmd_vel();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
