#ifndef SIMPLE_LISTENER_TWIST_H_
#define SIMPLE_LISTENER_TWIST_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class SimpleListenerTwist : public rclcpp::Node
{
public:
    SimpleListenerTwist();

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

#endif
