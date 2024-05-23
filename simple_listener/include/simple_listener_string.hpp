#ifndef SIMPLE_LISTENER_STRING_H_
#define SIMPLE_LISTENER_STRING_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class SimpleListenerString : public rclcpp::Node
{
public:
    SimpleListenerString();

private:
    void string_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
};

#endif
