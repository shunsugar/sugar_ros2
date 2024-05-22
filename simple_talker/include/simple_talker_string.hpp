#ifndef SIMPLE_TALKER_STRING_H_
#define SIMPLE_TALKER_STRING_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class SimpleTalkerString : public rclcpp::Node
{
public:
    SimpleTalkerString();

private:
    void publish_string();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
