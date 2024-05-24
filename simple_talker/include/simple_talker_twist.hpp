#ifndef SIMPLE_TALKER_TWIST_H_
#define SIMPLE_TALKER_TWIST_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class SimpleTalkerTwist : public rclcpp::Node
{
public:
    SimpleTalkerTwist();

private:
    void publish_twist();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
