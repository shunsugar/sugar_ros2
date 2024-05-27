#ifndef TELEOP_TWIST_SAMPLE_H_
#define TELEOP_TWIST_SAMPLE_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>


using namespace std::chrono_literals;

class TeleopTwistSample : public rclcpp::Node
{
public:
    TeleopTwistSample();

private:
    void publish_twist();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool toggle_;
};

#endif
