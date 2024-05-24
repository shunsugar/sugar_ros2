#ifndef SIMPLE_TALKER_LASERSCAN_H_
#define SIMPLE_TALKER_LASERSCAN_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


class SimpleTalkerLaserScan : public rclcpp::Node
{
public:
    SimpleTalkerLaserScan();

private:
    void publish_laserscan();

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
