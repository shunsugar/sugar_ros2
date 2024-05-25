#ifndef SIMPLE_LISTENER_LASERSCAN_H_
#define SIMPLE_LISTENER_LASERSCAN_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


class SimpleListenerLaserScan : public rclcpp::Node
{
public:
    SimpleListenerLaserScan();

private:
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
};

#endif
