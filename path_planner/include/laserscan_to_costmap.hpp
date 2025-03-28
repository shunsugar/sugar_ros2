#ifndef LASER_SCAN_TO_COSTMAP_HPP
#define LASER_SCAN_TO_COSTMAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>
#include <vector>
#include <limits>

class LaserScanToCostmap : public rclcpp::Node
{
public:
    LaserScanToCostmap();

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    nav_msgs::msg::OccupancyGrid costmap_;

    double map_resolution_;
    int map_width_;
    int map_height_;
    double origin_x_;
    double origin_y_;
};

#endif // LASER_SCAN_TO_COSTMAP_HPP

