#ifndef SIMPLE_LISTENER_POINTCLOUD2_H_
#define SIMPLE_LISTENER_POINTCLOUD2_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


class SimpleListenerPointCloud2 : public rclcpp::Node
{
public:
    SimpleListenerPointCloud2();

private:
    void pointcloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub_;
};

#endif
