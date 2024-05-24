#ifndef SIMPLE_TALKER_POINTCLOUD2_H_
#define SIMPLE_TALKER_POINTCLOUD2_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


class SimpleTalkerPointCloud2 : public rclcpp::Node
{
public:
    SimpleTalkerPointCloud2();

private:
    void publish_pointcloud2();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
