#include "simple_listener_pointcloud2.hpp"


SimpleListenerPointCloud2::SimpleListenerPointCloud2() : Node("simple_listener_pointcloud2")
{
    pointcloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>
    ("/pointcloud2_msg", 10, std::bind(&SimpleListenerPointCloud2::pointcloud2_callback, this, std::placeholders::_1));
}

void SimpleListenerPointCloud2::pointcloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("simple_listener_pointcloud2"),
    "Subscribe :\nframe_id = %s, point_step = %u", msg->header.frame_id.c_str(), msg->point_step);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleListenerPointCloud2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
