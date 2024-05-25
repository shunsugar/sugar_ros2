#include "simple_talker_pointcloud2.hpp"


SimpleTalkerPointCloud2::SimpleTalkerPointCloud2() : Node("simple_talker_pointcloud2")
{
    pointcloud2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud2_msg", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), 
    std::bind(&SimpleTalkerPointCloud2::publish_pointcloud2, this));
}

void SimpleTalkerPointCloud2::publish_pointcloud2()
{
    auto pointcloud2_msg = sensor_msgs::msg::PointCloud2();
    pointcloud2_msg.header.frame_id = "/pointcloud2_msg_frame";
    pointcloud2_msg.point_step = 10;
    pointcloud2_pub_->publish(pointcloud2_msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTalkerPointCloud2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
