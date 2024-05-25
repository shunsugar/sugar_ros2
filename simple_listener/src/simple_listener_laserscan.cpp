#include "simple_listener_laserscan.hpp"


SimpleListenerLaserScan::SimpleListenerLaserScan() : Node("simple_listener_laserscan")
{
    laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
    ("/laserscan_msg", 10, std::bind(&SimpleListenerLaserScan::laserscan_callback, this, std::placeholders::_1));
}

void SimpleListenerLaserScan::laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("simple_listener_laserscan"),
    "Subscribe :\nframe_id = %s, angle_increment = %.2f", msg->header.frame_id.c_str(), msg->angle_increment);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleListenerLaserScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
