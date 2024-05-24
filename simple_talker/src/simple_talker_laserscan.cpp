#include "simple_talker_laserscan.hpp"


SimpleTalkerLaserScan::SimpleTalkerLaserScan() : Node("simple_talker_laserscan")
{
    laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/laserscan_msg", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), 
    std::bind(&SimpleTalkerLaserScan::publish_laserscan, this));
}

void SimpleTalkerLaserScan::publish_laserscan()
{
    auto laserscan_msg = sensor_msgs::msg::LaserScan();
    laserscan_msg.header.stamp.sec = 10;
    laserscan_msg.header.stamp.nanosec = 1;
    laserscan_msg.header.frame_id = "/laserscan_msg_frame";
    laserscan_msg.angle_min = 0.10;
    laserscan_msg.angle_max = 1.0;
    laserscan_msg.angle_increment = 0.10;
    laserscan_msg.time_increment = 0.10;
    laserscan_msg.scan_time = 1.0;
    laserscan_msg.range_min = 0.10;
    laserscan_msg.range_max = 1.0;
    laserscan_msg.ranges = {1.0, 2.0, 3.0, 4.0};
    laserscan_msg.intensities = {5.0, 6.0, 7.0, 8.0};
    laserscan_pub_->publish(laserscan_msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTalkerLaserScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
