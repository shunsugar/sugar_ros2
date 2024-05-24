#include "simple_talker_twist.hpp"


SimpleTalkerTwist::SimpleTalkerTwist() : Node("simple_talker_twist")
{
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/twist_msg", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), 
    std::bind(&SimpleTalkerTwist::publish_twist, this));
}

void SimpleTalkerTwist::publish_twist()
{
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.4;
    twist_msg.angular.z = 0.2;
    twist_pub_->publish(twist_msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTalkerTwist>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
