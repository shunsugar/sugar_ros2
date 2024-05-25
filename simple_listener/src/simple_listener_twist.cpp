#include "simple_listener_twist.hpp"


SimpleListenerTwist::SimpleListenerTwist() : Node("simple_listener_twist")
{
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
    ("/twist_msg", 10, std::bind(&SimpleListenerTwist::twist_callback, this, std::placeholders::_1));
}

void SimpleListenerTwist::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("simple_listener_twist"),
    "Subscribe :\nlinear.x = %.2f, angular.z = %.2f", msg->linear.x, msg->angular.z);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleListenerTwist>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
