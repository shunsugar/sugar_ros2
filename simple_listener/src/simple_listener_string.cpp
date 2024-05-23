#include "simple_listener_string.hpp"


SimpleListenerString::SimpleListenerString() : Node("simple_listener_string")
{
    string_sub_ = this->create_subscription<std_msgs::msg::String>
    ("/string_msg", 10, std::bind(&SimpleListenerString::string_callback, this, std::placeholders::_1));
}

void SimpleListenerString::string_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("simple_listener_string"),
    "Subscribe : %s", msg->data.c_str());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleListenerString>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
