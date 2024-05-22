#include "simple_talker_string.hpp"


SimpleTalkerString::SimpleTalkerString() : Node("simple_talker_string")
{
    string_pub_ = this->create_publisher<std_msgs::msg::String>("/string_msg", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SimpleTalkerString::publish_string, this));
}

void SimpleTalkerString::publish_string()
{
    auto string_msg = std_msgs::msg::String();
    string_msg.data = "Hello world!";
    string_pub_->publish(string_msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTalkerString>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
