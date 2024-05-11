#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"


class MotorTalker : public rclcpp::Node
{
public:
    MotorTalker() : Node("motor_talker")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MotorTalker::publish_cmd_vel, this));
    }

    void publish_cmd_vel()
    {
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = 0.4;
        cmd_vel_msg.angular.z = 0.2;
        publisher_->publish(cmd_vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTalker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
