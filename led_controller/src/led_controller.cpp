#include "led_controller.hpp"

using namespace std::chrono_literals;

LedController::LedController()
    : Node("LED_controller"), nav_state_(false), estop_state_(false)
{
    try
    {
        serial_port_.setPort("/dev/sensors/LED");
        serial_port_.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_port_.setTimeout(to);
        serial_port_.open();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    }

    nav_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/nav_state", 10, std::bind(&LedController::navCallback, this, std::placeholders::_1));
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/estop_state", 10, std::bind(&LedController::estopCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(200ms, std::bind(&LedController::sendSerialCommand, this));
}

LedController::~LedController()
{
    if (serial_port_.isOpen())
    {
        serial_port_.close();
    }
}

void LedController::navCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    nav_state_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Received /nav_state: ");
}

void LedController::estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    estop_state_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Received /estop_state: ");
}

void LedController::sendSerialCommand()
{
    auto msg = std_msgs::msg::Bool();

    if (nav_state_ && !estop_state_)
    {
        led_state_ = "ON";
    }
    else if (nav_state_ && estop_state_)
    {
        led_state_ = "OFF";
    }
    else if (!nav_state_ && !estop_state_)
    {
        led_state_ = "OFF";
    }
    else if (!nav_state_ && estop_state_)
    {
       led_state_ = "OFF";
    }
    
    if (serial_port_.isOpen())
    {
        serial_port_.write(led_state_ + "\n");
        // RCLCPP_INFO(this->get_logger(), "Sent: %s", led_state_.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port is NOT open!");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedController>());
    rclcpp::shutdown();
    return 0;
}

