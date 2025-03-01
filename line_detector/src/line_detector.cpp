#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <sensor_msgs/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LineDetector : public rclcpp::Node {
public:
    LineDetector() : Node("line_detector") {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/converted_pointcloud2", rclcpp::SensorDataQoS(),
            std::bind(&LineDetector::lidarCallback, this, std::placeholders::_1)
        );
        
        white_line_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/white_line", 10
        );
        
        RCLCPP_INFO(this->get_logger(), "Line Detection Node Started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr white_line_pub_;
    
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        
        sensor_msgs::msg::PointCloud2 output;
        output.header = msg->header;
        output.height = 1;
        output.width = 0;  //
        output.is_dense = false;
        output.is_bigendian = msg->is_bigendian;
        output.fields = msg->fields;
        output.point_step = msg->point_step;
        output.row_step = msg->row_step;
        output.data.reserve(msg->data.size());  //

        //
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            if (*iter_intensity > 50.0) {  //
                //
                output.data.insert(output.data.end(),
                                   reinterpret_cast<const uint8_t*>(&(*iter_x)),
                                   reinterpret_cast<const uint8_t*>(&(*iter_x)) + msg->point_step);
                output.width++;
            }
        }

        //
        if (output.width > 0) {
            white_line_pub_->publish(output);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetector>());
    rclcpp::shutdown();
    return 0;
}

