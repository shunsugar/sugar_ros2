#include "obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector")
{
    is_upside_down_ = this->declare_parameter<bool>("is_upside_down", true);
    ground_height_threshold_ = this->declare_parameter<double>("ground_height_threshold", 0.32);
    lidar_height_ = this->declare_parameter<double>("lidar_height", 0.95);
    pcloud_in_ = this->declare_parameter<std::string>("pcloud_in", "/converted_pointcloud2");
    grd_pcloud_out_ = this->declare_parameter<std::string>("grd_pcloud_out", "/ground_points");
    obs_pcloud_out_ = this->declare_parameter<std::string>("obs_pcloud_out", "/obstacle_points");

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pcloud_in_, 10, std::bind(&ObstacleDetector::pointCloudCallback, this, std::placeholders::_1));
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(grd_pcloud_out_, 10);
    obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(obs_pcloud_out_, 10);

    RCLCPP_INFO(this->get_logger(), "Obstacle Detector Node Started.");
}

void ObstacleDetector::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (const auto &point : input_cloud->points)
    {
        if (is_upside_down_)
        {
            if (point.z > lidar_height_ - ground_height_threshold_)
            {
                ground_cloud->points.push_back(point);
            }
            else
            {
                obstacle_cloud->points.push_back(point);
            }
        }
        else
        {
            if (point.z < ground_height_threshold_ - lidar_height_)
            {
                ground_cloud->points.push_back(point);
            }
            else
            {
                obstacle_cloud->points.push_back(point);
            }
        }
    }

    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    ground_msg.header = msg->header;
    ground_pub_->publish(ground_msg);

    sensor_msgs::msg::PointCloud2 obstacle_msg;
    pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
    obstacle_msg.header = msg->header;
    obstacle_pub_->publish(obstacle_msg);

    //RCLCPP_INFO(this->get_logger(), "Processed point cloud: Ground (%zu points), Obstacles (%zu points)",
    //            ground_cloud->size(), obstacle_cloud->size());
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}

