from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            output='screen',
            parameters=['/home/ubuntu/ros2_ws/src/sugar_ros2/sensor_utils/config/costmap_params.yaml']
        )
    ])
