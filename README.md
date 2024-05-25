# sugar_cat2
This repository is for learning the basics, including ROS2 nodes and topics.


## Setup⚙️
The following command will copy the sugar_cat2 directory to your workspace.
```
cd ~/ros2_ws/src
git clone https://github.com/shunsugar/sugar_cat2.git
cd ~/ros2_ws
colcon build
source ~/.bashrc
```


## Publish various messages😏
The following 4 message types can be run.

[std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html), 
[geometry_msgs/Twist](https://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html), 
[sensor_msgs/LaserScan](https://docs.ros.org/en/groovy/api/sensor_msgs/html/msg/LaserScan.html), and
[sensor_msgs/PointCloud2](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/PointCloud2.html).
```
ros2 run simple_talker simple_talker_{MESSAGE_TYPES}
```
MESSAGE_TYPES can be selected from string, twist, laserscan, and pointcloud2.


## Subscribe various messages😋
The following 4 message types can be run.

[std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html), 
[geometry_msgs/Twist](https://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html), 
[sensor_msgs/LaserScan](https://docs.ros.org/en/groovy/api/sensor_msgs/html/msg/LaserScan.html), and
[sensor_msgs/PointCloud2](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/PointCloud2.html).
```
ros2 run simple_listener simple_listener_{MESSAGE_TYPES}
```
MESSAGE_TYPES can be selected from string, twist, laserscan, and pointcloud2.

>[!TIP]
>Let's run ```rqt_graph``` !
>
>The following is an example of running the ```simple_talker_twist``` and ```simple_listener_twist``` nodes. You can see that the topic ```/twist_msg``` is published and subscribed.
![simple_nodes](https://github.com/shunsugar/sugar_cat2/assets/120554165/af732472-0a2c-42a8-98e0-881c07acef55)
