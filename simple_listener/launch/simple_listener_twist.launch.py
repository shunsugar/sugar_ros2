from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define a variable indicating the composition of the launch
    ld = LaunchDescription()

    # Define nodes
    pub_node1 = Node(
        package="simple_talker",
        executable="simple_talker_twist",
        name="simple_talker_twist"
    )

    pub_node2 = Node(
        package="simple_listener",
        executable="simple_listener_twist",
        name="simple_listener_twist"
    )

    # Add nodes to be activated
    ld.add_action(pub_node1)
    ld.add_action(pub_node2)

    return ld
