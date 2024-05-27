from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define a variable indicating the composition of the launch
    ld = LaunchDescription()

    # Define a node
    pub_node1 = Node(
        package="simple_talker",
        executable="simple_talker_twist",
        name="simple_talker_twist"
    )

    # Add a node to be activated
    ld.add_action(pub_node1)

    return ld
