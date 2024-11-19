from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(package="mobile_control", executable="control"))
    ld.add_action(Node(package="mobile_control", executable="command"))

    return ld