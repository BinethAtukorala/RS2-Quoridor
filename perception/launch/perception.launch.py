from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="perception",      executable="camera_node",
             name="camera_node",        output="screen"),
        
        Node(package="perception",      executable="perception_node",
             name="perception_node",        output="screen"),
    ])