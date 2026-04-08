from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='control',
            executable='gripper',
            name='gripper_node',
            output='screen'
        ),

        Node(
            package='control',
            executable='control',
            name='control_node',
            output='screen'
        ),

    ])