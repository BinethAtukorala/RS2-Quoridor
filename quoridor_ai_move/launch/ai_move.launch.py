from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_model = str(Path.home() / "quoridor_models" / "latest")
    return LaunchDescription([
        DeclareLaunchArgument("model_dir", default_value=default_model),
        DeclareLaunchArgument("online_learning", default_value="true"),
        DeclareLaunchArgument("save_after_game", default_value="true"),
        DeclareLaunchArgument("epsilon", default_value="0.0"),
        DeclareLaunchArgument("side", default_value="bot"),
        Node(
            package="quoridor_ai_move",
            executable="ai_move_node",
            name="ai_move_node",
            output="screen",
            parameters=[{
                "model_dir": LaunchConfiguration("model_dir"),
                "online_learning": LaunchConfiguration("online_learning"),
                "save_after_game": LaunchConfiguration("save_after_game"),
                "epsilon": LaunchConfiguration("epsilon"),
                "side": LaunchConfiguration("side"),
            }],
        ),
    ])
