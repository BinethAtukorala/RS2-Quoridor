"""Launch a game against the trained DQN engine.

Starts state_manager, web_interface, and the ai_move inference node. You play
as the human (player) through the web UI; the AI plays as bot.

Usage:
    ros2 launch quoridor_ai_move play_vs_ai.launch.py model_dir:=$HOME/quoridor_models/v_mm
    ros2 launch quoridor_ai_move play_vs_ai.launch.py model_dir:=$HOME/quoridor_models/v_mm online_learning:=true
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = str(Path.home() / "quoridor_models" / "latest")
    return LaunchDescription([
        DeclareLaunchArgument("model_dir",       default_value=default_model),
        DeclareLaunchArgument("epsilon",         default_value="0.0"),
        DeclareLaunchArgument("side",            default_value="bot"),
        DeclareLaunchArgument("online_learning", default_value="false"),
        DeclareLaunchArgument("save_after_game", default_value="false"),
        DeclareLaunchArgument("web_port",        default_value="8088"),

        Node(package="quoridor_game",    executable="state_manager",
             name="state_manager",      output="screen"),

        Node(package="quoridor_game",    executable="web_interface",
             name="web_interface",      output="screen",
             parameters=[{"port": LaunchConfiguration("web_port")}]),

        Node(package="quoridor_ai_move", executable="ai_move_node",
             name="ai_move_node",       output="screen",
             parameters=[{
                 "model_dir":       LaunchConfiguration("model_dir"),
                 "epsilon":         LaunchConfiguration("epsilon"),
                 "side":            LaunchConfiguration("side"),
                 "online_learning": LaunchConfiguration("online_learning"),
                 "save_after_game": LaunchConfiguration("save_after_game"),
             }]),
    ])
