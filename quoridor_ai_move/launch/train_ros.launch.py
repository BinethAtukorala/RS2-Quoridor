"""Launch the full ROS-driven training stack.

Starts state_manager, move_decision (minimax teacher), web_interface, and
the train_ros DQN trainer. The trainer plays as "player" against the minimax
bot and drives the game lifecycle automatically.

Open http://localhost:8088 and set the mode to "manual" before training starts.

Usage:
    ros2 launch quoridor_ai_move train_ros.launch.py
    ros2 launch quoridor_ai_move train_ros.launch.py model_dir:=$HOME/quoridor_models/v_ros resume:=true
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = str(Path.home() / "quoridor_models" / "v_ros")
    default_tb = str(Path.home() / "quoridor_tensorboard" / "v_ros")
    return LaunchDescription([
        DeclareLaunchArgument("model_dir",       default_value=default_model),
        DeclareLaunchArgument("tb_log_dir",      default_value=default_tb),
        DeclareLaunchArgument("shaping_coef",    default_value="0.1"),
        DeclareLaunchArgument("resume",          default_value="false"),
        DeclareLaunchArgument("max_episodes",    default_value="0"),
        DeclareLaunchArgument("web_port",        default_value="8088"),

        Node(package="quoridor_game",    executable="state_manager",
             name="state_manager",      output="screen"),

        Node(package="quoridor_game",    executable="web_interface",
             name="web_interface",      output="screen",
             parameters=[{"port": LaunchConfiguration("web_port")}]),

        Node(package="quoridor_move_decision", executable="move_decision",
             name="move_decision",      output="screen"),

        Node(package="quoridor_ai_move", executable="train_ros",
             name="train_ros",          output="screen",
             parameters=[{
                 "model_dir":    LaunchConfiguration("model_dir"),
                 "tb_log_dir":   LaunchConfiguration("tb_log_dir"),
                 "shaping_coef": LaunchConfiguration("shaping_coef"),
                 "resume":       LaunchConfiguration("resume"),
                 "max_episodes": LaunchConfiguration("max_episodes"),
             }]),
    ])
