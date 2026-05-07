"""Launch a game against the trained AlphaZero engine.

Starts state_manager, web_interface, and the AZ inference node. You play
through the web UI; the AZ network + MCTS plays as bot.

Usage:
    ros2 launch quoridor_alphazero play_vs_az.launch.py model_dir:=$HOME/quoridor_az_models/latest
    ros2 launch quoridor_alphazero play_vs_az.launch.py simulations:=400
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = str(Path.home() / "quoridor_az_models" / "latest")
    return LaunchDescription([
        DeclareLaunchArgument("model_dir",   default_value=default_model),
        DeclareLaunchArgument("side",        default_value="bot"),
        DeclareLaunchArgument("simulations", default_value="200"),
        DeclareLaunchArgument("temperature", default_value="0.0"),
        DeclareLaunchArgument("web_port",    default_value="8088"),

        Node(package="quoridor_game",       executable="state_manager",
             name="state_manager",          output="screen"),

        Node(package="quoridor_game",       executable="web_interface",
             name="web_interface",          output="screen",
             parameters=[{"port": LaunchConfiguration("web_port")}]),

        Node(package="quoridor_alphazero", executable="ai_move_node",
             name="az_move_node",           output="screen",
             prefix=["/rs2_ws/src/quoridor_alphazero/env/bin/python3"],
             parameters=[{
                 "model_dir":   LaunchConfiguration("model_dir"),
                 "side":        LaunchConfiguration("side"),
                 "simulations": LaunchConfiguration("simulations"),
                 "temperature": LaunchConfiguration("temperature"),
             }]),
    ])
