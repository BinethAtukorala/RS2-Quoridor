"""Launch the full ROS-driven training stack:

  state_manager   - owns the QuoridorBoard, requests bot moves, broadcasts state
  web_interface   - browser GUI at http://localhost:8088
  minimax_bot     - answers /quoridor/compute_move_request with minimax (teacher)
  student_trainer - plays as "player" with a DQN, learns online (student)

The student node also drives game lifecycle: it sends the initial "start"
command and a fresh "start" after every terminal state. Open the web UI to
watch training in real time.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = str(Path.home() / "quoridor_models" / "v_ros")
    return LaunchDescription([
        DeclareLaunchArgument("model_dir", default_value=default_model),
        DeclareLaunchArgument("minimax_depth", default_value="3"),
        DeclareLaunchArgument("web_port", default_value="8088"),
        DeclareLaunchArgument("resume", default_value="true"),
        DeclareLaunchArgument("max_episodes", default_value="0"),

        Node(package="quoridor_game", executable="state_manager",
             name="state_manager", output="screen"),

        Node(package="quoridor_game", executable="web_interface",
             name="web_interface", output="screen",
             parameters=[{"port": LaunchConfiguration("web_port")}]),

        Node(package="quoridor_ai_move", executable="minimax_bot_node",
             name="minimax_bot_node", output="screen",
             parameters=[{"search_depth": LaunchConfiguration("minimax_depth")}]),

        Node(package="quoridor_ai_move", executable="student_trainer_node",
             name="student_trainer_node", output="screen",
             parameters=[{
                 "model_dir": LaunchConfiguration("model_dir"),
                 "resume": LaunchConfiguration("resume"),
                 "max_episodes": LaunchConfiguration("max_episodes"),
             }]),
    ])
