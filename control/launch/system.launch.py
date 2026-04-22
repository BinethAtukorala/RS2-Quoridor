from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare('ur_onrobot_control'), '/launch/start_robot.launch.py'
        #     ]),
        #     launch_arguments={
        #         'ur_type':           'ur3e',
        #         'onrobot_type':      'rg2',
        #         'robot_ip': '192.168.0.194',
        #     }.items()
        # ),

        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('ur_onrobot_moveit_config'), '/launch/ur_onrobot_moveit.launch.py'
                ]),
                launch_arguments={
                    'ur_type':      'ur3e',
                    'onrobot_type': 'rg2',
                }.items()
            ),
        ]),

        TimerAction(period=15.0, actions=[
            Node(
                package='perception',
                executable='camera_node',
                name='camera_node',
                output='screen'
            ),
        ]),
        TimerAction(period=17.0, actions=[
            Node(
                package='perception',
                executable='grid_detector_node',
                name='grid_detector_node',
                output='screen'
            ),
        ]),
        TimerAction(period=19.0, actions=[
            Node(
                package='perception',
                executable='circle_detector_node',
                name='circle_detector_node',
                output='screen'
            ),
        ]),

        TimerAction(period=21.0, actions=[
            Node(
                package='quoridor_game',
                executable='move_decision',
                name='move_decision',
                output='screen'
            ),
        ]),
        TimerAction(period=23.0, actions=[
            Node(
                package='quoridor_game',
                executable='state_manager',
                name='state_manager',
                output='screen'
            ),
        ]),
        TimerAction(period=25.0, actions=[
            Node(
                package='quoridor_game',
                executable='user_interface',
                name='user_interface',
                output='screen'
            ),
        ]),

        TimerAction(period=27.0, actions=[
            Node(
                package='control',
                executable='control',
                name='control_node',
                output='screen'
            ),
        ]),
        TimerAction(period=29.0, actions=[
            Node(
                package='control',
                executable='poses',
                name='poses_node',
                output='screen'
            ),
        ]),

    ])