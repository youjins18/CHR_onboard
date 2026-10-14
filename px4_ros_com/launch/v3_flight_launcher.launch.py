from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_bag_root = str(Path.cwd())
    default_bag_name = datetime.now().strftime("palletrone_rosbag2_%Y_%m_%d-%H_%M_%S")
    flight_bag_topics = [
        "/data_logging_msg",
        "/fmu/out/vehicle_odometry",
        "/fmu/out/vehicle_local_position",
        "/fmu/out/vehicle_status",
        "/poses",
        "/fmu/in/vehicle_visual_odometry",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_custom_trajectory",
                default_value="false",
                description=(
                    "Run the legacy Palletrone trajectory publisher. Keep this false "
                    "for CHR so chr_controller is the only custom-command publisher."
                ),
            ),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument("bag_output_root", default_value=default_bag_root),
            DeclareLaunchArgument("bag_name", default_value=default_bag_name),
            Node(
                package="px4_ros_com",
                executable="listen_and_speak_ros",
                name="listen_and_speak_ros",
                output="screen",
            ),
            Node(
                package="px4_ros_com",
                executable="custom_trajectory_generation",
                name="custom_trajectory_generation",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_custom_trajectory")),
            ),
            TimerAction(
                period=3.0,
                condition=IfCondition(LaunchConfiguration("record_bag")),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "bag",
                            "record",
                            "--storage",
                            "sqlite3",
                            "--output",
                            PathJoinSubstitution(
                                [
                                    LaunchConfiguration("bag_output_root"),
                                    LaunchConfiguration("bag_name"),
                                ]
                            ),
                            *flight_bag_topics,
                        ],
                        output="screen",
                        sigterm_timeout="15",
                        sigkill_timeout="5",
                    )
                ],
            ),
        ]
    )
