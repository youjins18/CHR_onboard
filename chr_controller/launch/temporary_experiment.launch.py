from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    controller_config = str(
        Path(get_package_share_directory("chr_controller"))
        / "config"
        / "chr_controller.yaml"
    )
    bag_output_root = str(
        Path.home() / "Desktop" / "chr_onboard_ws" / "src" / "data_logging" / "bags"
    )
    bag_name = datetime.now().strftime("chr_rosbag2_%Y_%m_%d-%H_%M_%S")
    bag_topics = [
        "/chr/data_logging_msg",
        "/cone/joint_states",
        "/cone/joint_commands",
        "/chr/state/tcp_pose",
        "/chr/target/tcp_pose",
        "/chr/controller_state",
        "/cone/torque_enabled",
        "/control_mode_flags",
        "/fmu/in/custom_command_position_mode",
        "/fmu/in/custom_command_velocity_mode",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("controller_config", default_value=controller_config),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument("bag_output_root", default_value=bag_output_root),
            DeclareLaunchArgument("bag_name", default_value=bag_name),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "--storage",
                    "sqlite3",
                    "--output",
                    PathJoinSubstitution(
                        [LaunchConfiguration("bag_output_root"), LaunchConfiguration("bag_name")]
                    ),
                    *bag_topics,
                ],
                output="screen",
                sigterm_timeout="15",
                sigkill_timeout="5",
                condition=IfCondition(LaunchConfiguration("record_bag")),
            ),
            Node(
                package="chr_controller",
                executable="chr_controller_node",
                name="chr_controller",
                output="screen",
                parameters=[LaunchConfiguration("controller_config")],
            ),
        ]
    )
