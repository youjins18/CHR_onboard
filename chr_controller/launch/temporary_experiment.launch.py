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
    arm_config = str(
        Path(get_package_share_directory("dynamixel_arm_driver"))
        / "config"
        / "arm_hardware.yaml"
    )
    bag_output_root = str(
        Path.home() / "Desktop" / "chr_onboard_ws" / "src" / "data_logging" / "bags"
    )
    bag_name = datetime.now().strftime("rosbag2_%Y_%m_%d-%H_%M_%S")
    bag_topics = [
        "/data_logging_msg",
        "/chr/data_logging_msg",
        "/fmu/out/vehicle_odometry",
        "/fmu/out/vehicle_status",
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
            DeclareLaunchArgument("arm_config", default_value=arm_config),
            DeclareLaunchArgument("launch_flight_bridge", default_value="true"),
            DeclareLaunchArgument("launch_tilt_controller", default_value="true"),
            DeclareLaunchArgument("launch_arm_driver", default_value="true"),
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
                package="px4_ros_com",
                executable="listen_and_speak_ros",
                name="listen_and_speak_ros",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_flight_bridge")),
            ),
            Node(
                package="quadrotor_controller",
                executable="dynamixel_fix",
                name="quadrotor_tilt_controller",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_tilt_controller")),
            ),
            Node(
                package="dynamixel_arm_driver",
                executable="dynamixel_driver_node",
                name="dynamixel_arm_driver",
                output="screen",
                parameters=[LaunchConfiguration("arm_config")],
                condition=IfCondition(LaunchConfiguration("launch_arm_driver")),
            ),
            Node(
                package="chr_controller",
                executable="chr_controller_node",
                name="chr_controller",
                output="screen",
                parameters=[LaunchConfiguration("controller_config")],
            ),
            # Deliberately do not launch px4_ros_com/custom_trajectory_generation:
            # chr_controller must be the only custom position/velocity publisher.
        ]
    )
