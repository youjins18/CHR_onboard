from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("dynamixel_arm_driver"))
        / "config"
        / "arm_hardware.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="dynamixel_arm_driver",
                executable="dynamixel_driver_node",
                name="dynamixel_arm_driver",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
