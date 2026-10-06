from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("keyboard_teleop_chr"))
        / "config"
        / "teleop.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="keyboard_teleop_chr",
                executable="keyboard_teleop",
                name="keyboard_teleop_chr",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            )
        ]
    )
