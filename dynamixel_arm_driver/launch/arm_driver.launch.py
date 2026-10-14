from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("dynamixel_arm_driver"))
        / "config"
        / "arm_hardware.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument(
                "torque_on_at_startup",
                default_value="true",
                description=(
                    "Capture the current hanging pose as q=[0,0,0,0] and enable "
                    "arm torque as soon as this launch starts."
                ),
            ),
            DeclareLaunchArgument(
                "capture_zero_on_torque_enable",
                default_value="true",
                description="Use the measured hanging pose as the arm zero.",
            ),
            DeclareLaunchArgument(
                "disable_torque_on_shutdown",
                default_value="true",
                description="Attempt arm torque-off during a normal Ctrl-C shutdown.",
            ),
            Node(
                package="dynamixel_arm_driver",
                executable="dynamixel_driver_node",
                name="dynamixel_arm_driver",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "torque_on_at_startup": ParameterValue(
                            LaunchConfiguration("torque_on_at_startup"),
                            value_type=bool,
                        ),
                        "capture_zero_on_torque_enable": ParameterValue(
                            LaunchConfiguration("capture_zero_on_torque_enable"),
                            value_type=bool,
                        ),
                        "disable_torque_on_shutdown": ParameterValue(
                            LaunchConfiguration("disable_torque_on_shutdown"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
