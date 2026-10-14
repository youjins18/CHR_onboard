import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _require_package(package_name, purpose):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"Required ROS 2 package '{package_name}' was not found. "
            f'It is needed to {purpose}. Build the workspace and source '
            'install/setup.bash before launching.'
        ) from exc


def generate_launch_description():
    px4_ros_com_share = _require_package(
        'px4_ros_com',
        'start the existing v3 flight launcher',
    )
    mocap_bridge_share = _require_package(
        'px4_mocap_bridge',
        'start the OptiTrack-to-PX4 bridge',
    )

    hostname = LaunchConfiguration('hostname')
    rigid_body_name = LaunchConfiguration('rigid_body_name')
    bag_output_root = LaunchConfiguration('bag_output_root')
    bag_name = LaunchConfiguration('bag_name')
    record_bag = LaunchConfiguration('record_bag')
    default_bag_root = str(
        Path.home() / 'Desktop' / 'chr_onboard_ws' / 'src' / 'data_logging' / 'bags'
    )
    default_bag_name = datetime.now().strftime(
        'palletrone_rosbag2_%Y_%m_%d-%H_%M_%S'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'hostname',
            default_value='192.168.0.100',
            description='Hostname or IP address of the Motive computer.',
        ),
        DeclareLaunchArgument(
            'rigid_body_name',
            default_value='drone',
            description='Motive rigid body name to forward to PX4.',
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description='Record the Palletrone flight and OptiTrack topics.',
        ),
        DeclareLaunchArgument(
            'bag_output_root',
            default_value=default_bag_root,
            description='Directory that receives the timestamped Palletrone bag.',
        ),
        DeclareLaunchArgument(
            'bag_name',
            default_value=default_bag_name,
            description='Palletrone bag directory name.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    px4_ros_com_share,
                    'launch',
                    'v3_flight_launcher.launch.py',
                )
            ),
            launch_arguments={
                'launch_custom_trajectory': 'false',
                'record_bag': record_bag,
                'bag_output_root': bag_output_root,
                'bag_name': bag_name,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    mocap_bridge_share,
                    'launch',
                    'optitrack_px4.launch.py',
                )
            ),
            launch_arguments={
                'hostname': hostname,
                'rigid_body_name': rigid_body_name,
            }.items(),
        ),
    ])
