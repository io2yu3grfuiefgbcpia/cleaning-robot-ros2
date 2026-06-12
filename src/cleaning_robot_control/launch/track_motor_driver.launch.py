#!/usr/bin/env python3
"""Launch the tracked motor hardware driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create launch description."""
    package_share = get_package_share_directory("cleaning_robot_control")
    default_config = os.path.join(
        package_share,
        "config",
        "track_motor_driver.yaml",
    )

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the track motor driver YAML config.",
            ),
            Node(
                package="cleaning_robot_control",
                executable="track_motor_driver",
                name="track_motor_driver",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
