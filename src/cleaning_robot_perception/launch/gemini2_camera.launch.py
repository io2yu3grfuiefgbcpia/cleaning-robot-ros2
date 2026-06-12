#!/usr/bin/env python3
"""Launch the direct Gemini 2 camera publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create launch description."""
    package_share = get_package_share_directory("cleaning_robot_perception")
    default_config = os.path.join(
        package_share,
        "config",
        "gemini2_camera.yaml",
    )

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the Gemini 2 camera YAML config.",
            ),
            Node(
                package="cleaning_robot_perception",
                executable="gemini2_camera_node",
                name="gemini2_camera_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
