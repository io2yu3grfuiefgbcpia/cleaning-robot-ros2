#!/usr/bin/env python3
"""Bring up N10P Ethernet LiDAR with Nav2 for the cleaning robot."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Create the Nav2 launch description for the N10P Ethernet setup."""
    pkg_navigation = get_package_share_directory("cleaning_robot_navigation")
    pkg_description = get_package_share_directory("cleaning_robot_description")
    pkg_lslidar = get_package_share_directory("lslidar_driver")
    pkg_control = get_package_share_directory("cleaning_robot_control")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    urdf_file = os.path.join(
        pkg_description,
        "urdf",
        "cleaning_robot.urdf.xacro",
    )
    n10p_net_params_file = os.path.join(
        pkg_lslidar,
        "params",
        "lidar_net_ros2",
        "cleaning_robot_n10p_net.yaml",
    )
    default_nav2_params_file = os.path.join(
        pkg_navigation,
        "config",
        "nav2_params.yaml",
    )
    default_motor_config_file = os.path.join(
        pkg_control,
        "config",
        "track_motor_driver.yaml",
    )
    default_rviz_config_file = os.path.join(
        pkg_nav2_bringup,
        "rviz",
        "nav2_default_view.rviz",
    )
    nav2_bringup_launch = os.path.join(
        pkg_nav2_bringup,
        "launch",
        "bringup_launch.py",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_scan_matcher = LaunchConfiguration("use_scan_matcher")
    use_motor_driver = LaunchConfiguration("use_motor_driver")
    slam = LaunchConfiguration("slam")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    motor_config_file = LaunchConfiguration("motor_config_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    device_ip = LaunchConfiguration("device_ip")
    host_ip = LaunchConfiguration("host_ip")

    def nav2_bool(value):
        return PythonExpression([
            "'",
            value,
            "'.lower() in ['true', '1', 'yes', 'on']",
        ])

    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    n10p_driver_node = Node(
        package="lslidar_driver",
        executable="lslidar_driver_node",
        name="lslidar_driver_node",
        output="screen",
        parameters=[
            n10p_net_params_file,
            {
                "use_sim_time": use_sim_time,
                "device_ip": device_ip,
                "device_ip_difop": host_ip,
            },
        ],
        remappings=[
            ("/scan", "/cleaning_robot/scan"),
            ("/lslidar_point_cloud", "/cleaning_robot/pointcloud"),
        ],
    )

    static_tf_laser_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_laser",
        arguments=["0", "0", "0", "0", "0", "0", "lidar_link", "laser"],
    )

    simple_odom_publisher_node = Node(
        package="cleaning_robot_navigation",
        executable="simple_odom_publisher",
        name="simple_odom_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_scan_matcher),
    )

    laser_scan_matcher_node = Node(
        package="ros2_laser_scan_matcher",
        executable="laser_scan_matcher",
        name="laser_scan_matcher_node",
        output="screen",
        condition=IfCondition(use_scan_matcher),
        parameters=[{
            "use_sim_time": use_sim_time,
            "base_frame": "base_link",
            "odom_frame": "odom",
            "map_frame": "map",
            "laser_frame": "lidar_link",
            "publish_odom": "/odom",
            "publish_tf": True,
        }],
        remappings=[
            ("scan", "/cleaning_robot/scan"),
        ],
    )

    track_motor_driver_node = Node(
        package="cleaning_robot_control",
        executable="track_motor_driver",
        name="track_motor_driver",
        output="screen",
        parameters=[motor_config_file],
        condition=IfCondition(use_motor_driver),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            "namespace": "",
            "use_namespace": "false",
            "slam": nav2_bool(slam),
            "map": map_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": nav2_bool(use_composition),
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with the Nav2 view.",
        ),
        DeclareLaunchArgument(
            "use_scan_matcher",
            default_value="false",
            description="Use ros2_laser_scan_matcher for odom instead of simple open-loop odom.",
        ),
        DeclareLaunchArgument(
            "use_motor_driver",
            default_value="false",
            description="Start the track motor driver. Keep false on a VM.",
        ),
        DeclareLaunchArgument(
            "slam",
            default_value="true",
            description="Run SLAM while navigating. Set false when using a saved map with AMCL.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Map YAML for localization mode. Required when slam is false.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_nav2_params_file,
            description="Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=default_rviz_config_file,
            description="RViz config file.",
        ),
        DeclareLaunchArgument(
            "motor_config_file",
            default_value=default_motor_config_file,
            description="Track motor driver config file.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate Nav2 lifecycle nodes.",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="false",
            description="Use Nav2 component composition.",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="false",
            description="Respawn Nav2 nodes if they exit.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Nav2 log level.",
        ),
        DeclareLaunchArgument(
            "device_ip",
            default_value="192.168.1.200",
            description="N10P LiDAR IP address.",
        ),
        DeclareLaunchArgument(
            "host_ip",
            default_value="192.168.1.102",
            description="Host computer IP address connected to the N10P LiDAR.",
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_tf_laser_node,
        simple_odom_publisher_node,
        laser_scan_matcher_node,
        track_motor_driver_node,
        n10p_driver_node,
        TimerAction(period=3.0, actions=[nav2_bringup]),
        TimerAction(period=5.0, actions=[rviz_node]),
    ])
