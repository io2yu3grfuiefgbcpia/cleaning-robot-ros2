#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import lifecycle_msgs.msg
import os

def generate_launch_description():
    # N10P激光雷达配置文件路径
    driver_dir = os.path.join(
        get_package_share_directory('lslidar_driver'), 
        'params', 'lidar_uart_ros2', 'cleaning_robot_n10p.yaml'
    )
                     
    # 镭神N10P激光雷达驱动节点
    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[driver_dir],
    )

    return LaunchDescription([
        driver_node,
    ]) 