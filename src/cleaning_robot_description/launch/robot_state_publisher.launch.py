#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 获取包的路径
    pkg_path = os.path.join(get_package_share_directory('cleaning_robot_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'cleaning_robot.urdf.xacro')
    
    # 处理xacro文件
    robot_description_config = xacro.process_file(xacro_file)
    
    # 创建参数字典
    params = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Joint State Publisher节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ]) 