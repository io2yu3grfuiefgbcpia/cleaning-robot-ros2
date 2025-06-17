#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Gazebo启动
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    # 在Gazebo中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'cleaning_robot'],
        output='screen'
    )

    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ]) 