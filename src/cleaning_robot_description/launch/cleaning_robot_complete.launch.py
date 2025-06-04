#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo启动
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'verbose': 'true'}.items()
    )

    # 在Gazebo中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'cleaning_robot',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}]
    )

    # Joint State Publisher节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 双目摄像头处理节点
    stereo_processor_node = Node(
        package='cleaning_robot_perception',
        executable='stereo_processor',
        name='stereo_processor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 多传感器SLAM节点
    slam_node = Node(
        package='cleaning_robot_slam',
        executable='multi_sensor_slam',
        name='multi_sensor_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 清扫控制节点
    cleaning_controller_node = Node(
        package='cleaning_robot_control',
        executable='cleaning_controller',
        name='cleaning_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz2节点
    rviz_config_file = os.path.join(pkg_path, 'config', 'cleaning_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        joint_state_publisher_node,
        stereo_processor_node,
        slam_node,
        cleaning_controller_node,
        rviz_node,
    ]) 