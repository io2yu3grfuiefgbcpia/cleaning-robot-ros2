#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 获取包路径
    cleaning_robot_description_path = get_package_share_directory('cleaning_robot_description')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    enable_orbbec_arg = DeclareLaunchArgument(
        'enable_orbbec',
        default_value='true',
        description='是否启用Orbbec深度相机'
    )
    
    enable_gazebo_arg = DeclareLaunchArgument(
        'enable_gazebo',
        default_value='false',
        description='是否启用Gazebo仿真'
    )
    
    # 获取参数值
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_orbbec = LaunchConfiguration('enable_orbbec')
    enable_gazebo = LaunchConfiguration('enable_gazebo')
    
    # 机器人状态发布器
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cleaning_robot_description'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Gazebo仿真（可选）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cleaning_robot_description'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=LaunchCondition(enable_gazebo)
    )
    
    # Orbbec深度相机节点
    orbbec_camera_node = Node(
        package='cleaning_robot_perception',
        executable='orbbec_camera_node',
        name='orbbec_camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_name': 'orbbec',
            'enable_color': True,
            'enable_depth': True,
            'enable_pointcloud': True,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
            'color_fps': 30,
            'depth_fps': 30
        }],
        condition=LaunchCondition(enable_orbbec)
    )
    
    # 深度相机处理器
    depth_camera_processor = Node(
        package='cleaning_robot_perception',
        executable='depth_camera_processor_node',
        name='depth_camera_processor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/cleaning_robot/depth_camera/depth/image_raw', '/cleaning_robot/orbbec_camera/depth/image_raw'),
            ('/cleaning_robot/depth_camera/color/image_raw', '/cleaning_robot/orbbec_camera/color/image_raw'),
            ('/cleaning_robot/depth_camera/depth/camera_info', '/cleaning_robot/orbbec_camera/depth/camera_info')
        ]
    )
    
    # RViz可视化配置
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('cleaning_robot_description'),
        'config',
        'cleaning_robot_orbbec.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # 启动参数
        use_sim_time_arg,
        enable_orbbec_arg,
        enable_gazebo_arg,
        
        # 节点
        robot_state_publisher,
        gazebo_launch,
        orbbec_camera_node,
        depth_camera_processor,
        rviz_node
    ])


# 导入缺失的LaunchCondition
from launch.conditions import LaunchCondition 