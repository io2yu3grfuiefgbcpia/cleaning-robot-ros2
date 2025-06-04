#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    清扫机器人N10P网络版完整系统启动文件
    适用于以太网连接的N10P激光雷达
    """
    
    # 包路径
    cleaning_robot_navigation_dir = FindPackageShare(package='cleaning_robot_navigation').find('cleaning_robot_navigation')
    cleaning_robot_description_dir = FindPackageShare(package='cleaning_robot_description').find('cleaning_robot_description')
    lslidar_driver_dir = FindPackageShare(package='lslidar_driver').find('lslidar_driver')
    
    # 配置文件路径
    urdf_file = os.path.join(cleaning_robot_description_dir, 'urdf', 'cleaning_robot.urdf')
    rviz_config_file = os.path.join(cleaning_robot_navigation_dir, 'rviz', 'cleaning_robot_n10p_net.rviz')
    n10p_net_params_file = os.path.join(lslidar_driver_dir, 'params', 'lidar_net_ros2', 'cleaning_robot_n10p_net.yaml')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    device_ip = LaunchConfiguration('device_ip', default='192.168.1.200')
    host_ip = LaunchConfiguration('host_ip', default='192.168.1.102')
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    declare_device_ip_cmd = DeclareLaunchArgument(
        'device_ip',
        default_value='192.168.1.200',
        description='N10P LiDAR IP address')
    
    declare_host_ip_cmd = DeclareLaunchArgument(
        'host_ip',
        default_value='192.168.1.102', 
        description='Host computer IP address')
    
    # 读取URDF文件
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }],
    )
    
    # Joint State Publisher节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # 镭神N10P激光雷达驱动节点（网络版）
    n10p_driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[
            n10p_net_params_file, 
            {
                'use_sim_time': use_sim_time,
                'device_ip': device_ip,
                'device_ip_difop': host_ip
            }
        ],
        remappings=[
            ('/scan', '/cleaning_robot/scan'),
            ('/lslidar_point_cloud', '/cleaning_robot/pointcloud')
        ]
    )
    
    # 静态TF发布器 - base_link到lidar_link
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_lidar',
        arguments=['0.15', '0', '0.2', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    # 静态TF发布器 - base_link到laser (与某些算法兼容)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        arguments=['0.15', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
    )
    
    # 激光扫描匹配器 - 用于里程计估计
    laser_scan_matcher_node = Node(
        package='laser_scan_matcher',
        executable='laser_scan_matcher_node',
        name='laser_scan_matcher_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': 'base_link',
            'fixed_frame': 'odom',
            'use_alpha_beta': False,
            'use_odom': False,
            'use_vel': False,
            'publish_tf': True,
            'publish_pose': True
        }],
        remappings=[
            ('/scan', '/cleaning_robot/scan'),
            ('/pose_with_covariance_stamped', '/cleaning_robot/pose')
        ]
    )
    
    # SLAM算法节点（使用slam_toolbox）
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/cleaning_robot/scan',
            'mode': 'mapping',
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': 'None',
            'throttle_scans': 1,
            'transform_publish_period': 0.02,
            'map_update_interval': 5.0,
            'resolution': 0.05,
            'max_laser_range': 20.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': True
        }]
    )
    
    # RVIZ可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加声明的参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_device_ip_cmd)
    ld.add_action(declare_host_ip_cmd)
    
    # 添加节点
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(static_tf_lidar)
    ld.add_action(static_tf_laser)
    ld.add_action(n10p_driver_node)
    ld.add_action(laser_scan_matcher_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    
    return ld 