#!/usr/bin/env python3
"""
清扫机器人完整系统启动文件
功能：
1. 启动机器人模型和状态发布
2. 启动激光雷达驱动
3. 启动SLAM建图
4. 启动导航系统
5. 启动摄像头处理
6. 启动清扫控制器
7. 启动RViz可视化
8. 可选启动控制面板
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution,
    PythonExpression
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # 包路径
    pkg_description = get_package_share_directory('cleaning_robot_description')
    pkg_navigation = get_package_share_directory('cleaning_robot_navigation')
    
    # URDF文件处理
    urdf_file = os.path.join(pkg_description, 'urdf', 'cleaning_robot.urdf.xacro')
    
    # 检查文件是否存在
    if os.path.exists(urdf_file):
        robot_description = xacro.process_file(urdf_file).toxml()
    else:
        # 使用简单的URDF
        robot_description = '''<?xml version="1.0"?>
        <robot name="cleaning_robot">
            <link name="base_link">
                <visual>
                    <geometry><box size="0.5 0.4 0.2"/></geometry>
                </visual>
            </link>
            <link name="lidar_link"/>
            <joint name="lidar_joint" type="fixed">
                <parent link="base_link"/>
                <child link="lidar_link"/>
                <origin xyz="0 0 0.15"/>
            </joint>
        </robot>'''
    
    # RViz配置文件
    rviz_config = os.path.join(pkg_description, 'rviz', 'cleaning_robot_full.rviz')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_dashboard = LaunchConfiguration('use_dashboard')
    lidar_type = LaunchConfiguration('lidar_type')
    
    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='启动RViz可视化')
    
    declare_use_dashboard = DeclareLaunchArgument(
        'use_dashboard',
        default_value='false',
        description='启动控制面板')
    
    declare_lidar_type = DeclareLaunchArgument(
        'lidar_type',
        default_value='n10p_net',
        description='激光雷达类型: n10p_net, n10p_serial, simulated')
    
    # 1. 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # 2. 关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 3. 里程计发布器（简单版本）
    odom_publisher = Node(
        package='cleaning_robot_navigation',
        executable='simple_odom_publisher',
        name='simple_odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 4. 静态TF变换
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    static_tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_lidar',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    static_tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser']
    )
    
    # 5. N10P激光雷达驱动（网络版）
    lidar_driver = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[{
            'frame_id': 'lidar_link',
            'device_ip': '192.168.1.200',
            'device_ip_difop': '192.168.1.102',
            'msop_port': 2368,
            'difop_port': 2369,
            'lidar_name': 'N10_P',
            'interface_selection': 'net',
            'min_range': 0.1,
            'max_range': 20.0,
            'pubScan': True,
            'pubPointCloud2': True,
            'angle_disable_min': 0.0,
            'angle_disable_max': 0.0,
            'scan_topic': '/cleaning_robot/scan'
        }],
        remappings=[
            ('/scan', '/cleaning_robot/scan'),
            ('/lslidar_point_cloud', '/cleaning_robot/pointcloud')
        ]
    )
    
    # 6. SLAM建图
    slam_toolbox = Node(
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
            'map_update_interval': 2.0,
            'resolution': 0.05,
            'max_laser_range': 20.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': True
        }]
    )
    
    # 7. 双目视觉处理器
    stereo_processor = Node(
        package='cleaning_robot_perception',
        executable='stereo_processor',
        name='stereo_processor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'baseline': 0.12,
            'focal_length': 500.0
        }]
    )
    
    # 8. 清扫控制器
    cleaning_controller = Node(
        package='cleaning_robot_control',
        executable='cleaning_controller',
        name='cleaning_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cleaning_speed': 0.2,
            'turn_speed': 0.5,
            'cleaning_width': 0.3,
            'obstacle_distance': 0.5,
            'goal_tolerance': 0.1
        }]
    )
    
    # 9. RViz2可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    # 10. 控制面板（延迟启动）
    dashboard_node = TimerAction(
        period=5.0,  # 延迟5秒启动
        actions=[
            Node(
                package='cleaning_robot_control',
                executable='cleaning_dashboard',
                name='cleaning_dashboard',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_dashboard)
            )
        ]
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_dashboard)
    ld.add_action(declare_lidar_type)
    
    # 添加节点（按顺序）
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(static_tf_map_odom)
    ld.add_action(static_tf_base_lidar)
    ld.add_action(static_tf_base_laser)
    ld.add_action(odom_publisher)
    ld.add_action(lidar_driver)
    
    # 延迟启动SLAM（等待TF就绪）
    ld.add_action(TimerAction(period=2.0, actions=[slam_toolbox]))
    
    # 延迟启动其他节点
    ld.add_action(TimerAction(period=3.0, actions=[stereo_processor]))
    ld.add_action(TimerAction(period=3.0, actions=[cleaning_controller]))
    ld.add_action(TimerAction(period=4.0, actions=[rviz_node]))
    ld.add_action(dashboard_node)
    
    return ld

