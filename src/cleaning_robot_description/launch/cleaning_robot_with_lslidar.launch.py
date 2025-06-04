#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    description_pkg = get_package_share_directory('cleaning_robot_description')
    slam_pkg = get_package_share_directory('cleaning_robot_slam')
    perception_pkg = get_package_share_directory('cleaning_robot_perception')
    control_pkg = get_package_share_directory('cleaning_robot_control')
    
    # 配置文件路径
    urdf_file = os.path.join(description_pkg, 'urdf', 'cleaning_robot.urdf.xacro')
    rviz_config = os.path.join(description_pkg, 'rviz', 'cleaning_robot.rviz')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # 镭神N10P激光雷达启动
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lslidar_driver'), 
                        'launch', 'cleaning_robot_n10p.launch.py')
        ])
    )
    
    # 多传感器SLAM节点
    slam_node = Node(
        package='cleaning_robot_slam',
        executable='multi_sensor_slam',
        name='multi_sensor_slam',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': 0.05,
            'map_width': 2000,
            'map_height': 2000,
            'map_origin_x': -50.0,
            'map_origin_y': -50.0
        }]
    )
    
    # 双目视觉处理节点
    stereo_processor = Node(
        package='cleaning_robot_perception',
        executable='stereo_processor',
        name='stereo_processor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'baseline': 0.12,
            'focal_length': 500.0,
            'cx': 320.0,
            'cy': 240.0
        }]
    )
    
    # 清扫控制节点
    cleaning_controller = Node(
        package='cleaning_robot_control',
        executable='cleaning_controller',
        name='cleaning_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'safety_distance': 0.3
        }]
    )
    
    # RViz2可视化
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        lslidar_launch,
        slam_node,
        stereo_processor,
        cleaning_controller,
        rviz2
    ]) 