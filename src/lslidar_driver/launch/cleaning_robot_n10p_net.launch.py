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
    """
    清扫机器人N10P激光雷达网络版启动文件
    使用以太网连接方式，适用于网络通讯模式的N10P激光雷达
    """
    
    # 获取N10P网络配置文件路径
    driver_config_file = os.path.join(
        get_package_share_directory('lslidar_driver'), 
        'params', 
        'lidar_net_ros2',
        'cleaning_robot_n10p_net.yaml'
    )
    
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间（Gazebo）如果为true'
    )
    
    declare_device_ip = DeclareLaunchArgument(
        'device_ip',
        default_value='192.168.1.200',
        description='N10P激光雷达的IP地址'
    )
    
    declare_host_ip = DeclareLaunchArgument(
        'host_ip', 
        default_value='192.168.1.102',
        description='接收数据的主机IP地址'
    )
    
    # N10P激光雷达驱动节点（网络版）
    n10p_driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[
            driver_config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'device_ip': LaunchConfiguration('device_ip'),
                'device_ip_difop': LaunchConfiguration('host_ip')
            }
        ],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_device_ip,
        declare_host_ip,
        n10p_driver_node,
    ]) 