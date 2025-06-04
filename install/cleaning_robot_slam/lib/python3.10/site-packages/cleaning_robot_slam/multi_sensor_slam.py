#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
import math
from builtin_interfaces.msg import Time

class MultiSensorSLAM(Node):
    def __init__(self):
        super().__init__('multi_sensor_slam')
        
        # 地图参数
        self.map_width = 2000  # 格子数
        self.map_height = 2000
        self.map_resolution = 0.05  # 米/格子
        self.map_origin_x = -50.0  # 米
        self.map_origin_y = -50.0
        
        # 初始化地图
        self.occupancy_map = np.ones((self.map_height, self.map_width), dtype=np.int8) * -1  # -1表示未知
        
        # 机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # 订阅者
        self.laser_sub = self.create_subscription(LaserScan, '/cleaning_robot/scan', self.laser_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/cleaning_robot/stereo/pointcloud', self.pointcloud_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/cleaning_robot/odom', self.odom_callback, 10)
        
        # 发布者
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/cleaning_robot/pose', 10)
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 定时器
        self.map_timer = self.create_timer(1.0, self.publish_map)
        self.pose_timer = self.create_timer(0.1, self.publish_pose)
        
        # SLAM参数
        self.laser_range_max = 10.0
        self.laser_range_min = 0.1
        
        self.get_logger().info('多传感器SLAM节点已启动')
    
    def odom_callback(self, msg):
        """里程计回调函数"""
        # 更新机器人位置（这里简化处理，实际SLAM需要更复杂的算法）
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # 从四元数转换为欧拉角
        q = msg.pose.pose.orientation
        self.robot_theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    
    def laser_callback(self, msg):
        """激光雷达回调函数"""
        try:
            self.update_map_with_laser(msg)
        except Exception as e:
            self.get_logger().error(f'激光雷达建图错误: {str(e)}')
    
    def pointcloud_callback(self, msg):
        """点云回调函数"""
        try:
            # 这里可以添加基于点云的建图逻辑
            # 目前主要使用激光雷达进行建图
            pass
        except Exception as e:
            self.get_logger().error(f'点云处理错误: {str(e)}')
    
    def update_map_with_laser(self, laser_msg):
        """使用激光雷达数据更新地图"""
        ranges = np.array(laser_msg.ranges)
        angles = np.linspace(laser_msg.angle_min, laser_msg.angle_max, len(ranges))
        
        # 机器人在地图中的位置
        robot_map_x = int((self.robot_x - self.map_origin_x) / self.map_resolution)
        robot_map_y = int((self.robot_y - self.map_origin_y) / self.map_resolution)
        
        for i, (distance, angle) in enumerate(zip(ranges, angles)):
            if self.laser_range_min < distance < self.laser_range_max:
                # 计算激光点的全局坐标
                global_angle = self.robot_theta + angle
                end_x = self.robot_x + distance * math.cos(global_angle)
                end_y = self.robot_y + distance * math.sin(global_angle)
                
                # 转换为地图坐标
                end_map_x = int((end_x - self.map_origin_x) / self.map_resolution)
                end_map_y = int((end_y - self.map_origin_y) / self.map_resolution)
                
                # 使用布雷森汉姆线算法更新地图
                self.update_line(robot_map_x, robot_map_y, end_map_x, end_map_y)
    
    def update_line(self, x0, y0, x1, y1):
        """使用布雷森汉姆算法更新一条线上的格子"""
        points = self.bresenham_line(x0, y0, x1, y1)
        
        for i, (x, y) in enumerate(points):
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if i == len(points) - 1:
                    # 最后一个点是障碍物
                    self.occupancy_map[y, x] = 100
                else:
                    # 其他点是自由空间
                    if self.occupancy_map[y, x] == -1:  # 只更新未知格子
                        self.occupancy_map[y, x] = 0
    
    def bresenham_line(self, x0, y0, x1, y1):
        """布雷森汉姆线算法"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def publish_map(self):
        """发布地图"""
        try:
            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = "map"
            
            map_msg.info.resolution = self.map_resolution
            map_msg.info.width = self.map_width
            map_msg.info.height = self.map_height
            map_msg.info.origin.position.x = self.map_origin_x
            map_msg.info.origin.position.y = self.map_origin_y
            map_msg.info.origin.position.z = 0.0
            map_msg.info.origin.orientation.w = 1.0
            
            # 转换地图数据格式
            map_data = self.occupancy_map.flatten().tolist()
            map_msg.data = map_data
            
            self.map_pub.publish(map_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布地图错误: {str(e)}')
    
    def publish_pose(self):
        """发布机器人位姿"""
        try:
            # 发布PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = self.robot_x
            pose_msg.pose.position.y = self.robot_y
            pose_msg.pose.position.z = 0.0
            
            # 欧拉角转四元数
            cy = math.cos(self.robot_theta * 0.5)
            sy = math.sin(self.robot_theta * 0.5)
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = sy
            pose_msg.pose.orientation.w = cy
            
            self.pose_pub.publish(pose_msg)
            
            # 发布TF变换
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.robot_x
            t.transform.translation.y = self.robot_y
            t.transform.translation.z = 0.0
            t.transform.rotation = pose_msg.pose.orientation
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'发布位姿错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    slam_node = MultiSensorSLAM()
    
    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 