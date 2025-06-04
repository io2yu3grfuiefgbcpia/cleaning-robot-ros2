#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import math
from collections import deque
import time

class CleaningController(Node):
    def __init__(self):
        super().__init__('cleaning_controller')
        
        # 清扫状态
        self.cleaning_active = False
        self.current_pose = None
        self.current_map = None
        self.cleaned_areas = set()  # 已清扫区域
        self.cleaning_path = deque()  # 清扫路径队列
        
        # 清扫参数
        self.cleaning_width = 0.3  # 清扫宽度（米）
        self.overlap_ratio = 0.1   # 重叠比例
        self.cleaning_speed = 0.2  # 清扫速度（米/秒）
        self.turn_speed = 0.5      # 转弯角速度
        
        # 导航参数
        self.goal_tolerance = 0.1  # 目标点容差
        self.obstacle_distance = 0.5  # 障碍物距离阈值
        
        # 订阅者
        self.pose_sub = self.create_subscription(PoseStamped, '/cleaning_robot/pose', self.pose_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/cleaning_robot/scan', self.laser_callback, 10)
        self.start_cleaning_sub = self.create_subscription(Bool, '/cleaning_robot/start_cleaning', self.start_cleaning_callback, 10)
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cleaning_robot/cmd_vel', 10)
        self.cleaning_path_pub = self.create_publisher(Path, '/cleaning_robot/cleaning_path', 10)
        self.cleaning_status_pub = self.create_publisher(Bool, '/cleaning_robot/cleaning_status', 10)
        self.brush_speed_pub = self.create_publisher(Float32, '/cleaning_robot/brush_speed', 10)
        
        # 定时器
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.path_planning_timer = self.create_timer(2.0, self.plan_cleaning_path)
        
        # 状态变量
        self.current_goal = None
        self.last_laser_data = None
        
        self.get_logger().info('清扫控制节点已启动')
    
    def pose_callback(self, msg):
        """位姿回调函数"""
        self.current_pose = msg
    
    def map_callback(self, msg):
        """地图回调函数"""
        self.current_map = msg
        if self.cleaning_active and not self.cleaning_path:
            # 如果正在清扫但没有路径，重新规划
            self.plan_cleaning_path()
    
    def laser_callback(self, msg):
        """激光雷达回调函数"""
        self.last_laser_data = msg
    
    def start_cleaning_callback(self, msg):
        """开始清扫回调函数"""
        if msg.data and not self.cleaning_active:
            self.start_cleaning()
        elif not msg.data and self.cleaning_active:
            self.stop_cleaning()
    
    def start_cleaning(self):
        """开始清扫"""
        self.cleaning_active = True
        self.cleaned_areas.clear()
        self.cleaning_path.clear()
        
        # 启动清扫刷
        brush_msg = Float32()
        brush_msg.data = 1.0  # 满速
        self.brush_speed_pub.publish(brush_msg)
        
        self.get_logger().info('开始自动清扫')
        
        # 发布清扫状态
        status_msg = Bool()
        status_msg.data = True
        self.cleaning_status_pub.publish(status_msg)
    
    def stop_cleaning(self):
        """停止清扫"""
        self.cleaning_active = False
        self.cleaning_path.clear()
        self.current_goal = None
        
        # 停止机器人
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # 停止清扫刷
        brush_msg = Float32()
        brush_msg.data = 0.0
        self.brush_speed_pub.publish(brush_msg)
        
        self.get_logger().info('停止自动清扫')
        
        # 发布清扫状态
        status_msg = Bool()
        status_msg.data = False
        self.cleaning_status_pub.publish(status_msg)
    
    def plan_cleaning_path(self):
        """规划清扫路径"""
        if not self.cleaning_active or not self.current_map or not self.current_pose:
            return
        
        try:
            # 使用弓字形路径规划
            path = self.generate_boustrophedon_path()
            
            if path:
                self.cleaning_path = deque(path)
                self.publish_cleaning_path(path)
                self.get_logger().info(f'生成清扫路径，包含 {len(path)} 个点')
            
        except Exception as e:
            self.get_logger().error(f'路径规划错误: {str(e)}')
    
    def generate_boustrophedon_path(self):
        """生成弓字形清扫路径"""
        if not self.current_map:
            return []
        
        # 地图参数
        resolution = self.current_map.info.resolution
        width = self.current_map.info.width
        height = self.current_map.info.height
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # 转换地图数据
        map_data = np.array(self.current_map.data).reshape((height, width))
        
        # 找到自由空间
        free_space = (map_data == 0)
        
        # 计算清扫条带宽度（格子数）
        strip_width = int(self.cleaning_width / resolution)
        
        path = []
        
        # 从左到右扫描
        for x in range(0, width, strip_width):
            if x % (2 * strip_width) == 0:
                # 从下到上
                y_range = range(0, height)
            else:
                # 从上到下
                y_range = range(height - 1, -1, -1)
            
            strip_points = []
            for y in y_range:
                if 0 <= x < width and 0 <= y < height and free_space[y, x]:
                    # 转换为世界坐标
                    world_x = origin_x + x * resolution
                    world_y = origin_y + y * resolution
                    strip_points.append((world_x, world_y))
            
            # 如果条带中有点，添加到路径中
            if strip_points:
                if len(strip_points) > 10:  # 只添加较长的条带
                    path.extend(strip_points)
        
        return path
    
    def publish_cleaning_path(self, path_points):
        """发布清扫路径"""
        try:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            
            for x, y in path_points:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            
            self.cleaning_path_pub.publish(path_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布路径错误: {str(e)}')
    
    def control_loop(self):
        """主控制循环"""
        if not self.cleaning_active or not self.current_pose:
            return
        
        try:
            # 检查是否有当前目标
            if not self.current_goal:
                self.get_next_goal()
            
            if self.current_goal:
                # 导航到目标点
                self.navigate_to_goal()
            
            # 标记已清扫区域
            self.mark_cleaned_area()
            
        except Exception as e:
            self.get_logger().error(f'控制循环错误: {str(e)}')
    
    def get_next_goal(self):
        """获取下一个目标点"""
        if self.cleaning_path:
            x, y = self.cleaning_path.popleft()
            self.current_goal = (x, y)
        else:
            # 清扫完成
            self.stop_cleaning()
            self.get_logger().info('清扫任务完成')
    
    def navigate_to_goal(self):
        """导航到目标点"""
        if not self.current_goal or not self.current_pose:
            return
        
        # 计算到目标点的距离和角度
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        # 从四元数获取当前航向角
        q = self.current_pose.pose.orientation
        current_theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        goal_x, goal_y = self.current_goal
        
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # 计算角度差
        angle_diff = self.normalize_angle(target_angle - current_theta)
        
        cmd = Twist()
        
        if distance < self.goal_tolerance:
            # 到达目标点
            self.current_goal = None
        else:
            # 检查障碍物
            if self.check_obstacle():
                # 有障碍物，停止或绕行
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed if angle_diff > 0 else -self.turn_speed
            else:
                # 无障碍物，正常导航
                if abs(angle_diff) > 0.1:
                    # 需要转弯
                    cmd.angular.z = self.turn_speed if angle_diff > 0 else -self.turn_speed
                    cmd.linear.x = self.cleaning_speed * 0.5  # 转弯时减速
                else:
                    # 直行
                    cmd.linear.x = self.cleaning_speed
                    cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
    
    def check_obstacle(self):
        """检查前方是否有障碍物"""
        if not self.last_laser_data:
            return False
        
        ranges = self.last_laser_data.ranges
        min_angle_idx = len(ranges) // 3
        max_angle_idx = 2 * len(ranges) // 3
        
        front_ranges = ranges[min_angle_idx:max_angle_idx]
        min_distance = min([r for r in front_ranges if r > 0])
        
        return min_distance < self.obstacle_distance
    
    def mark_cleaned_area(self):
        """标记已清扫区域"""
        if not self.current_pose:
            return
        
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        
        # 将位置量化为网格
        grid_x = int(x * 10)  # 10cm精度
        grid_y = int(y * 10)
        
        self.cleaned_areas.add((grid_x, grid_y))
    
    def normalize_angle(self, angle):
        """角度归一化到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    
    controller = CleaningController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 