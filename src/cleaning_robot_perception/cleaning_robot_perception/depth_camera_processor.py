#!/usr/bin/env python3

"""
深度相机处理器
支持多种深度相机，包括RealSense、Kinect、奥比中光等
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
import time

class DepthCameraProcessor(Node):
    def __init__(self):
        super().__init__('depth_camera_processor')
        
        self.bridge = CvBridge()
        
        # 深度相机参数
        self.depth_scale = 0.001  # 深度值转换比例（通常为毫米转米）
        self.min_depth = 0.1      # 最小有效深度（米）
        self.max_depth = 5.0      # 最大有效深度（米）
        
        # 相机内参（这些值需要根据实际相机标定结果设置）
        self.fx = 525.0  # 焦距x
        self.fy = 525.0  # 焦距y
        self.cx = 320.0  # 主点x
        self.cy = 240.0  # 主点y
        
        # 订阅深度相机数据
        self.depth_sub = self.create_subscription(
            Image, '/cleaning_robot/depth_camera/depth/image_raw', 
            self.depth_callback, 10)
        
        self.color_sub = self.create_subscription(
            Image, '/cleaning_robot/depth_camera/color/image_raw', 
            self.color_callback, 10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/cleaning_robot/depth_camera/depth/camera_info', 
            self.camera_info_callback, 10)
        
        # 发布处理后的数据
        self.filtered_depth_pub = self.create_publisher(
            Image, '/cleaning_robot/depth_camera/depth_filtered', 10)
        
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, '/cleaning_robot/depth_camera/pointcloud', 10)
        
        self.obstacle_detection_pub = self.create_publisher(
            Image, '/cleaning_robot/depth_camera/obstacles', 10)
        
        # 存储最新的图像数据
        self.latest_depth = None
        self.latest_color = None
        self.latest_depth_time = None
        self.latest_color_time = None
        
        # 创建定时器进行数据处理
        self.processing_timer = self.create_timer(0.1, self.process_data)  # 10Hz
        
        self.get_logger().info('🎥 深度相机处理器已启动')
        self.get_logger().info('等待深度相机数据...')
    
    def camera_info_callback(self, msg):
        """更新相机内参"""
        if len(msg.k) == 9:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f'更新相机内参: fx={self.fx:.1f}, fy={self.fy:.1f}')
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            # 根据编码格式转换深度图像
            if msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            elif msg.encoding == 'mono16':
                depth_image = self.bridge.imgmsg_to_cv2(msg, "mono16")
            else:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            self.latest_depth = depth_image
            self.latest_depth_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f'深度图像处理错误: {str(e)}')
    
    def color_callback(self, msg):
        """彩色图像回调"""
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_color = color_image
            self.latest_color_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f'彩色图像处理错误: {str(e)}')
    
    def process_data(self):
        """处理深度相机数据"""
        if self.latest_depth is None:
            return
        
        try:
            # 1. 深度图像滤波
            filtered_depth = self.filter_depth_image(self.latest_depth)
            
            # 2. 生成点云
            pointcloud = self.generate_pointcloud(filtered_depth, self.latest_color)
            
            # 3. 障碍物检测
            obstacle_map = self.detect_obstacles(filtered_depth)
            
            # 4. 发布结果
            self.publish_filtered_depth(filtered_depth)
            self.publish_pointcloud(pointcloud)
            self.publish_obstacle_detection(obstacle_map)
            
        except Exception as e:
            self.get_logger().error(f'数据处理错误: {str(e)}')
    
    def filter_depth_image(self, depth_image):
        """深度图像滤波和预处理"""
        # 转换为浮点数并应用深度缩放
        depth_float = depth_image.astype(np.float32) * self.depth_scale
        
        # 限制深度范围
        depth_float[depth_float < self.min_depth] = 0
        depth_float[depth_float > self.max_depth] = 0
        
        # 中值滤波去除噪声
        depth_filtered = cv2.medianBlur(depth_float, 5)
        
        # 高斯滤波平滑
        depth_filtered = cv2.GaussianBlur(depth_filtered, (3, 3), 0)
        
        return depth_filtered
    
    def generate_pointcloud(self, depth_image, color_image=None):
        """从深度图像生成3D点云"""
        height, width = depth_image.shape
        points = []
        
        # 遍历每个像素点（降采样以提高性能）
        step = 2  # 跳过一些像素点
        for v in range(0, height, step):
            for u in range(0, width, step):
                depth = depth_image[v, u]
                
                if depth > 0:  # 有效深度值
                    # 转换为3D坐标
                    x = (u - self.cx) * depth / self.fx
                    y = (v - self.cy) * depth / self.fy
                    z = depth
                    
                    # 添加颜色信息（如果有）
                    if color_image is not None and color_image.shape[0] > v and color_image.shape[1] > u:
                        b, g, r = color_image[v, u]
                        rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                        points.append([x, y, z, rgb])
                    else:
                        # 根据深度着色
                        intensity = min(255, int(255 * (1.0 - depth / self.max_depth)))
                        rgb = (intensity << 16) | (intensity << 8) | intensity
                        points.append([x, y, z, rgb])
        
        return points
    
    def detect_obstacles(self, depth_image):
        """基于深度图像的障碍物检测"""
        height, width = depth_image.shape
        obstacle_map = np.zeros((height, width), dtype=np.uint8)
        
        # 定义障碍物检测区域（机器人前方）
        roi_height = int(height * 0.8)  # 关注下方80%的区域
        roi_width = int(width * 0.6)    # 关注中间60%的区域
        roi_y_start = height - roi_height
        roi_x_start = (width - roi_width) // 2
        
        # 在关注区域内检测障碍物
        roi_depth = depth_image[roi_y_start:height, roi_x_start:roi_x_start+roi_width]
        
        # 设置障碍物检测阈值
        obstacle_threshold = 1.5  # 1.5米内的物体视为障碍物
        
        # 检测障碍物
        obstacles = (roi_depth > 0) & (roi_depth < obstacle_threshold)
        obstacle_map[roi_y_start:height, roi_x_start:roi_x_start+roi_width] = obstacles * 255
        
        # 形态学操作减少噪声
        kernel = np.ones((5, 5), np.uint8)
        obstacle_map = cv2.morphologyEx(obstacle_map, cv2.MORPH_CLOSE, kernel)
        obstacle_map = cv2.morphologyEx(obstacle_map, cv2.MORPH_OPEN, kernel)
        
        return obstacle_map
    
    def publish_filtered_depth(self, depth_image):
        """发布滤波后的深度图像"""
        try:
            # 转换为16位整数格式发布
            depth_16bit = (depth_image * 1000).astype(np.uint16)  # 转换为毫米
            depth_msg = self.bridge.cv2_to_imgmsg(depth_16bit, "16UC1")
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "depth_camera_link"
            self.filtered_depth_pub.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布深度图像错误: {str(e)}')
    
    def publish_pointcloud(self, points):
        """发布点云数据"""
        try:
            if len(points) > 0:
                fields = [
                    ('x', np.float32),
                    ('y', np.float32),
                    ('z', np.float32),
                    ('rgb', np.uint32)
                ]
                
                header = self.create_header()
                pointcloud_msg = point_cloud2.create_cloud(header, fields, points)
                self.pointcloud_pub.publish(pointcloud_msg)
                
        except Exception as e:
            self.get_logger().error(f'发布点云错误: {str(e)}')
    
    def publish_obstacle_detection(self, obstacle_map):
        """发布障碍物检测结果"""
        try:
            obstacle_msg = self.bridge.cv2_to_imgmsg(obstacle_map, "mono8")
            obstacle_msg.header.stamp = self.get_clock().now().to_msg()
            obstacle_msg.header.frame_id = "depth_camera_link"
            self.obstacle_detection_pub.publish(obstacle_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布障碍物检测错误: {str(e)}')
    
    def create_header(self):
        """创建消息头"""
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "depth_camera_link"
        return header

def main(args=None):
    rclpy.init(args=args)
    
    depth_processor = DepthCameraProcessor()
    
    try:
        rclpy.spin(depth_processor)
    except KeyboardInterrupt:
        pass
    finally:
        depth_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 