#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import message_filters
from sensor_msgs_py import point_cloud2
import threading

class StereoProcessor(Node):
    def __init__(self):
        super().__init__('stereo_processor')
        
        self.bridge = CvBridge()
        
        # 双目摄像头参数
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        self.dist_coeffs_left = None
        self.dist_coeffs_right = None
        self.R = None
        self.T = None
        self.baseline = 0.12  # 基线距离，单位：米
        
        # 订阅者
        self.left_image_sub = message_filters.Subscriber(self, Image, '/cleaning_robot/camera/left/image_raw')
        self.right_image_sub = message_filters.Subscriber(self, Image, '/cleaning_robot/camera/right/image_raw')
        self.left_camera_info_sub = self.create_subscription(CameraInfo, '/cleaning_robot/camera/left/camera_info', self.left_camera_info_callback, 10)
        self.right_camera_info_sub = self.create_subscription(CameraInfo, '/cleaning_robot/camera/right/camera_info', self.right_camera_info_callback, 10)
        
        # 时间同步
        self.ts = message_filters.TimeSynchronizer([self.left_image_sub, self.right_image_sub], 10)
        self.ts.registerCallback(self.stereo_callback)
        
        # 发布者
        self.disparity_pub = self.create_publisher(Image, '/cleaning_robot/stereo/disparity', 10)
        self.depth_pub = self.create_publisher(Image, '/cleaning_robot/stereo/depth', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/cleaning_robot/stereo/pointcloud', 10)
        
        # OpenCV立体匹配器
        self.stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)
        
        # 参数
        self.focal_length = 525.0  # 假设焦距
        
        self.get_logger().info('双目摄像头处理节点已启动')
    
    def left_camera_info_callback(self, msg):
        """左摄像头信息回调"""
        self.camera_matrix_left = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs_left = np.array(msg.d)
        self.focal_length = self.camera_matrix_left[0, 0]
    
    def right_camera_info_callback(self, msg):
        """右摄像头信息回调"""
        self.camera_matrix_right = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs_right = np.array(msg.d)
    
    def stereo_callback(self, left_msg, right_msg):
        """双目图像回调函数"""
        try:
            # 将ROS图像转换为OpenCV格式
            left_image = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
            right_image = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
            
            # 转换为灰度图像
            left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
            
            # 计算视差图
            disparity = self.stereo.compute(left_gray, right_gray)
            
            # 归一化视差图用于显示
            disparity_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
            disparity_normalized = disparity_normalized.astype(np.uint8)
            
            # 计算深度图
            depth_image = self.compute_depth(disparity)
            
            # 生成点云
            pointcloud = self.generate_pointcloud(disparity, left_image)
            
            # 发布结果
            self.publish_disparity(disparity_normalized, left_msg.header)
            self.publish_depth(depth_image, left_msg.header)
            self.publish_pointcloud(pointcloud, left_msg.header)
            
        except Exception as e:
            self.get_logger().error(f'双目处理错误: {str(e)}')
    
    def compute_depth(self, disparity):
        """计算深度图"""
        # 避免除零
        disparity_float = disparity.astype(np.float32) / 16.0  # StereoBM返回的视差值需要除以16
        disparity_float[disparity_float <= 0] = 0.1
        
        # 深度 = (焦距 * 基线) / 视差
        depth = (self.focal_length * self.baseline) / disparity_float
        
        # 限制深度范围
        depth = np.clip(depth, 0.1, 10.0)
        
        return depth
    
    def generate_pointcloud(self, disparity, color_image):
        """生成彩色点云"""
        height, width = disparity.shape
        points = []
        
        # 计算深度
        disparity_float = disparity.astype(np.float32) / 16.0
        depth = self.compute_depth(disparity)
        
        # 相机内参
        fx = self.focal_length
        fy = self.focal_length
        cx = width / 2.0
        cy = height / 2.0
        
        for v in range(0, height, 4):  # 跳过一些点以减少计算量
            for u in range(0, width, 4):
                if disparity_float[v, u] > 0:
                    z = depth[v, u]
                    if 0.1 < z < 5.0:  # 只保留合理范围内的点
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        
                        # 获取颜色信息
                        b, g, r = color_image[v, u]
                        rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                        
                        points.append([x, y, z, rgb])
        
        return points
    
    def publish_disparity(self, disparity, header):
        """发布视差图"""
        try:
            disparity_msg = self.bridge.cv2_to_imgmsg(disparity, "mono8")
            disparity_msg.header = header
            self.disparity_pub.publish(disparity_msg)
        except Exception as e:
            self.get_logger().error(f'发布视差图错误: {str(e)}')
    
    def publish_depth(self, depth, header):
        """发布深度图"""
        try:
            # 将深度图转换为16位整数（毫米单位）
            depth_mm = (depth * 1000).astype(np.uint16)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, "16UC1")
            depth_msg.header = header
            self.depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(f'发布深度图错误: {str(e)}')
    
    def publish_pointcloud(self, points, header):
        """发布点云"""
        try:
            if len(points) > 0:
                fields = [
                    ('x', np.float32),
                    ('y', np.float32), 
                    ('z', np.float32),
                    ('rgb', np.uint32)
                ]
                
                pointcloud_msg = point_cloud2.create_cloud(header, fields, points)
                pointcloud_msg.header.frame_id = "camera_left_link"
                self.pointcloud_pub.publish(pointcloud_msg)
        except Exception as e:
            self.get_logger().error(f'发布点云错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    stereo_processor = StereoProcessor()
    
    try:
        rclpy.spin(stereo_processor)
    except KeyboardInterrupt:
        pass
    finally:
        stereo_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 