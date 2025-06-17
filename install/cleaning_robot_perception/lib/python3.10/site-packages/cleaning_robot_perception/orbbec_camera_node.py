#!/usr/bin/env python3

"""
奥比中光(Orbbec)深度相机节点
基于文档中的使用方法集成到清扫机器人系统
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Transform, TransformStamped
from tf2_ros import TransformBroadcaster
import subprocess
import threading
import time
import signal
import os

class OrbbecCameraNode(Node):
    def __init__(self):
        super().__init__('orbbec_camera_node')
        
        # 声明参数
        self.declare_parameter('camera_name', 'orbbec')
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_pointcloud', True)
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('depth_fps', 30)
        
        # 获取参数
        self.camera_name = self.get_parameter('camera_name').value
        self.enable_color = self.get_parameter('enable_color').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.enable_pointcloud = self.get_parameter('enable_pointcloud').value
        self.color_width = self.get_parameter('color_width').value
        self.color_height = self.get_parameter('color_height').value
        self.depth_width = self.get_parameter('depth_width').value
        self.depth_height = self.get_parameter('depth_height').value
        self.color_fps = self.get_parameter('color_fps').value
        self.depth_fps = self.get_parameter('depth_fps').value
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 监听相机数据话题
        self.setup_subscriptions()
        
        # 重新映射的发布者
        self.setup_publishers()
        
        # 启动TF发布定时器
        self.tf_timer = self.create_timer(0.1, self.publish_camera_transforms)
        
        # Orbbec相机进程
        self.camera_process = None
        
        # 启动Orbbec相机驱动
        self.start_orbbec_camera()
        
        self.get_logger().info('🎥 奥比中光深度相机节点已启动')
        self.get_logger().info(f'📷 相机名称: {self.camera_name}')
        self.get_logger().info(f'🎨 彩色图像: {self.color_width}x{self.color_height}@{self.color_fps}fps')
        self.get_logger().info(f'📐 深度图像: {self.depth_width}x{self.depth_height}@{self.depth_fps}fps')
    
    def setup_subscriptions(self):
        """设置订阅者"""
        # 订阅Orbbec相机原始话题
        if self.enable_color:
            self.color_sub = self.create_subscription(
                Image, f'/camera/color/image_raw', 
                self.color_callback, 10)
            
            self.color_info_sub = self.create_subscription(
                CameraInfo, f'/camera/color/camera_info', 
                self.color_info_callback, 10)
        
        if self.enable_depth:
            self.depth_sub = self.create_subscription(
                Image, f'/camera/depth/image_raw', 
                self.depth_callback, 10)
            
            self.depth_info_sub = self.create_subscription(
                CameraInfo, f'/camera/depth/camera_info', 
                self.depth_info_callback, 10)
        
        if self.enable_pointcloud:
            self.pointcloud_sub = self.create_subscription(
                PointCloud2, f'/camera/depth/points', 
                self.pointcloud_callback, 10)
    
    def setup_publishers(self):
        """设置发布者（重新映射到清扫机器人命名空间）"""
        if self.enable_color:
            self.color_pub = self.create_publisher(
                Image, f'/cleaning_robot/orbbec_camera/color/image_raw', 10)
            
            self.color_info_pub = self.create_publisher(
                CameraInfo, f'/cleaning_robot/orbbec_camera/color/camera_info', 10)
        
        if self.enable_depth:
            self.depth_pub = self.create_publisher(
                Image, f'/cleaning_robot/orbbec_camera/depth/image_raw', 10)
            
            self.depth_info_pub = self.create_publisher(
                CameraInfo, f'/cleaning_robot/orbbec_camera/depth/camera_info', 10)
        
        if self.enable_pointcloud:
            self.pointcloud_pub = self.create_publisher(
                PointCloud2, f'/cleaning_robot/orbbec_camera/pointcloud', 10)
    
    def start_orbbec_camera(self):
        """启动Orbbec相机驱动"""
        try:
            # 构建启动命令
            cmd = [
                'ros2', 'launch', 'orbbec_camera', 'gemini2.launch.py'
            ]
            
            # 添加参数
            params = [
                f'color_width:={self.color_width}',
                f'color_height:={self.color_height}',
                f'depth_width:={self.depth_width}',
                f'depth_height:={self.depth_height}',
                f'color_fps:={self.color_fps}',
                f'depth_fps:={self.depth_fps}',
                f'enable_color:={str(self.enable_color).lower()}',
                f'enable_depth:={str(self.enable_depth).lower()}',
                f'enable_pointcloud:={str(self.enable_pointcloud).lower()}'
            ]
            
            cmd.extend(params)
            
            self.get_logger().info(f'启动Orbbec相机: {" ".join(cmd)}')
            
            # 启动进程
            self.camera_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            self.get_logger().info('✅ Orbbec相机驱动启动成功')
            
        except Exception as e:
            self.get_logger().error(f'❌ Orbbec相机驱动启动失败: {str(e)}')
            self.get_logger().info('⚠️  请确保已正确安装Orbbec相机包')
    
    def color_callback(self, msg):
        """彩色图像回调"""
        # 更新frame_id为清扫机器人坐标系
        msg.header.frame_id = 'orbbec_color_optical_frame'
        self.color_pub.publish(msg)
    
    def color_info_callback(self, msg):
        """彩色相机信息回调"""
        msg.header.frame_id = 'orbbec_color_optical_frame'
        self.color_info_pub.publish(msg)
    
    def depth_callback(self, msg):
        """深度图像回调"""
        msg.header.frame_id = 'orbbec_depth_optical_frame'
        self.depth_pub.publish(msg)
    
    def depth_info_callback(self, msg):
        """深度相机信息回调"""
        msg.header.frame_id = 'orbbec_depth_optical_frame'
        self.depth_info_pub.publish(msg)
    
    def pointcloud_callback(self, msg):
        """点云回调"""
        msg.header.frame_id = 'orbbec_depth_optical_frame'
        self.pointcloud_pub.publish(msg)
    
    def publish_camera_transforms(self):
        """发布相机坐标变换"""
        try:
            current_time = self.get_clock().now()
            
            # 基础坐标系到相机基座的变换
            base_to_camera = TransformStamped()
            base_to_camera.header.stamp = current_time.to_msg()
            base_to_camera.header.frame_id = 'base_link'
            base_to_camera.child_frame_id = 'orbbec_camera_link'
            
            # 相机安装位置（根据实际安装位置调整）
            base_to_camera.transform.translation.x = 0.2  # 前方20cm
            base_to_camera.transform.translation.y = 0.0  # 中央
            base_to_camera.transform.translation.z = 0.15 # 高15cm
            base_to_camera.transform.rotation.x = 0.0
            base_to_camera.transform.rotation.y = 0.0
            base_to_camera.transform.rotation.z = 0.0
            base_to_camera.transform.rotation.w = 1.0
            
            # 相机基座到彩色光学坐标系
            camera_to_color = TransformStamped()
            camera_to_color.header.stamp = current_time.to_msg()
            camera_to_color.header.frame_id = 'orbbec_camera_link'
            camera_to_color.child_frame_id = 'orbbec_color_optical_frame'
            camera_to_color.transform.translation.x = 0.0
            camera_to_color.transform.translation.y = 0.0
            camera_to_color.transform.translation.z = 0.0
            # 光学坐标系旋转（Z向前，Y向下，X向右）
            camera_to_color.transform.rotation.x = -0.5
            camera_to_color.transform.rotation.y = 0.5
            camera_to_color.transform.rotation.z = -0.5
            camera_to_color.transform.rotation.w = 0.5
            
            # 相机基座到深度光学坐标系
            camera_to_depth = TransformStamped()
            camera_to_depth.header.stamp = current_time.to_msg()
            camera_to_depth.header.frame_id = 'orbbec_camera_link'
            camera_to_depth.child_frame_id = 'orbbec_depth_optical_frame'
            camera_to_depth.transform.translation.x = 0.0
            camera_to_depth.transform.translation.y = 0.0
            camera_to_depth.transform.translation.z = 0.0
            camera_to_depth.transform.rotation.x = -0.5
            camera_to_depth.transform.rotation.y = 0.5
            camera_to_depth.transform.rotation.z = -0.5
            camera_to_depth.transform.rotation.w = 0.5
            
            # 发布变换
            self.tf_broadcaster.sendTransform([
                base_to_camera,
                camera_to_color,
                camera_to_depth
            ])
            
        except Exception as e:
            self.get_logger().error(f'TF发布错误: {str(e)}')
    
    def destroy_node(self):
        """清理资源"""
        if self.camera_process is not None:
            try:
                self.camera_process.terminate()
                self.camera_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.camera_process.kill()
            except Exception as e:
                self.get_logger().error(f'停止相机进程错误: {str(e)}')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = OrbbecCameraNode()
    
    def signal_handler(sig, frame):
        node.get_logger().info('收到停止信号，正在关闭...')
        node.destroy_node()
        rclpy.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 