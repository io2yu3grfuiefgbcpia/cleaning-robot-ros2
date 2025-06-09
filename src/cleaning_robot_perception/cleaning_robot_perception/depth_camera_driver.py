#!/usr/bin/env python3

"""
深度相机驱动程序
支持多种深度相机，包括USB相机、网络相机、RealSense等
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import threading
import time

class DepthCameraDriver(Node):
    def __init__(self):
        super().__init__('depth_camera_driver')
        
        self.bridge = CvBridge()
        
        # 获取参数
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('depth_camera_device', 1)  # 深度相机设备
        self.declare_parameter('use_realsense', False)    # 是否使用RealSense
        self.declare_parameter('use_network_camera', False)  # 是否使用网络相机
        self.declare_parameter('network_camera_url', '')  # 网络相机URL
        
        # 获取参数值
        self.camera_device = self.get_parameter('camera_device').value
        self.width = self.get_parameter('camera_width').value
        self.height = self.get_parameter('camera_height').value
        self.fps = self.get_parameter('camera_fps').value
        self.depth_device = self.get_parameter('depth_camera_device').value
        self.use_realsense = self.get_parameter('use_realsense').value
        self.use_network = self.get_parameter('use_network_camera').value
        self.network_url = self.get_parameter('network_camera_url').value
        
        # 发布者
        self.color_pub = self.create_publisher(
            Image, '/cleaning_robot/depth_camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(
            Image, '/cleaning_robot/depth_camera/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(
            CameraInfo, '/cleaning_robot/depth_camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, '/cleaning_robot/depth_camera/depth/camera_info', 10)
        
        # 相机对象
        self.color_camera = None
        self.depth_camera = None
        self.realsense_pipeline = None
        
        # 运行状态
        self.running = True
        
        # 初始化相机
        self.init_cameras()
        
        # 启动相机线程
        self.camera_thread = threading.Thread(target=self.camera_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
        # 创建定时器发布相机信息
        self.info_timer = self.create_timer(1.0, self.publish_camera_info)
        
        self.get_logger().info('🎥 深度相机驱动程序已启动')
    
    def init_cameras(self):
        """初始化相机"""
        try:
            if self.use_realsense:
                self.init_realsense()
            elif self.use_network:
                self.init_network_camera()
            else:
                self.init_usb_cameras()
                
        except Exception as e:
            self.get_logger().error(f'相机初始化失败: {str(e)}')
    
    def init_realsense(self):
        """初始化RealSense相机"""
        try:
            import pyrealsense2 as rs
            
            # 创建管道
            self.realsense_pipeline = rs.pipeline()
            config = rs.config()
            
            # 配置流
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            
            # 启动管道
            self.realsense_pipeline.start(config)
            
            self.get_logger().info('✅ RealSense相机初始化成功')
            
        except ImportError:
            self.get_logger().error('❌ 未安装pyrealsense2库，请安装: pip install pyrealsense2')
            self.init_usb_cameras()
        except Exception as e:
            self.get_logger().error(f'❌ RealSense初始化失败: {str(e)}')
            self.init_usb_cameras()
    
    def init_network_camera(self):
        """初始化网络相机"""
        try:
            if not self.network_url:
                self.get_logger().error('网络相机URL未设置')
                return
            
            self.color_camera = cv2.VideoCapture(self.network_url)
            
            if self.color_camera.isOpened():
                self.get_logger().info(f'✅ 网络相机连接成功: {self.network_url}')
            else:
                self.get_logger().error(f'❌ 网络相机连接失败: {self.network_url}')
                
        except Exception as e:
            self.get_logger().error(f'网络相机初始化失败: {str(e)}')
    
    def init_usb_cameras(self):
        """初始化USB相机"""
        try:
            # 初始化彩色相机
            self.color_camera = cv2.VideoCapture(self.camera_device)
            if self.color_camera.isOpened():
                self.color_camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.color_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.color_camera.set(cv2.CAP_PROP_FPS, self.fps)
                self.get_logger().info(f'✅ 彩色相机初始化成功 (设备{self.camera_device})')
            else:
                self.get_logger().error(f'❌ 彩色相机初始化失败 (设备{self.camera_device})')
            
            # 尝试初始化深度相机（如果有单独的深度相机设备）
            if self.depth_device != self.camera_device:
                self.depth_camera = cv2.VideoCapture(self.depth_device)
                if self.depth_camera.isOpened():
                    self.depth_camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    self.depth_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    self.depth_camera.set(cv2.CAP_PROP_FPS, self.fps)
                    self.get_logger().info(f'✅ 深度相机初始化成功 (设备{self.depth_device})')
                else:
                    self.get_logger().warning(f'⚠️  深度相机初始化失败 (设备{self.depth_device})')
                    self.depth_camera = None
            
        except Exception as e:
            self.get_logger().error(f'USB相机初始化失败: {str(e)}')
    
    def camera_loop(self):
        """相机捕获循环"""
        while self.running and rclpy.ok():
            try:
                if self.use_realsense and self.realsense_pipeline:
                    self.capture_realsense()
                elif self.color_camera is not None:
                    self.capture_usb()
                else:
                    time.sleep(0.1)
                    continue
                
                # 控制帧率
                time.sleep(1.0 / self.fps)
                
            except Exception as e:
                self.get_logger().error(f'相机捕获错误: {str(e)}')
                time.sleep(0.1)
    
    def capture_realsense(self):
        """捕获RealSense相机数据"""
        try:
            import pyrealsense2 as rs
            
            # 等待帧
            frames = self.realsense_pipeline.wait_for_frames()
            
            # 获取彩色和深度帧
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if color_frame and depth_frame:
                # 转换为numpy数组
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # 发布图像
                self.publish_color_image(color_image)
                self.publish_depth_image(depth_image)
                
        except Exception as e:
            self.get_logger().error(f'RealSense捕获错误: {str(e)}')
    
    def capture_usb(self):
        """捕获USB相机数据"""
        try:
            # 捕获彩色图像
            if self.color_camera is not None:
                ret, color_frame = self.color_camera.read()
                if ret:
                    self.publish_color_image(color_frame)
            
            # 捕获深度图像（如果有单独的深度相机）
            if self.depth_camera is not None:
                ret, depth_frame = self.depth_camera.read()
                if ret:
                    # 将彩色图像转换为深度图像（简单模拟）
                    if len(depth_frame.shape) == 3:
                        depth_gray = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)
                        # 简单的深度估计（基于亮度）
                        depth_simulated = self.simulate_depth_from_gray(depth_gray)
                        self.publish_depth_image(depth_simulated)
            else:
                # 如果没有深度相机，从彩色图像模拟深度
                if self.color_camera is not None:
                    ret, frame = self.color_camera.read()
                    if ret:
                        depth_simulated = self.simulate_depth_from_color(frame)
                        self.publish_depth_image(depth_simulated)
                        
        except Exception as e:
            self.get_logger().error(f'USB相机捕获错误: {str(e)}')
    
    def simulate_depth_from_color(self, color_image):
        """从彩色图像模拟深度信息"""
        # 转换为灰度图
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        return self.simulate_depth_from_gray(gray)
    
    def simulate_depth_from_gray(self, gray_image):
        """从灰度图模拟深度信息"""
        # 简单的深度模拟：使用边缘检测和亮度信息
        edges = cv2.Canny(gray_image, 50, 150)
        
        # 基于亮度和边缘生成深度图
        # 亮的区域较近，暗的区域较远
        depth = 255 - gray_image
        
        # 边缘区域设为较近的距离
        depth[edges > 0] = np.maximum(depth[edges > 0], 200)
        
        # 转换为16位深度图（模拟毫米值）
        depth_16bit = (depth.astype(np.float32) / 255.0 * 5000).astype(np.uint16)  # 0-5米范围
        
        return depth_16bit
    
    def publish_color_image(self, image):
        """发布彩色图像"""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "depth_camera_color_optical_frame"
            self.color_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'发布彩色图像错误: {str(e)}')
    
    def publish_depth_image(self, image):
        """发布深度图像"""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, "16UC1")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "depth_camera_depth_optical_frame"
            self.depth_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'发布深度图像错误: {str(e)}')
    
    def publish_camera_info(self):
        """发布相机标定信息"""
        try:
            # 创建相机信息消息
            camera_info = CameraInfo()
            camera_info.header.stamp = self.get_clock().now().to_msg()
            camera_info.width = self.width
            camera_info.height = self.height
            
            # 假设的相机内参（实际使用时需要通过标定获得）
            fx = fy = self.width * 0.8  # 简单估计
            cx = self.width / 2.0
            cy = self.height / 2.0
            
            camera_info.k = [
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0
            ]
            
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # 假设无畸变
            
            camera_info.r = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]
            
            camera_info.p = [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            # 发布彩色相机信息
            camera_info.header.frame_id = "depth_camera_color_optical_frame"
            self.color_info_pub.publish(camera_info)
            
            # 发布深度相机信息
            camera_info.header.frame_id = "depth_camera_depth_optical_frame"
            self.depth_info_pub.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f'发布相机信息错误: {str(e)}')
    
    def destroy_node(self):
        """清理资源"""
        self.running = False
        
        if self.color_camera is not None:
            self.color_camera.release()
        
        if self.depth_camera is not None:
            self.depth_camera.release()
        
        if self.realsense_pipeline is not None:
            try:
                self.realsense_pipeline.stop()
            except:
                pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    driver = DepthCameraDriver()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 