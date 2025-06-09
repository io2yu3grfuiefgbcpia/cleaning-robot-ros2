#!/usr/bin/env python3

"""
奥比中光深度相机功能测试脚本
测试深度相机的彩色图像、深度图像和点云功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import time
import cv2
from cv_bridge import CvBridge

class OrbbecCameraTest(Node):
    def __init__(self):
        super().__init__('orbbec_camera_test')
        
        self.bridge = CvBridge()
        
        # 订阅深度相机数据
        self.color_sub = self.create_subscription(
            Image, '/cleaning_robot/orbbec_camera/color/image_raw',
            self.color_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/cleaning_robot/orbbec_camera/depth/image_raw',
            self.depth_callback, 10)
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/cleaning_robot/orbbec_camera/pointcloud',
            self.pointcloud_callback, 10)
        
        self.color_info_sub = self.create_subscription(
            CameraInfo, '/cleaning_robot/orbbec_camera/color/camera_info',
            self.color_info_callback, 10)
        
        self.depth_info_sub = self.create_subscription(
            CameraInfo, '/cleaning_robot/orbbec_camera/depth/camera_info',
            self.depth_info_callback, 10)
        
        # 测试状态
        self.color_received = False
        self.depth_received = False
        self.pointcloud_received = False
        self.color_info_received = False
        self.depth_info_received = False
        
        self.color_count = 0
        self.depth_count = 0
        self.pointcloud_count = 0
        
        self.last_color_time = None
        self.last_depth_time = None
        self.last_pointcloud_time = None
        
        # 创建定时器进行测试报告
        self.test_timer = self.create_timer(2.0, self.print_status)
        
        # 测试开始时间
        self.start_time = time.time()
        
        self.get_logger().info("🎥 奥比中光深度相机测试开始")
        self.get_logger().info("等待相机数据...")
    
    def color_callback(self, msg):
        """彩色图像回调"""
        self.color_received = True
        self.color_count += 1
        self.last_color_time = time.time()
        
        if self.color_count % 30 == 0:  # 每30帧显示一次
            self.get_logger().info(
                f"🎨 彩色图像 #{self.color_count}: "
                f"{msg.width}x{msg.height}, 编码: {msg.encoding}"
            )
    
    def depth_callback(self, msg):
        """深度图像回调"""
        self.depth_received = True
        self.depth_count += 1
        self.last_depth_time = time.time()
        
        if self.depth_count % 30 == 0:  # 每30帧显示一次
            try:
                # 分析深度数据
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                valid_depths = depth_image[depth_image > 0]
                
                if len(valid_depths) > 0:
                    min_depth = valid_depths.min() * 0.001  # 转换为米
                    max_depth = valid_depths.max() * 0.001
                    mean_depth = valid_depths.mean() * 0.001
                    
                    self.get_logger().info(
                        f"📐 深度图像 #{self.depth_count}: "
                        f"{msg.width}x{msg.height}, "
                        f"深度范围: {min_depth:.2f}-{max_depth:.2f}m, "
                        f"平均深度: {mean_depth:.2f}m"
                    )
                else:
                    self.get_logger().warning(f"📐 深度图像 #{self.depth_count}: 无有效深度数据")
                    
            except Exception as e:
                self.get_logger().error(f"深度图像处理错误: {str(e)}")
    
    def pointcloud_callback(self, msg):
        """点云回调"""
        self.pointcloud_received = True
        self.pointcloud_count += 1
        self.last_pointcloud_time = time.time()
        
        if self.pointcloud_count % 10 == 0:  # 每10次显示一次
            point_count = msg.height * msg.width
            self.get_logger().info(
                f"☁️ 点云数据 #{self.pointcloud_count}: "
                f"点数: {point_count}, 数据大小: {len(msg.data)} 字节"
            )
    
    def color_info_callback(self, msg):
        """彩色相机信息回调"""
        if not self.color_info_received:
            self.color_info_received = True
            self.get_logger().info(
                f"📷 彩色相机内参: "
                f"分辨率: {msg.width}x{msg.height}, "
                f"焦距: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}"
            )
    
    def depth_info_callback(self, msg):
        """深度相机信息回调"""
        if not self.depth_info_received:
            self.depth_info_received = True
            self.get_logger().info(
                f"📐 深度相机内参: "
                f"分辨率: {msg.width}x{msg.height}, "
                f"焦距: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}"
            )
    
    def print_status(self):
        """打印测试状态"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if elapsed_time < 5:  # 前5秒不检查
            return
        
        status_report = f"\n🔍 Orbbec相机状态报告 (运行时间: {elapsed_time:.1f}s):\n"
        
        # 检查彩色图像
        if self.color_received:
            color_age = current_time - self.last_color_time if self.last_color_time else float('inf')
            fps = self.color_count / elapsed_time
            status_report += f"✅ 彩色图像: 接收 {self.color_count} 帧, FPS: {fps:.1f}, 延迟: {color_age:.1f}s\n"
        else:
            status_report += "❌ 彩色图像: 未接收到数据\n"
        
        # 检查深度图像
        if self.depth_received:
            depth_age = current_time - self.last_depth_time if self.last_depth_time else float('inf')
            fps = self.depth_count / elapsed_time
            status_report += f"✅ 深度图像: 接收 {self.depth_count} 帧, FPS: {fps:.1f}, 延迟: {depth_age:.1f}s\n"
        else:
            status_report += "❌ 深度图像: 未接收到数据\n"
        
        # 检查点云
        if self.pointcloud_received:
            pc_age = current_time - self.last_pointcloud_time if self.last_pointcloud_time else float('inf')
            fps = self.pointcloud_count / elapsed_time
            status_report += f"✅ 点云数据: 接收 {self.pointcloud_count} 次, FPS: {fps:.1f}, 延迟: {pc_age:.1f}s\n"
        else:
            status_report += "❌ 点云数据: 未接收到数据\n"
        
        # 相机信息
        if self.color_info_received and self.depth_info_received:
            status_report += "✅ 相机标定信息: 已接收\n"
        else:
            status_report += "⚠️  相机标定信息: 部分缺失\n"
        
        # 整体状态评估
        if all([self.color_received, self.depth_received, self.pointcloud_received]):
            status_report += "🎯 整体状态: Orbbec深度相机工作正常！\n"
        else:
            status_report += "🚨 整体状态: Orbbec深度相机存在问题！\n"
            status_report += "   请检查:\n"
            status_report += "   1. 相机是否正确连接\n"
            status_report += "   2. Orbbec驱动是否启动\n"
            status_report += "   3. USB权限设置\n"
            status_report += "   4. udev规则配置\n"
        
        self.get_logger().info(status_report)

def main():
    rclpy.init()
    
    print("🎥 奥比中光深度相机功能测试")
    print("=" * 50)
    print("此测试将验证以下功能:")
    print("1. ✅ 彩色图像数据接收")
    print("2. ✅ 深度图像数据接收")
    print("3. ✅ 点云数据生成")
    print("4. ✅ 相机标定信息")
    print("5. ✅ 实时数据刷新率")
    print("=" * 50)
    print("按 Ctrl+C 停止测试")
    print()
    
    try:
        test_node = OrbbecCameraTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\n🛑 测试被用户停止")
    finally:
        # 清理
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()
        
        print("\n📊 测试总结:")
        if 'test_node' in locals():
            print(f"彩色图像接收次数: {test_node.color_count}")
            print(f"深度图像接收次数: {test_node.depth_count}")
            print(f"点云数据接收次数: {test_node.pointcloud_count}")
            
            if all([test_node.color_received, test_node.depth_received, test_node.pointcloud_received]):
                print("🎉 Orbbec深度相机测试成功！")
            else:
                print("⚠️  Orbbec深度相机需要检查配置")
        print("测试结束")

if __name__ == '__main__':
    main() 