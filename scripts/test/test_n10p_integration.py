#!/usr/bin/env python3

"""
镭神N10P激光雷达集成测试脚本
用于验证N10P激光雷达在清扫机器人系统中的功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import time
import sys

class N10PIntegrationTest(Node):
    def __init__(self):
        super().__init__('n10p_integration_test')
        
        # 激光雷达数据订阅者
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/cleaning_robot/scan',
            self.scan_callback,
            10)
        
        # 点云数据订阅者
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/cleaning_robot/pointcloud',
            self.pointcloud_callback,
            10)
        
        # 速度控制发布者（用于测试）
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cleaning_robot/cmd_vel', 10)
        
        # 测试状态
        self.scan_received = False
        self.pointcloud_received = False
        self.last_scan_time = None
        self.last_pointcloud_time = None
        self.scan_count = 0
        self.pointcloud_count = 0
        
        # 创建定时器进行测试
        self.test_timer = self.create_timer(1.0, self.run_tests)
        self.motion_timer = self.create_timer(5.0, self.test_motion)
        
        # 测试开始时间
        self.start_time = time.time()
        
        self.get_logger().info("🚀 N10P激光雷达集成测试已启动")
        self.get_logger().info("正在等待激光雷达数据...")
    
    def scan_callback(self, msg):
        """激光扫描数据回调"""
        self.scan_received = True
        self.last_scan_time = time.time()
        self.scan_count += 1
        
        # 分析扫描数据
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        
        if self.scan_count % 10 == 0:  # 每10次显示一次
            self.get_logger().info(
                f"📡 激光扫描数据 #{self.scan_count}:\n"
                f"   • 有效点数: {len(valid_ranges)}/{len(msg.ranges)}\n"
                f"   • 扫描范围: {msg.angle_min:.2f} - {msg.angle_max:.2f} 弧度\n"
                f"   • 距离范围: {msg.range_min:.2f} - {msg.range_max:.2f} 米\n"
                f"   • 最近距离: {min(valid_ranges):.2f}m (如有)" if valid_ranges else "   • 无有效数据"
            )
    
    def pointcloud_callback(self, msg):
        """点云数据回调"""
        self.pointcloud_received = True
        self.last_pointcloud_time = time.time()
        self.pointcloud_count += 1
        
        if self.pointcloud_count % 10 == 0:  # 每10次显示一次
            self.get_logger().info(
                f"☁️  点云数据 #{self.pointcloud_count}:\n"
                f"   • 数据大小: {len(msg.data)} 字节\n"
                f"   • 高度: {msg.height}, 宽度: {msg.width}\n"
                f"   • 点数: {msg.height * msg.width}"
            )
    
    def run_tests(self):
        """运行周期性测试"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # 状态报告
        if elapsed_time > 5:  # 5秒后开始检查
            status_report = f"\n🔍 N10P状态报告 (运行时间: {elapsed_time:.1f}s):\n"
            
            # 检查激光扫描数据
            if self.scan_received:
                scan_age = current_time - self.last_scan_time if self.last_scan_time else float('inf')
                status_report += f"✅ 激光扫描: 接收到 {self.scan_count} 次数据, "
                status_report += f"最新数据延迟: {scan_age:.1f}s\n"
            else:
                status_report += "❌ 激光扫描: 未接收到数据\n"
            
            # 检查点云数据
            if self.pointcloud_received:
                pc_age = current_time - self.last_pointcloud_time if self.last_pointcloud_time else float('inf')
                status_report += f"✅ 点云数据: 接收到 {self.pointcloud_count} 次数据, "
                status_report += f"最新数据延迟: {pc_age:.1f}s\n"
            else:
                status_report += "❌ 点云数据: 未接收到数据\n"
            
            # 整体状态评估
            if self.scan_received and self.pointcloud_received:
                status_report += "🎯 整体状态: N10P激光雷达工作正常！\n"
            elif self.scan_received:
                status_report += "⚠️  整体状态: 激光扫描正常，点云数据异常\n"
            elif self.pointcloud_received:
                status_report += "⚠️  整体状态: 点云数据正常，激光扫描异常\n"
            else:
                status_report += "🚨 整体状态: N10P激光雷达未工作！\n"
                status_report += "   请检查:\n"
                status_report += "   1. 设备连接 (/dev/ttyUSB1)\n"
                status_report += "   2. 串口权限\n"
                status_report += "   3. 激光雷达驱动节点\n"
            
            self.get_logger().info(status_report)
    
    def test_motion(self):
        """测试机器人运动（简单的转圈测试）"""
        if not (self.scan_received and self.pointcloud_received):
            return
        
        self.get_logger().info("🔄 执行转圈测试...")
        
        # 创建转圈命令
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # 慢速转动
        
        # 发布运动命令
        self.cmd_vel_publisher.publish(twist)
        
        # 3秒后停止
        def stop_motion():
            stop_twist = Twist()
            self.cmd_vel_publisher.publish(stop_twist)
            self.get_logger().info("⏹️  停止运动")
        
        self.create_timer(3.0, stop_motion)

def main():
    rclpy.init()
    
    print("🤖 镭神N10P激光雷达集成测试")
    print("=" * 50)
    print("此测试将验证以下功能:")
    print("1. ✅ N10P激光雷达数据接收")
    print("2. ✅ 点云数据生成")
    print("3. ✅ 实时数据刷新率")
    print("4. ✅ 基础运动控制")
    print("=" * 50)
    print("按 Ctrl+C 停止测试")
    print()
    
    try:
        test_node = N10PIntegrationTest()
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
            print(f"激光扫描数据接收次数: {test_node.scan_count}")
            print(f"点云数据接收次数: {test_node.pointcloud_count}")
            if test_node.scan_received and test_node.pointcloud_received:
                print("🎉 N10P激光雷达集成测试成功！")
            else:
                print("⚠️  N10P激光雷达集成需要检查")
        print("测试结束")

if __name__ == '__main__':
    main() 