#!/usr/bin/env python3

"""
镭神N10P激光雷达网络版集成测试脚本
用于验证以太网连接的N10P激光雷达在清扫机器人系统中的功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import time
import sys
import subprocess
import socket

class N10PNetworkIntegrationTest(Node):
    def __init__(self):
        super().__init__('n10p_network_integration_test')
        
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
        
        # 网络配置
        self.lidar_ip = "192.168.1.200"
        self.host_ip = "192.168.1.102"
        self.msop_port = 2368
        self.difop_port = 2369
        
        # 创建定时器进行测试
        self.test_timer = self.create_timer(1.0, self.run_tests)
        self.network_timer = self.create_timer(5.0, self.test_network)
        self.motion_timer = self.create_timer(10.0, self.test_motion)
        
        # 测试开始时间
        self.start_time = time.time()
        
        self.get_logger().info("🌐 N10P激光雷达网络版集成测试已启动")
        self.get_logger().info(f"📍 激光雷达IP: {self.lidar_ip}")
        self.get_logger().info(f"📍 本机IP: {self.host_ip}")
        self.get_logger().info("正在等待激光雷达数据...")
        
        # 进行网络连接测试
        self.test_network_connection()
    
    def test_network_connection(self):
        """测试网络连接状态"""
        self.get_logger().info("🔍 测试网络连接...")
        
        # 测试ping连接
        try:
            result = subprocess.run(['ping', '-c', '1', self.lidar_ip], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.get_logger().info(f"✅ 成功ping通激光雷达 {self.lidar_ip}")
            else:
                self.get_logger().warn(f"❌ 无法ping通激光雷达 {self.lidar_ip}")
        except Exception as e:
            self.get_logger().warn(f"⚠️  Ping测试异常: {e}")
        
        # 测试UDP端口连接
        self.test_udp_port(self.msop_port, "数据端口")
        self.test_udp_port(self.difop_port, "控制端口")
    
    def test_udp_port(self, port, port_name):
        """测试UDP端口连接"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(2)
            sock.bind(('', port))
            sock.close()
            self.get_logger().info(f"✅ {port_name} {port} 可用")
        except Exception as e:
            self.get_logger().warn(f"⚠️  {port_name} {port} 测试失败: {e}")
    
    def scan_callback(self, msg):
        """激光扫描数据回调"""
        self.scan_received = True
        self.last_scan_time = time.time()
        self.scan_count += 1
        
        # 分析扫描数据
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        
        if self.scan_count % 10 == 0:  # 每10次显示一次
            self.get_logger().info(
                f"📡 网络激光扫描数据 #{self.scan_count}:\n"
                f"   • 有效点数: {len(valid_ranges)}/{len(msg.ranges)}\n"
                f"   • 扫描范围: {msg.angle_min:.2f} - {msg.angle_max:.2f} 弧度\n"
                f"   • 距离范围: {msg.range_min:.2f} - {msg.range_max:.2f} 米\n"
                f"   • 最近距离: {min(valid_ranges):.2f}m" if valid_ranges else "   • 无有效数据\n"
                f"   • 数据来源: 网络通讯 (UDP:{self.msop_port})"
            )
    
    def pointcloud_callback(self, msg):
        """点云数据回调"""
        self.pointcloud_received = True
        self.last_pointcloud_time = time.time()
        self.pointcloud_count += 1
        
        if self.pointcloud_count % 10 == 0:  # 每10次显示一次
            self.get_logger().info(
                f"☁️  网络点云数据 #{self.pointcloud_count}:\n"
                f"   • 数据大小: {len(msg.data)} 字节\n"
                f"   • 高度: {msg.height}, 宽度: {msg.width}\n"
                f"   • 点数: {msg.height * msg.width}\n"
                f"   • 传输方式: 以太网"
            )
    
    def test_network(self):
        """测试网络性能"""
        if not (self.scan_received and self.pointcloud_received):
            return
        
        self.get_logger().info("🌐 网络性能测试...")
        
        # 测试网络延迟
        try:
            result = subprocess.run(['ping', '-c', '1', self.lidar_ip], 
                                  capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                # 解析ping延迟
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'time=' in line:
                        latency = line.split('time=')[1].split(' ')[0]
                        self.get_logger().info(f"📊 网络延迟: {latency} ms")
                        break
        except Exception as e:
            self.get_logger().warn(f"⚠️  延迟测试失败: {e}")
        
        # 检查网络流量（如果iftop可用）
        try:
            result = subprocess.run(['which', 'iftop'], capture_output=True)
            if result.returncode == 0:
                self.get_logger().info("💡 可使用 'sudo iftop -i eth0' 监控网络流量")
        except:
            pass
    
    def run_tests(self):
        """运行周期性测试"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # 状态报告
        if elapsed_time > 5:  # 5秒后开始检查
            status_report = f"\n🔍 N10P网络版状态报告 (运行时间: {elapsed_time:.1f}s):\n"
            
            # 检查激光扫描数据
            if self.scan_received:
                scan_age = current_time - self.last_scan_time if self.last_scan_time else float('inf')
                status_report += f"✅ 网络激光扫描: 接收到 {self.scan_count} 次数据, "
                status_report += f"最新数据延迟: {scan_age:.1f}s\n"
                
                # 计算数据接收频率
                if elapsed_time > 10:
                    scan_freq = self.scan_count / elapsed_time
                    status_report += f"📊 扫描频率: {scan_freq:.1f} Hz\n"
            else:
                status_report += "❌ 网络激光扫描: 未接收到数据\n"
            
            # 检查点云数据
            if self.pointcloud_received:
                pc_age = current_time - self.last_pointcloud_time if self.last_pointcloud_time else float('inf')
                status_report += f"✅ 网络点云数据: 接收到 {self.pointcloud_count} 次数据, "
                status_report += f"最新数据延迟: {pc_age:.1f}s\n"
                
                # 计算点云接收频率
                if elapsed_time > 10:
                    pc_freq = self.pointcloud_count / elapsed_time
                    status_report += f"📊 点云频率: {pc_freq:.1f} Hz\n"
            else:
                status_report += "❌ 网络点云数据: 未接收到数据\n"
            
            # 整体状态评估
            if self.scan_received and self.pointcloud_received:
                status_report += "🎯 整体状态: N10P网络版激光雷达工作正常！\n"
                status_report += f"🌐 网络连接: 以太网连接稳定 ({self.lidar_ip})\n"
            elif self.scan_received:
                status_report += "⚠️  整体状态: 激光扫描正常，点云数据异常\n"
                status_report += "🔧 建议检查点云发布配置\n"
            elif self.pointcloud_received:
                status_report += "⚠️  整体状态: 点云数据正常，激光扫描异常\n"
                status_report += "🔧 建议检查扫描话题配置\n"
            else:
                status_report += "🚨 整体状态: N10P网络版激光雷达未工作！\n"
                status_report += "   请检查:\n"
                status_report += f"   1. 激光雷达网络连接 ({self.lidar_ip})\n"
                status_report += "   2. 网络配置和路由\n"
                status_report += f"   3. UDP端口 {self.msop_port}/{self.difop_port}\n"
                status_report += "   4. 防火墙设置\n"
                status_report += "   5. 激光雷达驱动节点\n"
            
            self.get_logger().info(status_report)
    
    def test_motion(self):
        """测试机器人运动（简单的转圈测试）"""
        if not (self.scan_received and self.pointcloud_received):
            return
        
        self.get_logger().info("🔄 执行网络版转圈测试...")
        
        # 创建转圈命令
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3  # 慢速转动
        
        # 发布运动命令
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("🌐 通过网络控制机器人运动...")
        
        # 3秒后停止
        def stop_motion():
            stop_twist = Twist()
            self.cmd_vel_publisher.publish(stop_twist)
            self.get_logger().info("⏹️  停止网络控制运动")
        
        self.create_timer(3.0, stop_motion)

def main():
    rclpy.init()
    
    print("🌐 镭神N10P激光雷达网络版集成测试")
    print("=" * 60)
    print("此测试将验证以下网络功能:")
    print("1. ✅ N10P激光雷达网络连接")
    print("2. ✅ UDP数据包接收")
    print("3. ✅ 激光扫描数据传输")
    print("4. ✅ 点云数据生成")
    print("5. ✅ 网络延迟和频率测试")
    print("6. ✅ 基础运动控制")
    print("=" * 60)
    print("网络配置:")
    print("• 激光雷达IP: 192.168.1.200")
    print("• 本机IP: 192.168.1.102")
    print("• 数据端口: 2368 (UDP)")
    print("• 控制端口: 2369 (UDP)")
    print("=" * 60)
    print("按 Ctrl+C 停止测试")
    print()
    
    try:
        test_node = N10PNetworkIntegrationTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\n🛑 测试被用户停止")
    finally:
        # 清理
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()
        
        print("\n📊 网络版测试总结:")
        if 'test_node' in locals():
            print(f"激光扫描数据接收次数: {test_node.scan_count}")
            print(f"点云数据接收次数: {test_node.pointcloud_count}")
            if test_node.scan_received and test_node.pointcloud_received:
                print("🎉 N10P网络版激光雷达集成测试成功！")
                print("🌐 以太网连接工作正常")
            else:
                print("⚠️  N10P网络版激光雷达集成需要检查")
                print("🔧 请检查网络配置和连接")
        print("测试结束")

if __name__ == '__main__':
    main() 