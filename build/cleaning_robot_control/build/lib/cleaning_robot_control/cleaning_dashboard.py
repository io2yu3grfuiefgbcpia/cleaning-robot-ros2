#!/usr/bin/env python3
"""
清扫机器人统一可视化控制界面
功能：
1. 显示SLAM建图实时效果
2. 显示摄像头数据（深度图/彩色图）
3. 显示规划路径
4. 显示机器人当前位置和状态
5. 提供交互式控制按钮
"""

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from matplotlib.patches import Circle, Arrow, FancyArrowPatch
import matplotlib.patches as mpatches
from threading import Thread, Lock
import math
import time

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float32


class CleaningDashboard(Node):
    def __init__(self):
        super().__init__('cleaning_dashboard')
        
        # 数据锁
        self.data_lock = Lock()
        
        # 数据存储
        self.map_data = None
        self.map_info = None
        self.robot_pose = None
        self.robot_odom = None
        self.laser_scan = None
        self.cleaning_path = None
        self.camera_image = None
        self.depth_image = None
        self.cleaning_status = False
        self.cleaning_progress = 0.0
        
        # 已清扫区域
        self.cleaned_cells = set()
        
        # ROS2订阅者
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/cleaning_robot/pose', self.pose_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/cleaning_robot/scan', self.laser_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/cleaning_robot/cleaning_path', self.path_callback, 10)
        self.status_sub = self.create_subscription(
            Bool, '/cleaning_robot/cleaning_status', self.status_callback, 10)
        
        # 摄像头订阅
        self.color_sub = self.create_subscription(
            Image, '/cleaning_robot/camera/left/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/cleaning_robot/stereo/depth', self.depth_callback, 10)
        
        # ROS2发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cleaning_robot/cmd_vel', 10)
        self.start_cleaning_pub = self.create_publisher(Bool, '/cleaning_robot/start_cleaning', 10)
        
        # 界面状态
        self.start_time = time.time()
        
        self.get_logger().info('清扫机器人控制面板已启动')
    
    def map_callback(self, msg):
        with self.data_lock:
            self.map_info = msg.info
            self.map_data = np.array(msg.data).reshape(
                (msg.info.height, msg.info.width))
    
    def pose_callback(self, msg):
        with self.data_lock:
            self.robot_pose = msg.pose
    
    def odom_callback(self, msg):
        with self.data_lock:
            self.robot_odom = msg
            # 如果没有pose数据，使用odom
            if self.robot_pose is None:
                self.robot_pose = msg.pose.pose
    
    def laser_callback(self, msg):
        with self.data_lock:
            self.laser_scan = msg
    
    def path_callback(self, msg):
        with self.data_lock:
            self.cleaning_path = msg
    
    def status_callback(self, msg):
        with self.data_lock:
            self.cleaning_status = msg.data
    
    def color_callback(self, msg):
        with self.data_lock:
            # 简单的ROS Image到numpy转换
            if msg.encoding == 'bgr8' or msg.encoding == 'rgb8':
                self.camera_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 3))
            elif msg.encoding == 'mono8':
                self.camera_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width))
    
    def depth_callback(self, msg):
        with self.data_lock:
            if msg.encoding == '16UC1':
                self.depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    (msg.height, msg.width))
            elif msg.encoding == '32FC1':
                self.depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape(
                    (msg.height, msg.width))
    
    def start_cleaning_clicked(self, event):
        """开始清扫按钮回调"""
        msg = Bool()
        msg.data = True
        self.start_cleaning_pub.publish(msg)
        self.get_logger().info('发送开始清扫命令')
    
    def stop_cleaning_clicked(self, event):
        """停止清扫按钮回调"""
        msg = Bool()
        msg.data = False
        self.start_cleaning_pub.publish(msg)
        # 停止机器人
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('发送停止清扫命令')
    
    def emergency_stop_clicked(self, event):
        """紧急停止按钮回调"""
        # 发送多次停止命令确保机器人停止
        for _ in range(5):
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
        msg = Bool()
        msg.data = False
        self.start_cleaning_pub.publish(msg)
        self.get_logger().warn('紧急停止！')


def create_dashboard(node):
    """创建可视化界面"""
    # 创建图形窗口
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('🤖 清扫机器人控制面板', fontsize=16, fontweight='bold', color='#00ff88')
    
    # 创建网格布局
    gs = fig.add_gridspec(3, 4, hspace=0.3, wspace=0.3,
                          left=0.05, right=0.95, top=0.92, bottom=0.12)
    
    # 1. 主地图视图 (左上，大区域)
    ax_map = fig.add_subplot(gs[0:2, 0:2])
    ax_map.set_title('🗺️ SLAM地图 & 清扫路径', fontsize=12, color='#00ffff')
    ax_map.set_facecolor('#1a1a2e')
    ax_map.set_aspect('equal')
    
    # 2. 激光雷达视图 (右上)
    ax_lidar = fig.add_subplot(gs[0, 2], projection='polar')
    ax_lidar.set_title('📡 激光雷达', fontsize=11, color='#ff6b6b')
    ax_lidar.set_facecolor('#1a1a2e')
    
    # 3. 摄像头视图 (右上2)
    ax_camera = fig.add_subplot(gs[0, 3])
    ax_camera.set_title('📷 摄像头', fontsize=11, color='#ffd93d')
    ax_camera.set_facecolor('#1a1a2e')
    ax_camera.axis('off')
    
    # 4. 深度图视图 (右中)
    ax_depth = fig.add_subplot(gs[1, 2])
    ax_depth.set_title('🌈 深度图', fontsize=11, color='#6bcb77')
    ax_depth.set_facecolor('#1a1a2e')
    ax_depth.axis('off')
    
    # 5. 状态信息面板 (右中2)
    ax_status = fig.add_subplot(gs[1, 3])
    ax_status.set_title('📊 系统状态', fontsize=11, color='#4d96ff')
    ax_status.set_facecolor('#1a1a2e')
    ax_status.axis('off')
    
    # 6. 路径规划视图 (左下)
    ax_path = fig.add_subplot(gs[2, 0:2])
    ax_path.set_title('🛤️ 路径规划详情', fontsize=11, color='#ff9f43')
    ax_path.set_facecolor('#1a1a2e')
    
    # 7. 控制按钮区域 (右下)
    ax_controls = fig.add_subplot(gs[2, 2:4])
    ax_controls.set_title('🎮 控制面板', fontsize=11, color='#a55eea')
    ax_controls.set_facecolor('#1a1a2e')
    ax_controls.axis('off')
    
    # 添加控制按钮
    btn_start_ax = fig.add_axes([0.55, 0.02, 0.12, 0.05])
    btn_start = Button(btn_start_ax, '▶ 开始清扫', color='#2ecc71', hovercolor='#27ae60')
    btn_start.label.set_fontsize(10)
    btn_start.on_clicked(node.start_cleaning_clicked)
    
    btn_stop_ax = fig.add_axes([0.68, 0.02, 0.12, 0.05])
    btn_stop = Button(btn_stop_ax, '⏹ 停止清扫', color='#e74c3c', hovercolor='#c0392b')
    btn_stop.label.set_fontsize(10)
    btn_stop.on_clicked(node.stop_cleaning_clicked)
    
    btn_emergency_ax = fig.add_axes([0.81, 0.02, 0.12, 0.05])
    btn_emergency = Button(btn_emergency_ax, '🚨 紧急停止', color='#9b59b6', hovercolor='#8e44ad')
    btn_emergency.label.set_fontsize(10)
    btn_emergency.on_clicked(node.emergency_stop_clicked)
    
    # 图形元素
    map_img = None
    lidar_plot = None
    camera_img = None
    depth_img = None
    robot_marker = None
    path_line = None
    
    def update(frame):
        nonlocal map_img, lidar_plot, camera_img, depth_img, robot_marker, path_line
        
        with node.data_lock:
            # 1. 更新地图
            ax_map.clear()
            ax_map.set_title('🗺️ SLAM地图 & 清扫路径', fontsize=12, color='#00ffff')
            ax_map.set_facecolor('#1a1a2e')
            
            if node.map_data is not None and node.map_info is not None:
                # 显示地图
                display_map = np.copy(node.map_data).astype(float)
                display_map[display_map == -1] = 50  # 未知区域
                display_map[display_map == 0] = 100   # 自由空间
                display_map[display_map == 100] = 0   # 障碍物
                
                extent = [
                    node.map_info.origin.position.x,
                    node.map_info.origin.position.x + node.map_info.width * node.map_info.resolution,
                    node.map_info.origin.position.y,
                    node.map_info.origin.position.y + node.map_info.height * node.map_info.resolution
                ]
                
                ax_map.imshow(display_map, cmap='RdYlGn', origin='lower', extent=extent,
                            alpha=0.8, vmin=0, vmax=100)
            
            # 绘制清扫路径
            has_path_label = False
            if node.cleaning_path is not None and len(node.cleaning_path.poses) > 0:
                path_x = [p.pose.position.x for p in node.cleaning_path.poses]
                path_y = [p.pose.position.y for p in node.cleaning_path.poses]
                ax_map.plot(path_x, path_y, 'c-', linewidth=1.5, alpha=0.7, label='规划路径')
                ax_map.scatter(path_x[::10], path_y[::10], c='cyan', s=5, alpha=0.5)
                has_path_label = True
            
            # 绘制机器人位置
            if node.robot_pose is not None:
                robot_x = node.robot_pose.position.x
                robot_y = node.robot_pose.position.y
                
                # 计算航向角
                q = node.robot_pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                
                # 绘制机器人
                robot_circle = Circle((robot_x, robot_y), 0.15, color='#ff6b6b', alpha=0.8)
                ax_map.add_patch(robot_circle)
                
                # 绘制方向箭头
                arrow_len = 0.3
                dx = arrow_len * math.cos(yaw)
                dy = arrow_len * math.sin(yaw)
                ax_map.arrow(robot_x, robot_y, dx, dy, head_width=0.1, head_length=0.05,
                           fc='yellow', ec='yellow')
            
            ax_map.set_xlabel('X (米)', color='white')
            ax_map.set_ylabel('Y (米)', color='white')
            ax_map.grid(True, alpha=0.3, color='gray')
            # 只有当有带label的artists时才显示图例
            if has_path_label:
                ax_map.legend(loc='upper right', fontsize=8)
            
            # 2. 更新激光雷达
            ax_lidar.clear()
            ax_lidar.set_title('📡 激光雷达', fontsize=11, color='#ff6b6b')
            ax_lidar.set_facecolor('#1a1a2e')
            
            if node.laser_scan is not None:
                angles = np.linspace(node.laser_scan.angle_min,
                                   node.laser_scan.angle_max,
                                   len(node.laser_scan.ranges))
                ranges = np.array(node.laser_scan.ranges)
                ranges = np.clip(ranges, 0, node.laser_scan.range_max)
                
                ax_lidar.scatter(angles, ranges, c=ranges, cmap='plasma', s=2, alpha=0.8)
                ax_lidar.set_ylim(0, min(10, node.laser_scan.range_max))
            else:
                # 显示等待数据
                ax_lidar.text(0, 0.5, '等待数据...', ha='center', va='center',
                            fontsize=10, color='gray')
            
            # 3. 更新摄像头图像
            ax_camera.clear()
            ax_camera.set_title('📷 摄像头', fontsize=11, color='#ffd93d')
            ax_camera.set_facecolor('#1a1a2e')
            ax_camera.axis('off')
            
            if node.camera_image is not None:
                if len(node.camera_image.shape) == 3:
                    ax_camera.imshow(node.camera_image[:, :, ::-1])  # BGR to RGB
                else:
                    ax_camera.imshow(node.camera_image, cmap='gray')
            else:
                ax_camera.text(0.5, 0.5, '📷\n等待图像...', ha='center', va='center',
                             transform=ax_camera.transAxes, fontsize=12, color='gray')
            
            # 4. 更新深度图
            ax_depth.clear()
            ax_depth.set_title('🌈 深度图', fontsize=11, color='#6bcb77')
            ax_depth.set_facecolor('#1a1a2e')
            ax_depth.axis('off')
            
            if node.depth_image is not None:
                ax_depth.imshow(node.depth_image, cmap='jet', vmin=0, 
                              vmax=np.percentile(node.depth_image, 95))
            else:
                ax_depth.text(0.5, 0.5, '🌈\n等待深度...', ha='center', va='center',
                            transform=ax_depth.transAxes, fontsize=12, color='gray')
            
            # 5. 更新状态信息
            ax_status.clear()
            ax_status.set_title('📊 系统状态', fontsize=11, color='#4d96ff')
            ax_status.set_facecolor('#1a1a2e')
            ax_status.axis('off')
            
            status_text = []
            status_text.append(f"运行时间: {time.time() - node.start_time:.1f}秒")
            status_text.append(f"清扫状态: {'🟢 进行中' if node.cleaning_status else '🔴 停止'}")
            
            if node.robot_pose is not None:
                status_text.append(f"位置: ({node.robot_pose.position.x:.2f}, {node.robot_pose.position.y:.2f})")
            else:
                status_text.append("位置: 等待定位...")
            
            if node.cleaning_path is not None:
                status_text.append(f"路径点数: {len(node.cleaning_path.poses)}")
            else:
                status_text.append("路径: 未规划")
            
            if node.map_data is not None:
                free_cells = np.sum(node.map_data == 0)
                total_cells = node.map_data.size
                status_text.append(f"地图覆盖: {free_cells}/{total_cells}")
            
            for i, text in enumerate(status_text):
                ax_status.text(0.05, 0.85 - i * 0.18, text, transform=ax_status.transAxes,
                             fontsize=9, color='white', family='monospace')
            
            # 6. 更新路径规划详情
            ax_path.clear()
            ax_path.set_title('🛤️ 路径规划详情', fontsize=11, color='#ff9f43')
            ax_path.set_facecolor('#1a1a2e')
            
            if node.cleaning_path is not None and len(node.cleaning_path.poses) > 0:
                path_x = [p.pose.position.x for p in node.cleaning_path.poses]
                path_y = [p.pose.position.y for p in node.cleaning_path.poses]
                
                # 绘制路径
                colors = np.linspace(0, 1, len(path_x))
                ax_path.scatter(path_x, path_y, c=colors, cmap='viridis', s=3, alpha=0.7)
                ax_path.plot(path_x, path_y, 'w-', alpha=0.3, linewidth=0.5)
                
                # 标记起点和终点
                ax_path.scatter([path_x[0]], [path_y[0]], c='green', s=100, marker='o', 
                              label='起点', zorder=5)
                ax_path.scatter([path_x[-1]], [path_y[-1]], c='red', s=100, marker='x',
                              label='终点', zorder=5)
                
                ax_path.set_xlabel('X (米)', color='white')
                ax_path.set_ylabel('Y (米)', color='white')
                ax_path.legend(loc='upper right', fontsize=8)
                ax_path.grid(True, alpha=0.3, color='gray')
                ax_path.set_aspect('equal')
            else:
                ax_path.text(0.5, 0.5, '等待路径规划...\n\n点击"开始清扫"自动规划路径',
                           ha='center', va='center', transform=ax_path.transAxes,
                           fontsize=11, color='gray')
        
        return []
    
    # 创建动画
    ani = FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)
    
    return fig, ani


def main(args=None):
    rclpy.init(args=args)
    
    node = CleaningDashboard()
    
    # 在单独的线程中运行ROS2
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # 创建并显示界面
        fig, ani = create_dashboard(node)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

