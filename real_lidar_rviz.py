#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
from matplotlib.patches import Circle
import threading

class RealLidarRViz(Node):
    def __init__(self):
        super().__init__('real_lidar_rviz')
        
        # 创建2x2子图布局
        self.fig, ((self.ax_lidar, self.ax_map), (self.ax_robot, self.ax_data)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # 激光雷达数据
        self.laser_data = None
        self.angles = None
        self.ranges = None
        self.max_range = 10.0
        
        # 机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # 地图数据
        self.map_data = None
        self.map_info = None
        self.obstacles = []
        self.trajectory = []
        
        # 统计数据
        self.scan_count = 0
        self.last_update_time = time.time()
        
        # ROS2订阅者
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/cleaning_robot/scan',
            self.laser_callback,
            10
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/cleaning_robot/odom',
            self.odom_callback,
            10
        )
        
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cleaning_robot/cmd_vel',
            self.cmd_callback,
            10
        )
        
        self.setup_plots()
        
        print("🚀 真实激光雷达RViz已启动")
        print("📡 正在订阅以下话题:")
        print("   • /cleaning_robot/scan - 激光雷达数据")
        print("   • /cleaning_robot/odom - 里程计数据")
        print("   • /map - 地图数据")
        print("   • /cleaning_robot/cmd_vel - 速度命令")
        
    def laser_callback(self, msg):
        """激光雷达数据回调"""
        self.laser_data = msg
        self.ranges = np.array(msg.ranges)
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        self.max_range = msg.range_max
        self.scan_count += 1
        
        # 更新障碍物数据
        self.update_obstacles_from_laser()
        
    def odom_callback(self, msg):
        """里程计数据回调"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # 从四元数计算角度
        orientation = msg.pose.pose.orientation
        self.robot_theta = 2 * np.arctan2(orientation.z, orientation.w)
        
        # 记录轨迹
        self.trajectory.append((self.robot_x, self.robot_y))
        if len(self.trajectory) > 200:
            self.trajectory.pop(0)
            
    def map_callback(self, msg):
        """地图数据回调"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        
    def cmd_callback(self, msg):
        """速度命令回调"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def update_obstacles_from_laser(self):
        """从激光雷达数据更新障碍物"""
        if self.ranges is None or self.angles is None:
            return
            
        new_obstacles = []
        for angle, distance in zip(self.angles, self.ranges):
            if 0.1 < distance < self.max_range - 0.1:
                # 转换到世界坐标
                world_angle = angle + self.robot_theta
                obs_x = self.robot_x + distance * np.cos(world_angle)
                obs_y = self.robot_y + distance * np.sin(world_angle)
                new_obstacles.append((obs_x, obs_y))
        
        # 保持最近的障碍物点
        self.obstacles.extend(new_obstacles)
        if len(self.obstacles) > 1000:
            self.obstacles = self.obstacles[-1000:]
    
    def setup_plots(self):
        """初始化所有子图"""
        
        # 1. 激光雷达极坐标图
        self.ax_lidar.remove()
        self.ax_lidar = self.fig.add_subplot(221, projection='polar')
        self.ax_lidar.set_title('真实激光雷达360°扫描', fontsize=12, fontweight='bold')
        self.ax_lidar.set_theta_zero_location('N')
        self.ax_lidar.set_theta_direction(-1)
        
        # 2. 地图视图
        self.ax_map.set_title('ROS2地图 + 激光雷达数据', fontsize=12, fontweight='bold')
        self.ax_map.set_xlabel('X (米)')
        self.ax_map.set_ylabel('Y (米)')
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        
        # 3. 机器人局部视图
        self.ax_robot.set_title('机器人局部视图 (真实数据)', fontsize=12, fontweight='bold')
        self.ax_robot.set_xlabel('X (米)')
        self.ax_robot.set_ylabel('Y (米)')
        self.ax_robot.set_aspect('equal')
        self.ax_robot.grid(True, alpha=0.3)
        
        # 4. 实时统计
        self.ax_data.set_title('真实数据统计', fontsize=12, fontweight='bold')
        self.ax_data.axis('off')
    
    def draw_lidar_scan(self):
        """绘制真实激光雷达扫描"""
        self.ax_lidar.clear()
        self.ax_lidar.set_title('真实激光雷达360°扫描', fontsize=12, fontweight='bold')
        self.ax_lidar.set_theta_zero_location('N')
        self.ax_lidar.set_theta_direction(-1)
        
        if self.ranges is not None and self.angles is not None:
            # 设置范围
            self.ax_lidar.set_ylim(0, self.max_range)
            
            # 绘制扫描数据
            self.ax_lidar.plot(self.angles, self.ranges, 'r-', linewidth=1.5, alpha=0.8, label='激光扫描')
            self.ax_lidar.fill_between(self.angles, 0, self.ranges, alpha=0.2, color='red')
            
            # 标记危险区域
            danger_mask = self.ranges < 0.5
            if np.any(danger_mask):
                danger_angles = self.angles[danger_mask]
                danger_ranges = self.ranges[danger_mask]
                self.ax_lidar.scatter(danger_angles, danger_ranges, c='orange', s=50, marker='x', label='危险区域(<0.5m)')
            
            # 设置极坐标网格
            self.ax_lidar.set_rticks([1, 2, 4, 6, 8, 10])
            self.ax_lidar.grid(True, alpha=0.3)
            
            if np.any(danger_mask):
                self.ax_lidar.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))
        else:
            self.ax_lidar.text(0.5, 0.5, '等待激光雷达数据...', 
                             transform=self.ax_lidar.transAxes, 
                             ha='center', va='center', fontsize=14)
    
    def draw_map_view(self):
        """绘制地图和机器人"""
        self.ax_map.clear()
        self.ax_map.set_title('ROS2地图 + 激光雷达数据', fontsize=12, fontweight='bold')
        
        # 绘制ROS2地图
        if self.map_data is not None and self.map_info is not None:
            # 创建地图显示
            map_display = np.zeros_like(self.map_data, dtype=float)
            map_display[self.map_data == -1] = 0.5  # 未知区域
            map_display[self.map_data == 0] = 1.0   # 空地
            map_display[self.map_data == 100] = 0.0 # 障碍物
            
            # 计算地图坐标
            origin_x = self.map_info.origin.position.x
            origin_y = self.map_info.origin.position.y
            resolution = self.map_info.resolution
            
            extent = [origin_x, 
                     origin_x + self.map_data.shape[1] * resolution,
                     origin_y,
                     origin_y + self.map_data.shape[0] * resolution]
            
            self.ax_map.imshow(map_display, extent=extent, origin='lower', 
                             cmap='gray', alpha=0.8, aspect='equal')
        
        # 绘制激光雷达检测的障碍物
        if self.obstacles:
            obs_x = [o[0] for o in self.obstacles]
            obs_y = [o[1] for o in self.obstacles]
            self.ax_map.scatter(obs_x, obs_y, c='red', s=2, alpha=0.6, label='激光雷达检测')
        
        # 绘制机器人轨迹
        if len(self.trajectory) > 1:
            traj_x = [p[0] for p in self.trajectory]
            traj_y = [p[1] for p in self.trajectory]
            self.ax_map.plot(traj_x, traj_y, 'g-', linewidth=2, alpha=0.7, label='机器人轨迹')
        
        # 绘制当前机器人位置
        self.ax_map.scatter([self.robot_x], [self.robot_y], c='blue', s=100, marker='o', 
                           edgecolors='darkblue', linewidth=2, label='机器人位置', zorder=5)
        
        # 机器人方向指示
        arrow_length = 0.5
        arrow_x = arrow_length * np.cos(self.robot_theta)
        arrow_y = arrow_length * np.sin(self.robot_theta)
        self.ax_map.arrow(self.robot_x, self.robot_y, arrow_x, arrow_y,
                         head_width=0.1, head_length=0.08, fc='darkblue', ec='darkblue')
        
        self.ax_map.set_xlabel('X (米)')
        self.ax_map.set_ylabel('Y (米)')
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.legend(loc='upper left', fontsize=8)
    
    def draw_robot_view(self):
        """绘制机器人局部视图"""
        self.ax_robot.clear()
        self.ax_robot.set_title('机器人局部视图 (真实数据)', fontsize=12, fontweight='bold')
        
        if self.ranges is not None and self.angles is not None:
            # 设置局部视图范围
            view_range = 5.0
            self.ax_robot.set_xlim(self.robot_x - view_range, self.robot_x + view_range)
            self.ax_robot.set_ylim(self.robot_y - view_range, self.robot_y + view_range)
            
            # 绘制激光射线（每10度一条）
            step = max(1, len(self.angles) // 36)  # 大约36条射线
            for i in range(0, len(self.angles), step):
                angle = self.angles[i]
                distance = min(self.ranges[i], view_range)
                if distance > 0.1:  # 有效距离
                    world_angle = angle + self.robot_theta
                    end_x = self.robot_x + distance * np.cos(world_angle)
                    end_y = self.robot_y + distance * np.sin(world_angle)
                    
                    # 根据距离设置颜色
                    if distance < 0.5:
                        color = 'red'
                        alpha = 0.8
                    elif distance < 1.0:
                        color = 'orange'
                        alpha = 0.6
                    else:
                        color = 'lightblue'
                        alpha = 0.4
                    
                    self.ax_robot.plot([self.robot_x, end_x], [self.robot_y, end_y], 
                                     color=color, alpha=alpha, linewidth=1)
            
            # 绘制机器人
            robot_circle = Circle((self.robot_x, self.robot_y), 0.2, 
                                facecolor='blue', edgecolor='darkblue', alpha=0.8)
            self.ax_robot.add_patch(robot_circle)
            
            # 机器人方向指示
            arrow_length = 0.4
            arrow_x = arrow_length * np.cos(self.robot_theta)
            arrow_y = arrow_length * np.sin(self.robot_theta)
            self.ax_robot.arrow(self.robot_x, self.robot_y, arrow_x, arrow_y,
                               head_width=0.1, head_length=0.05, fc='darkblue', ec='darkblue')
        
        self.ax_robot.set_xlabel('X (米)')
        self.ax_robot.set_ylabel('Y (米)')
        self.ax_robot.grid(True, alpha=0.3)
        self.ax_robot.set_aspect('equal')
    
    def draw_statistics(self):
        """绘制统计信息"""
        self.ax_data.clear()
        self.ax_data.set_title('真实数据统计', fontsize=12, fontweight='bold')
        self.ax_data.axis('off')
        
        # 计算统计数据
        current_time = time.time()
        scan_rate = self.scan_count / (current_time - self.last_update_time + 1)
        
        if self.ranges is not None:
            valid_ranges = self.ranges[np.isfinite(self.ranges) & (self.ranges > 0.1)]
            if len(valid_ranges) > 0:
                avg_distance = np.mean(valid_ranges)
                min_distance = np.min(valid_ranges)
                max_distance = np.max(valid_ranges)
                obstacle_count = len([r for r in valid_ranges if r < self.max_range - 0.1])
            else:
                avg_distance = min_distance = max_distance = 0.0
                obstacle_count = 0
        else:
            avg_distance = min_distance = max_distance = 0.0
            obstacle_count = 0
        
        # 显示统计信息
        stats_text = f"""
🤖 机器人状态 (真实数据):
   位置: ({self.robot_x:.2f}, {self.robot_y:.2f})
   角度: {np.degrees(self.robot_theta):.1f}°
   线速度: {self.linear_vel:.2f} m/s
   角速度: {np.degrees(self.angular_vel):.1f}°/s

📡 激光雷达数据:
   扫描频率: {scan_rate:.1f} Hz
   扫描次数: {self.scan_count}
   检测障碍物: {obstacle_count}个
   
📏 距离测量:
   平均距离: {avg_distance:.2f}m
   最近障碍物: {min_distance:.2f}m
   最远距离: {max_distance:.2f}m
   扫描范围: {self.max_range:.1f}m

🗺️ 地图数据:
   轨迹点数: {len(self.trajectory)}
   障碍物点: {len(self.obstacles)}
   地图状态: {'✅ 已加载' if self.map_data is not None else '❌ 未加载'}

⏰ 系统时间: {time.strftime('%H:%M:%S')}
🔗 数据源: ROS2话题 (真实传感器)
"""
        
        self.ax_data.text(0.05, 0.95, stats_text, transform=self.ax_data.transAxes,
                         fontsize=10, verticalalignment='top', fontfamily='monospace',
                         bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgreen", alpha=0.8))
    
    def animate(self, frame):
        """动画更新函数"""
        self.draw_lidar_scan()
        self.draw_map_view()
        self.draw_robot_view()
        self.draw_statistics()
        
        # 更新总标题
        self.fig.suptitle('清扫机器人真实激光雷达RViz - ROS2数据可视化', fontsize=16, fontweight='bold')
    
    def start_visualization(self):
        """启动可视化"""
        print("=" * 60)
        print("📊 显示窗口包含:")
        print("  • 左上: 真实激光雷达360°扫描数据")
        print("  • 右上: ROS2地图 + 激光雷达检测")
        print("  • 左下: 机器人局部视图 (真实数据)")
        print("  • 右下: 真实数据统计")
        print("=" * 60)
        print("💡 数据来源: ROS2话题 (真实N10P激光雷达)")
        print("🔧 控制: 关闭窗口停止程序")
        print("=" * 60)
        
        # 设置动画
        ani = animation.FuncAnimation(self.fig, self.animate, interval=100, blit=False)
        
        # 调整布局
        plt.tight_layout()
        
        # 显示窗口
        plt.show()

def main():
    rclpy.init()
    
    try:
        rviz_node = RealLidarRViz()
        
        # 在单独线程中运行ROS2
        def ros_spin():
            rclpy.spin(rviz_node)
        
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        # 启动可视化
        rviz_node.start_visualization()
        
    except KeyboardInterrupt:
        print("\n✅ 真实激光雷达RViz已停止")
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("请确保ROS2环境和激光雷达驱动正常运行")
    finally:
        if rclpy.ok():
            rviz_node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main() 