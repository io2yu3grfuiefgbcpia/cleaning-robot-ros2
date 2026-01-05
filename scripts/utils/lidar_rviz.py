#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
import subprocess
from matplotlib.patches import Circle
import random

class LidarRViz:
    def __init__(self):
        # 创建2x2子图布局
        self.fig, ((self.ax_lidar, self.ax_map), (self.ax_robot, self.ax_data)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # 激光雷达参数
        self.angles = np.linspace(0, 2*np.pi, 360)
        self.ranges = np.ones(360) * 8.0
        self.max_range = 10.0
        
        # 机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # 地图数据
        self.map_points = []
        self.obstacles = []
        self.trajectory = []
        
        # 统计数据
        self.scan_count = 0
        self.obstacle_count = 0
        self.distance_traveled = 0.0
        
        self.setup_plots()
        
    def setup_plots(self):
        """初始化所有子图"""
        
        # 1. 激光雷达极坐标图
        self.ax_lidar.remove()
        self.ax_lidar = self.fig.add_subplot(221, projection='polar')
        self.ax_lidar.set_title('激光雷达360°扫描', fontsize=12, fontweight='bold')
        self.ax_lidar.set_ylim(0, self.max_range)
        self.ax_lidar.set_theta_zero_location('N')
        self.ax_lidar.set_theta_direction(-1)
        
        # 2. SLAM地图
        self.ax_map.set_title('SLAM地图构建', fontsize=12, fontweight='bold')
        self.ax_map.set_xlabel('X (米)')
        self.ax_map.set_ylabel('Y (米)')
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        
        # 3. 机器人局部视图
        self.ax_robot.set_title('机器人5米范围视图', fontsize=12, fontweight='bold')
        self.ax_robot.set_xlabel('X (米)')
        self.ax_robot.set_ylabel('Y (米)')
        self.ax_robot.set_aspect('equal')
        self.ax_robot.grid(True, alpha=0.3)
        
        # 4. 数据统计
        self.ax_data.set_title('实时数据统计', fontsize=12, fontweight='bold')
        self.ax_data.axis('off')
    
    def simulate_lidar_data(self):
        """模拟激光雷达数据"""
        t = time.time()
        
        # 基础环境：房间边界
        ranges = np.ones(360) * 8.0
        
        # 前方墙壁 (0°-30°, 330°-360°)
        for i in range(0, 30):
            ranges[i] = 5.0 + 0.3 * np.sin(t * 2)
        for i in range(330, 360):
            ranges[i] = 5.0 + 0.3 * np.sin(t * 2)
        
        # 右侧墙壁 (60°-120°)
        for i in range(60, 120):
            ranges[i] = 6.0 + 0.2 * np.cos(t)
        
        # 后方开阔区域 (150°-210°)
        for i in range(150, 210):
            ranges[i] = 9.0
        
        # 左侧家具 (240°-300°)
        for i in range(240, 300):
            if 260 <= i <= 280:
                ranges[i] = 2.5 + 0.5 * np.sin(t * 3)  # 移动的障碍物
            else:
                ranges[i] = 4.0
        
        # 添加噪声
        noise = np.random.normal(0, 0.03, 360)
        self.ranges = np.clip(ranges + noise, 0.1, self.max_range)
        
        # 更新机器人位置（螺旋运动）
        self.robot_theta += 0.05
        radius = 1.5 + 0.5 * np.sin(t * 0.1)
        self.robot_x = radius * np.cos(t * 0.2)
        self.robot_y = radius * np.sin(t * 0.2)
    
    def update_map_data(self):
        """更新地图数据"""
        # 记录轨迹
        self.trajectory.append((self.robot_x, self.robot_y))
        if len(self.trajectory) > 200:  # 限制轨迹长度
            self.trajectory.pop(0)
        
        # 将激光雷达数据转换为世界坐标
        new_points = []
        new_obstacles = []
        
        for i, (angle, distance) in enumerate(zip(self.angles, self.ranges)):
            if distance < self.max_range - 0.1:  # 检测到障碍物
                # 转换到世界坐标
                world_angle = angle + self.robot_theta
                obs_x = self.robot_x + distance * np.cos(world_angle)
                obs_y = self.robot_y + distance * np.sin(world_angle)
                new_obstacles.append((obs_x, obs_y))
                
                # 添加从机器人到障碍物之间的空地点
                steps = max(1, int(distance * 10))
                for step in range(steps):
                    ratio = step / steps
                    free_x = self.robot_x + ratio * distance * np.cos(world_angle)
                    free_y = self.robot_y + ratio * distance * np.sin(world_angle)
                    new_points.append((free_x, free_y))
        
        # 更新地图点（限制数量）
        self.map_points.extend(new_points)
        if len(self.map_points) > 2000:
            self.map_points = self.map_points[-2000:]
        
        self.obstacles.extend(new_obstacles)
        if len(self.obstacles) > 1000:
            self.obstacles = self.obstacles[-1000:]
    
    def draw_lidar_scan(self):
        """绘制激光雷达扫描"""
        self.ax_lidar.clear()
        self.ax_lidar.set_title('激光雷达360°扫描', fontsize=12, fontweight='bold')
        self.ax_lidar.set_ylim(0, self.max_range)
        self.ax_lidar.set_theta_zero_location('N')
        self.ax_lidar.set_theta_direction(-1)
        
        # 绘制扫描数据
        self.ax_lidar.plot(self.angles, self.ranges, 'r-', linewidth=1.5, alpha=0.8)
        self.ax_lidar.fill_between(self.angles, 0, self.ranges, alpha=0.2, color='red')
        
        # 标记危险区域
        danger_mask = self.ranges < 1.0
        if np.any(danger_mask):
            danger_angles = self.angles[danger_mask]
            danger_ranges = self.ranges[danger_mask]
            self.ax_lidar.scatter(danger_angles, danger_ranges, c='orange', s=50, marker='x', label='危险区域')
        
        # 设置极坐标网格
        self.ax_lidar.set_rticks([2, 4, 6, 8, 10])
        self.ax_lidar.grid(True, alpha=0.3)
        
        if np.any(danger_mask):
            self.ax_lidar.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))
    
    def draw_map(self):
        """绘制SLAM地图"""
        self.ax_map.clear()
        self.ax_map.set_title('SLAM地图构建', fontsize=12, fontweight='bold')
        
        # 绘制已知空地
        if self.map_points:
            map_x = [p[0] for p in self.map_points]
            map_y = [p[1] for p in self.map_points]
            self.ax_map.scatter(map_x, map_y, c='lightblue', s=1, alpha=0.3, label='已扫描区域')
        
        # 绘制障碍物
        if self.obstacles:
            obs_x = [o[0] for o in self.obstacles]
            obs_y = [o[1] for o in self.obstacles]
            self.ax_map.scatter(obs_x, obs_y, c='red', s=3, alpha=0.7, label='障碍物')
        
        # 绘制机器人轨迹
        if len(self.trajectory) > 1:
            traj_x = [p[0] for p in self.trajectory]
            traj_y = [p[1] for p in self.trajectory]
            self.ax_map.plot(traj_x, traj_y, 'g-', linewidth=2, alpha=0.7, label='机器人轨迹')
        
        # 绘制当前机器人位置
        self.ax_map.scatter([self.robot_x], [self.robot_y], c='blue', s=100, marker='o', 
                           edgecolors='darkblue', linewidth=2, label='机器人位置', zorder=5)
        
        # 机器人方向指示
        arrow_length = 1.0
        arrow_x = arrow_length * np.cos(self.robot_theta)
        arrow_y = arrow_length * np.sin(self.robot_theta)
        self.ax_map.arrow(self.robot_x, self.robot_y, arrow_x, arrow_y,
                         head_width=0.2, head_length=0.15, fc='darkblue', ec='darkblue')
        
        # 设置坐标轴
        if self.map_points or self.obstacles:
            all_x = [self.robot_x]
            all_y = [self.robot_y]
            if self.map_points:
                all_x.extend([p[0] for p in self.map_points])
                all_y.extend([p[1] for p in self.map_points])
            if self.obstacles:
                all_x.extend([o[0] for o in self.obstacles])
                all_y.extend([o[1] for o in self.obstacles])
            
            margin = 1.0
            self.ax_map.set_xlim(min(all_x) - margin, max(all_x) + margin)
            self.ax_map.set_ylim(min(all_y) - margin, max(all_y) + margin)
        else:
            self.ax_map.set_xlim(-5, 5)
            self.ax_map.set_ylim(-5, 5)
        
        self.ax_map.set_xlabel('X (米)')
        self.ax_map.set_ylabel('Y (米)')
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.legend(loc='upper left', fontsize=8)
    
    def draw_robot_view(self):
        """绘制机器人5米范围局部视图"""
        self.ax_robot.clear()
        self.ax_robot.set_title('机器人5米范围视图', fontsize=12, fontweight='bold')
        
        # 设置5米范围
        view_range = 5.0
        self.ax_robot.set_xlim(self.robot_x - view_range, self.robot_x + view_range)
        self.ax_robot.set_ylim(self.robot_y - view_range, self.robot_y + view_range)
        
        # 绘制扫描射线（每10度一条）
        for i in range(0, 360, 10):
            angle = self.angles[i]
            distance = min(self.ranges[i], view_range)
            world_angle = angle + self.robot_theta
            end_x = self.robot_x + distance * np.cos(world_angle)
            end_y = self.robot_y + distance * np.sin(world_angle)
            self.ax_robot.plot([self.robot_x, end_x], [self.robot_y, end_y], 'b-', alpha=0.3, linewidth=0.5)
        
        # 绘制范围内的障碍物
        local_obstacles = [(ox, oy) for ox, oy in self.obstacles 
                          if abs(ox - self.robot_x) <= view_range and abs(oy - self.robot_y) <= view_range]
        if local_obstacles:
            obs_x = [o[0] for o in local_obstacles]
            obs_y = [o[1] for o in local_obstacles]
            self.ax_robot.scatter(obs_x, obs_y, c='red', s=20, alpha=0.8, label='障碍物')
        
        # 绘制机器人
        robot_size = 0.3
        robot_circle = Circle((self.robot_x, self.robot_y), robot_size, 
                             facecolor='blue', edgecolor='darkblue', alpha=0.7)
        self.ax_robot.add_patch(robot_circle)
        
        # 机器人方向指示
        arrow_length = 0.8
        arrow_x = arrow_length * np.cos(self.robot_theta)
        arrow_y = arrow_length * np.sin(self.robot_theta)
        self.ax_robot.arrow(self.robot_x, self.robot_y, arrow_x, arrow_y,
                           head_width=0.15, head_length=0.1, fc='darkblue', ec='darkblue')
        
        # 绘制扫描范围圆
        scan_circle = Circle((self.robot_x, self.robot_y), view_range, 
                            fill=False, linestyle='--', alpha=0.3, color='gray')
        self.ax_robot.add_patch(scan_circle)
        
        self.ax_robot.set_xlabel('X (米)')
        self.ax_robot.set_ylabel('Y (米)')
        self.ax_robot.grid(True, alpha=0.3)
        self.ax_robot.set_aspect('equal')
    
    def draw_statistics(self):
        """绘制统计信息"""
        self.ax_data.clear()
        self.ax_data.set_title('实时数据统计', fontsize=12, fontweight='bold')
        self.ax_data.axis('off')
        
        # 更新统计数据
        self.scan_count += 1
        self.obstacle_count = len([r for r in self.ranges if r < self.max_range - 0.5])
        
        if len(self.trajectory) > 1:
            last_pos = self.trajectory[-2] if len(self.trajectory) > 1 else (0, 0)
            curr_pos = self.trajectory[-1]
            distance_step = np.sqrt((curr_pos[0] - last_pos[0])**2 + (curr_pos[1] - last_pos[1])**2)
            self.distance_traveled += distance_step
        
        # 计算扫描质量
        valid_scans = len([r for r in self.ranges if 0.1 < r < self.max_range])
        scan_quality = (valid_scans / len(self.ranges)) * 100
        
        # 计算平均距离
        avg_distance = np.mean([r for r in self.ranges if r < self.max_range])
        min_distance = np.min(self.ranges)
        
        # 显示统计信息
        stats_text = f"""
🤖 机器人状态:
   位置: ({self.robot_x:.2f}, {self.robot_y:.2f})
   角度: {np.degrees(self.robot_theta):.1f}°
   行驶距离: {self.distance_traveled:.2f}m

📡 激光雷达数据:
   扫描次数: {self.scan_count}
   扫描质量: {scan_quality:.1f}%
   检测障碍物: {self.obstacle_count}个
   
📏 距离测量:
   平均距离: {avg_distance:.2f}m
   最近障碍物: {min_distance:.2f}m
   扫描范围: {self.max_range}m

🗺️ 地图构建:
   地图点数: {len(self.map_points)}
   障碍物点: {len(self.obstacles)}
   轨迹点数: {len(self.trajectory)}

⏰ 系统时间: {time.strftime('%H:%M:%S')}
"""
        
        self.ax_data.text(0.05, 0.95, stats_text, transform=self.ax_data.transAxes,
                         fontsize=11, verticalalignment='top', fontfamily='monospace',
                         bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
    
    def animate(self, frame):
        """动画更新函数"""
        # 获取或模拟数据
        self.simulate_lidar_data()
        self.update_map_data()
        
        # 更新所有图表
        self.draw_lidar_scan()
        self.draw_map()
        self.draw_robot_view()
        self.draw_statistics()
        
        # 更新总标题
        self.fig.suptitle('清扫机器人激光雷达RViz - 实时可视化系统', fontsize=16, fontweight='bold')
    
    def start_visualization(self):
        """启动可视化"""
        print("🚀 启动激光雷达RViz可视化系统")
        print("=" * 50)
        print("📊 显示窗口包含:")
        print("  • 左上: 激光雷达360°扫描数据")
        print("  • 右上: SLAM地图构建过程")
        print("  • 左下: 机器人5米范围局部视图")
        print("  • 右下: 实时数据统计")
        print("=" * 50)
        print("💡 功能特点:")
        print("  • 实时激光雷达数据可视化")
        print("  • 动态地图构建显示")
        print("  • 障碍物检测与标记")
        print("  • 机器人轨迹追踪")
        print("  • 详细统计信息")
        print("=" * 50)
        print("🔧 控制:")
        print("  • 关闭窗口或按Ctrl+C停止")
        print("  • 可视化会自动更新")
        print("=" * 50)
        
        # 设置动画
        ani = animation.FuncAnimation(self.fig, self.animate, interval=200, blit=False)
        
        # 调整布局
        plt.tight_layout()
        
        # 显示窗口
        plt.show()

def main():
    try:
        # 启动可视化
        rviz = LidarRViz()
        rviz.start_visualization()
        
    except KeyboardInterrupt:
        print("\n✅ 激光雷达RViz已停止")
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("请确保已安装matplotlib: sudo apt install python3-matplotlib python3-tk")

if __name__ == "__main__":
    main() 