#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
from matplotlib.patches import Circle, Wedge

class SimpleLidarViewer:
    def __init__(self):
        # 创建1x2子图布局
        self.fig, (self.ax_polar, self.ax_cartesian) = plt.subplots(1, 2, figsize=(16, 8))
        
        # 激光雷达参数
        self.angles = np.linspace(0, 2*np.pi, 360)
        self.ranges = np.ones(360) * 5.0
        self.max_range = 8.0
        
        # 机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # 地图数据
        self.obstacles = []
        
        self.setup_plots()
        
    def setup_plots(self):
        """初始化图表"""
        # 极坐标图
        self.ax_polar.remove()
        self.ax_polar = self.fig.add_subplot(121, projection='polar')
        self.ax_polar.set_title('激光雷达360°扫描', fontsize=14, fontweight='bold')
        self.ax_polar.set_ylim(0, self.max_range)
        self.ax_polar.set_theta_zero_location('N')
        self.ax_polar.set_theta_direction(-1)
        
        # 笛卡尔坐标图
        self.ax_cartesian.set_title('机器人局部视图 (8米范围)', fontsize=14, fontweight='bold')
        self.ax_cartesian.set_xlabel('X (米)')
        self.ax_cartesian.set_ylabel('Y (米)')
        self.ax_cartesian.set_aspect('equal')
        self.ax_cartesian.grid(True, alpha=0.3)
    
    def simulate_environment(self):
        """模拟激光雷达在室内环境的数据"""
        t = time.time()
        
        # 创建一个带有家具的房间环境
        ranges = np.ones(360) * self.max_range
        
        # 房间墙壁
        # 前墙 (350°-10°)
        for i in list(range(350, 360)) + list(range(0, 10)):
            ranges[i] = 6.0 + 0.2 * np.sin(t)
        
        # 右墙 (80°-100°)
        for i in range(80, 100):
            ranges[i] = 5.5 + 0.1 * np.cos(t * 2)
        
        # 后墙 (170°-190°)
        for i in range(170, 190):
            ranges[i] = 7.0
        
        # 左墙 (260°-280°)
        for i in range(260, 280):
            ranges[i] = 5.0
        
        # 桌子 (30°-50°)
        for i in range(30, 50):
            ranges[i] = 2.0 + 0.3 * np.sin(t * 3)
        
        # 椅子 (120°-140°)
        for i in range(120, 140):
            ranges[i] = 1.5 + 0.2 * np.cos(t * 4)
        
        # 柜子 (220°-240°)
        for i in range(220, 240):
            ranges[i] = 3.5
        
        # 移动的人/宠物 (在不同位置出现)
        person_angle = int(180 + 30 * np.sin(t * 0.5)) % 360
        for i in range(max(0, person_angle-5), min(360, person_angle+5)):
            ranges[i] = 2.0 + 0.5 * np.sin(t * 2)
        
        # 添加现实的噪声
        noise = np.random.normal(0, 0.05, 360)
        self.ranges = np.clip(ranges + noise, 0.1, self.max_range)
        
        # 机器人旋转运动
        self.robot_theta += 0.03
        
        # 轻微的前进运动
        self.robot_x += 0.02 * np.cos(self.robot_theta)
        self.robot_y += 0.02 * np.sin(self.robot_theta)
    
    def update_obstacles(self):
        """更新障碍物位置"""
        new_obstacles = []
        
        for i, (angle, distance) in enumerate(zip(self.angles, self.ranges)):
            if distance < self.max_range - 0.1:
                # 转换到世界坐标
                world_angle = angle + self.robot_theta
                obs_x = self.robot_x + distance * np.cos(world_angle)
                obs_y = self.robot_y + distance * np.sin(world_angle)
                new_obstacles.append((obs_x, obs_y))
        
        # 保持最近的障碍物点
        self.obstacles.extend(new_obstacles)
        if len(self.obstacles) > 500:
            self.obstacles = self.obstacles[-500:]
    
    def draw_polar_view(self):
        """绘制极坐标激光雷达视图"""
        self.ax_polar.clear()
        self.ax_polar.set_title('激光雷达360°扫描', fontsize=14, fontweight='bold')
        self.ax_polar.set_ylim(0, self.max_range)
        self.ax_polar.set_theta_zero_location('N')
        self.ax_polar.set_theta_direction(-1)
        
        # 绘制扫描数据 - 主要线条
        self.ax_polar.plot(self.angles, self.ranges, 'r-', linewidth=2, alpha=0.9, label='激光扫描')
        
        # 填充扫描区域
        self.ax_polar.fill_between(self.angles, 0, self.ranges, alpha=0.15, color='red')
        
        # 标记危险区域（1米内）
        danger_mask = self.ranges < 1.0
        if np.any(danger_mask):
            danger_angles = self.angles[danger_mask]
            danger_ranges = self.ranges[danger_mask]
            self.ax_polar.scatter(danger_angles, danger_ranges, 
                                c='orange', s=80, marker='X', 
                                label='危险区域(<1m)', zorder=5)
        
        # 标记近距离物体（2米内）
        close_mask = (self.ranges < 2.0) & (self.ranges >= 1.0)
        if np.any(close_mask):
            close_angles = self.angles[close_mask]
            close_ranges = self.ranges[close_mask]
            self.ax_polar.scatter(close_angles, close_ranges, 
                                c='yellow', s=30, marker='o', 
                                alpha=0.7, label='近距离物体(<2m)')
        
        # 设置极坐标网格
        self.ax_polar.set_rticks([1, 2, 4, 6, 8])
        self.ax_polar.set_rmax(self.max_range)
        self.ax_polar.grid(True, alpha=0.4)
        
        # 添加方向标注
        self.ax_polar.text(0, self.max_range*0.9, '前', ha='center', va='center', 
                          fontsize=12, fontweight='bold', color='blue')
        self.ax_polar.text(np.pi/2, self.max_range*0.9, '右', ha='center', va='center', 
                          fontsize=12, fontweight='bold', color='blue')
        self.ax_polar.text(np.pi, self.max_range*0.9, '后', ha='center', va='center', 
                          fontsize=12, fontweight='bold', color='blue')
        self.ax_polar.text(3*np.pi/2, self.max_range*0.9, '左', ha='center', va='center', 
                          fontsize=12, fontweight='bold', color='blue')
        
        # 图例
        if np.any(danger_mask) or np.any(close_mask):
            self.ax_polar.legend(loc='upper left', bbox_to_anchor=(0.1, 1.0), fontsize=10)
    
    def draw_cartesian_view(self):
        """绘制笛卡尔坐标机器人视图"""
        self.ax_cartesian.clear()
        self.ax_cartesian.set_title('机器人局部视图 (8米范围)', fontsize=14, fontweight='bold')
        
        # 设置视图范围
        view_range = self.max_range
        self.ax_cartesian.set_xlim(self.robot_x - view_range, self.robot_x + view_range)
        self.ax_cartesian.set_ylim(self.robot_y - view_range, self.robot_y + view_range)
        
        # 绘制扫描射线（每15度一条）
        for i in range(0, 360, 15):
            angle = self.angles[i]
            distance = self.ranges[i]
            world_angle = angle + self.robot_theta
            end_x = self.robot_x + distance * np.cos(world_angle)
            end_y = self.robot_y + distance * np.sin(world_angle)
            
            # 不同距离用不同颜色
            if distance < 1.0:
                color = 'red'
                alpha = 0.8
                linewidth = 2
            elif distance < 2.0:
                color = 'orange'
                alpha = 0.6
                linewidth = 1.5
            else:
                color = 'lightblue'
                alpha = 0.4
                linewidth = 1
            
            self.ax_cartesian.plot([self.robot_x, end_x], [self.robot_y, end_y], 
                                 color=color, alpha=alpha, linewidth=linewidth)
        
        # 绘制障碍物点
        if self.obstacles:
            local_obstacles = [(ox, oy) for ox, oy in self.obstacles 
                             if abs(ox - self.robot_x) <= view_range and 
                                abs(oy - self.robot_y) <= view_range]
            if local_obstacles:
                obs_x = [o[0] for o in local_obstacles]
                obs_y = [o[1] for o in local_obstacles]
                self.ax_cartesian.scatter(obs_x, obs_y, c='red', s=15, 
                                        alpha=0.7, label='检测到的障碍物')
        
        # 绘制机器人本体
        robot_radius = 0.25
        robot_circle = Circle((self.robot_x, self.robot_y), robot_radius, 
                            facecolor='blue', edgecolor='darkblue', 
                            alpha=0.8, linewidth=2)
        self.ax_cartesian.add_patch(robot_circle)
        
        # 机器人朝向箭头
        arrow_length = 0.6
        arrow_x = arrow_length * np.cos(self.robot_theta)
        arrow_y = arrow_length * np.sin(self.robot_theta)
        self.ax_cartesian.arrow(self.robot_x, self.robot_y, arrow_x, arrow_y,
                              head_width=0.15, head_length=0.1, 
                              fc='darkblue', ec='darkblue', linewidth=2)
        
        # 绘制扫描范围圆
        scan_circle = Circle((self.robot_x, self.robot_y), self.max_range, 
                           fill=False, linestyle='--', alpha=0.3, 
                           color='gray', linewidth=2)
        self.ax_cartesian.add_patch(scan_circle)
        
        # 绘制危险区域圆
        danger_circle = Circle((self.robot_x, self.robot_y), 1.0, 
                             fill=False, linestyle=':', alpha=0.5, 
                             color='red', linewidth=2)
        self.ax_cartesian.add_patch(danger_circle)
        
        # 添加文本信息
        info_text = f"""机器人位置: ({self.robot_x:.1f}, {self.robot_y:.1f})
朝向角度: {np.degrees(self.robot_theta):.0f}°
障碍物数: {len([r for r in self.ranges if r < self.max_range-0.1])}
最近距离: {np.min(self.ranges):.2f}m"""
        
        self.ax_cartesian.text(0.02, 0.98, info_text, transform=self.ax_cartesian.transAxes,
                             fontsize=10, verticalalignment='top', fontfamily='monospace',
                             bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        self.ax_cartesian.set_xlabel('X (米)')
        self.ax_cartesian.set_ylabel('Y (米)')
        self.ax_cartesian.grid(True, alpha=0.3)
        self.ax_cartesian.set_aspect('equal')
        
        if self.obstacles:
            self.ax_cartesian.legend(loc='upper right', fontsize=9)
    
    def animate(self, frame):
        """动画更新函数"""
        self.simulate_environment()
        self.update_obstacles()
        
        self.draw_polar_view()
        self.draw_cartesian_view()
        
        # 设置总标题
        self.fig.suptitle(f'清扫机器人激光雷达实时可视化 - 帧:{frame}', 
                         fontsize=16, fontweight='bold')
    
    def start_visualization(self):
        """启动可视化"""
        print("🚀 启动简化激光雷达可视化系统")
        print("=" * 40)
        print("📊 显示内容:")
        print("  • 左侧: 激光雷达360°极坐标扫描")
        print("  • 右侧: 机器人8米范围笛卡尔视图")
        print("=" * 40)
        print("💡 功能:")
        print("  • 实时环境扫描模拟")
        print("  • 障碍物检测和显示")
        print("  • 危险区域标记")
        print("  • 机器人位置和朝向")
        print("=" * 40)
        print("🎨 颜色说明:")
        print("  • 红色: 激光扫描数据/障碍物")
        print("  • 蓝色: 机器人位置和朝向")
        print("  • 橙色: 危险区域(<1米)")
        print("  • 黄色: 近距离物体(<2米)")
        print("=" * 40)
        
        # 创建动画
        ani = animation.FuncAnimation(self.fig, self.animate, interval=150, blit=False)
        
        # 调整布局
        plt.tight_layout()
        
        # 显示
        plt.show()

def main():
    try:
        viewer = SimpleLidarViewer()
        viewer.start_visualization()
    except KeyboardInterrupt:
        print("\n✅ 可视化已停止")
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("请确保已安装matplotlib: sudo apt install python3-matplotlib python3-tk")

if __name__ == "__main__":
    main() 