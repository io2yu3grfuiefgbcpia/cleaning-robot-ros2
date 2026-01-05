#!/usr/bin/env python3
"""
清扫机器人键盘遥控控制器
控制方式：
    W - 前进
    S - 后退
    A - 左转
    D - 右转
    Z - 减速（油门-）
    X - 加速（油门+）
    空格 - 紧急停止
    Q - 退出
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys
import termios
import tty
import select
import threading


# 控制说明
HELP_MSG = """
╔══════════════════════════════════════════════════════════════╗
║          🎮 清扫机器人键盘遥控控制器 🎮                      ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║                    W (前进)                                  ║
║                       ↑                                      ║
║            A (左转) ←   → D (右转)                           ║
║                       ↓                                      ║
║                    S (后退)                                  ║
║                                                              ║
╠══════════════════════════════════════════════════════════════╣
║  Z - 减速 (油门-)    X - 加速 (油门+)                        ║
║  空格 - 紧急停止     Q - 退出                                ║
║  C - 开始清扫        V - 停止清扫                            ║
╠══════════════════════════════════════════════════════════════╣
║  当前速度将实时显示在下方                                    ║
╚══════════════════════════════════════════════════════════════╝
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # 速度参数
        self.max_linear_speed = 0.5   # 最大线速度 m/s
        self.max_angular_speed = 1.5  # 最大角速度 rad/s
        self.linear_step = 0.05       # 线速度步进
        self.angular_step = 0.1       # 角速度步进
        self.throttle_step = 0.1      # 油门步进
        
        # 当前状态
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.throttle = 0.5  # 油门比例 (0.0 - 1.0)
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cleaning_robot/cmd_vel', 10)
        self.cmd_vel_pub2 = self.create_publisher(Twist, '/cmd_vel', 10)  # 兼容性
        self.start_cleaning_pub = self.create_publisher(Bool, '/cleaning_robot/start_cleaning', 10)
        
        # 终端设置
        self.old_settings = None
        self.running = True
        
        # 定时发送速度命令
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info('键盘遥控控制器已启动')
    
    def get_key(self, timeout=0.1):
        """非阻塞获取键盘输入"""
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None
    
    def publish_velocity(self):
        """发布速度命令"""
        msg = Twist()
        msg.linear.x = self.current_linear * self.throttle
        msg.angular.z = self.current_angular * self.throttle
        
        self.cmd_vel_pub.publish(msg)
        self.cmd_vel_pub2.publish(msg)
    
    def stop(self):
        """紧急停止"""
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(msg)
            self.cmd_vel_pub2.publish(msg)
    
    def start_cleaning(self):
        """开始清扫"""
        msg = Bool()
        msg.data = True
        self.start_cleaning_pub.publish(msg)
        self.get_logger().info('发送开始清扫命令')
    
    def stop_cleaning(self):
        """停止清扫"""
        msg = Bool()
        msg.data = False
        self.start_cleaning_pub.publish(msg)
        self.get_logger().info('发送停止清扫命令')
    
    def print_status(self):
        """打印当前状态"""
        # 计算实际速度
        actual_linear = self.current_linear * self.throttle
        actual_angular = self.current_angular * self.throttle
        
        # 创建速度条
        linear_bar = self.create_bar(actual_linear, self.max_linear_speed)
        angular_bar = self.create_bar(actual_angular, self.max_angular_speed)
        throttle_bar = self.create_bar(self.throttle, 1.0)
        
        # 清除之前的输出并打印新状态
        status = f"\r线速度: [{linear_bar}] {actual_linear:+.2f} m/s  |  "
        status += f"角速度: [{angular_bar}] {actual_angular:+.2f} rad/s  |  "
        status += f"油门: [{throttle_bar}] {self.throttle*100:.0f}%    "
        
        print(status, end='', flush=True)
    
    def create_bar(self, value, max_value):
        """创建进度条"""
        bar_length = 10
        if max_value == 0:
            return '=' * bar_length
        
        # 处理负值
        if value < 0:
            filled = int(abs(value) / max_value * bar_length)
            return '◀' + '=' * filled + ' ' * (bar_length - filled - 1)
        else:
            filled = int(value / max_value * bar_length)
            return ' ' * (bar_length - filled - 1) + '=' * filled + '▶'
    
    def run(self):
        """主循环"""
        # 保存终端设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            # 设置终端为原始模式
            tty.setraw(sys.stdin.fileno())
            
            print(HELP_MSG)
            print("\n按任意键开始控制...\n")
            
            while self.running and rclpy.ok():
                key = self.get_key()
                
                if key:
                    key = key.lower()
                    
                    if key == 'w':
                        # 前进
                        self.current_linear = min(self.current_linear + self.linear_step, 
                                                  self.max_linear_speed)
                    elif key == 's':
                        # 后退
                        self.current_linear = max(self.current_linear - self.linear_step, 
                                                  -self.max_linear_speed)
                    elif key == 'a':
                        # 左转
                        self.current_angular = min(self.current_angular + self.angular_step, 
                                                   self.max_angular_speed)
                    elif key == 'd':
                        # 右转
                        self.current_angular = max(self.current_angular - self.angular_step, 
                                                   -self.max_angular_speed)
                    elif key == 'z':
                        # 减速（油门-）
                        self.throttle = max(self.throttle - self.throttle_step, 0.1)
                    elif key == 'x':
                        # 加速（油门+）
                        self.throttle = min(self.throttle + self.throttle_step, 1.0)
                    elif key == ' ':
                        # 紧急停止
                        self.stop()
                        print("\n🛑 紧急停止！")
                    elif key == 'c':
                        # 开始清扫
                        self.start_cleaning()
                    elif key == 'v':
                        # 停止清扫
                        self.stop_cleaning()
                    elif key == 'q' or key == '\x03':  # q 或 Ctrl+C
                        # 退出
                        self.stop()
                        self.running = False
                        print("\n\n👋 退出遥控控制器")
                        break
                    
                    # 如果没有按W/S，线速度逐渐归零
                    if key not in ['w', 's']:
                        if abs(self.current_linear) < 0.01:
                            self.current_linear = 0.0
                    
                    # 如果没有按A/D，角速度逐渐归零
                    if key not in ['a', 'd']:
                        if abs(self.current_angular) < 0.01:
                            self.current_angular = 0.0
                
                # 打印状态
                self.print_status()
                
                # 处理ROS2回调
                rclpy.spin_once(self, timeout_sec=0.01)
        
        except Exception as e:
            self.get_logger().error(f'错误: {str(e)}')
        
        finally:
            # 恢复终端设置
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            
            # 停止机器人
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    
    teleop = KeyboardTeleop()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.stop()
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





