#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class CleaningTester(Node):
    def __init__(self):
        super().__init__('cleaning_tester')
        
        # 发布者
        self.start_cleaning_pub = self.create_publisher(Bool, '/cleaning_robot/start_cleaning', 10)
        
        self.get_logger().info('清扫测试节点已启动')
        
        # 等待一段时间后开始清扫
        self.timer = self.create_timer(5.0, self.start_cleaning)
    
    def start_cleaning(self):
        """开始清扫"""
        msg = Bool()
        msg.data = True
        self.start_cleaning_pub.publish(msg)
        self.get_logger().info('发送开始清扫命令')
        
        # 只发送一次，然后销毁定时器
        self.timer.destroy()

def main(args=None):
    rclpy.init(args=args)
    
    tester = CleaningTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 