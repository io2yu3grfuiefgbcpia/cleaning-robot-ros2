#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math

class SimpleOdomPublisher(Node):
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # 发布器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅器
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 状态变量
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # 定时器
        self.timer = self.create_timer(0.05, self.update_odometry)  # 20Hz
        
        self.get_logger().info('简单里程计发布器已启动')

    def cmd_vel_callback(self, msg):
        # 存储最新的速度命令
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # 简单的里程计积分
        delta_x = self.linear_vel * math.cos(self.theta) * dt
        delta_y = self.linear_vel * math.sin(self.theta) * dt
        delta_theta = self.angular_vel * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # 发布TF变换
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # 转换欧拉角到四元数（简化版本）
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        self.tf_broadcaster.sendTransform(t)
        
        # 发布里程计消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # 位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # 速度
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 