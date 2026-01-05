#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time

class SimpleOdomPublisher(Node):
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # 订阅速度命令
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 发布里程计
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 状态变量
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        self.last_time = time.time()
        
        # 定时器发布里程计
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10Hz
        
        self.get_logger().info('简单里程计发布器已启动')
    
    def cmd_vel_callback(self, msg):
        """更新速度命令"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        
        return [x, y, z, w]
    
    def publish_odom(self):
        """发布里程计信息"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # 积分计算位置
        delta_x = self.linear_vel * math.cos(self.theta) * dt
        delta_y = self.linear_vel * math.sin(self.theta) * dt
        delta_theta = self.angular_vel * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # 规范化角度
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # 创建时间戳
        current_ros_time = self.get_clock().now().to_msg()
        
        # 发布TF变换
        transform = TransformStamped()
        transform.header.stamp = current_ros_time
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        quat = self.euler_to_quaternion(0, 0, self.theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
        
        # 发布里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_ros_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # 位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # 速度
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel
        
        # 协方差矩阵（简单设置）
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        
        odom_msg.twist.covariance[0] = 0.1  # vx
        odom_msg.twist.covariance[35] = 0.1  # vyaw
        
        self.odom_pub.publish(odom_msg)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    odom_publisher = SimpleOdomPublisher()
    
    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 