# 实物硬件适配指南

## 🔧 从仿真到实物的适配步骤

### 1. 硬件清单和接口说明

请提供以下实物硬件的具体信息：

#### 🚗 小车底盘和电机
```
需要的信息：
- 电机类型：步进电机/伺服电机/直流电机
- 控制器型号：Arduino/树莓派/工控板
- 通信协议：串口/I2C/CAN/以太网
- 编码器类型：增量式/绝对式编码器
- 轮子直径和轴距
- 最大速度和载重
```

#### 📡 激光雷达
```
需要的信息：
- 雷达型号：RPLIDAR A1/A2/S1, Hokuyo, Velodyne等
- 接口类型：USB/串口/以太网
- 扫描频率和精度
- 测距范围
- 数据格式
```

#### 📷 摄像头
```
需要的信息：
- 摄像头型号：USB摄像头/CSI摄像头/网络摄像头
- 分辨率和帧率
- 是否支持双目立体视觉
- 镜头参数（焦距、畸变参数）
```

#### 🧹 清扫装置
```
需要的信息：
- 清扫刷类型：滚刷/边刷
- 驱动方式：电机驱动/气动
- 控制接口：PWM/继电器
```

### 2. 硬件驱动开发模板

我将为您创建通用的硬件驱动接口：

#### 电机控制器驱动模板
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial  # 或其他通信库

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # 硬件连接参数 - 需要根据实际硬件修改
        self.port = '/dev/ttyUSB0'  # 串口设备
        self.baudrate = 115200
        
        # 机器人参数 - 需要根据实际尺寸修改
        self.wheel_diameter = 0.1  # 轮子直径（米）
        self.wheel_separation = 0.35  # 轮距（米）
        
        # 初始化硬件连接
        self.setup_hardware()
        
        # ROS2订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cleaning_robot/cmd_vel', 
            self.cmd_vel_callback, 10)
    
    def setup_hardware(self):
        """初始化硬件连接"""
        try:
            self.serial_conn = serial.Serial(
                self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'电机控制器连接成功: {self.port}')
        except Exception as e:
            self.get_logger().error(f'电机控制器连接失败: {e}')
    
    def cmd_vel_callback(self, msg):
        """速度命令回调函数"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 差分驱动运动学计算
        left_speed, right_speed = self.diff_drive_kinematics(
            linear_x, angular_z)
        
        # 发送命令到硬件
        self.send_motor_command(left_speed, right_speed)
    
    def diff_drive_kinematics(self, linear, angular):
        """差分驱动运动学计算"""
        left_speed = linear - angular * self.wheel_separation / 2.0
        right_speed = linear + angular * self.wheel_separation / 2.0
        return left_speed, right_speed
    
    def send_motor_command(self, left_speed, right_speed):
        """发送电机命令 - 需要根据实际协议修改"""
        # 示例：发送串口命令
        command = f"M{left_speed:.2f},{right_speed:.2f}\n"
        try:
            self.serial_conn.write(command.encode())
        except Exception as e:
            self.get_logger().error(f'发送电机命令失败: {e}')
```

#### 激光雷达驱动模板
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarDriver(Node):
    def __init__(self):
        super().__init__('lidar_driver')
        
        # 雷达参数 - 需要根据实际雷达修改
        self.lidar_port = '/dev/ttyUSB1'
        self.scan_frequency = 10.0  # Hz
        self.angle_min = -3.14159
        self.angle_max = 3.14159
        self.range_min = 0.1
        self.range_max = 10.0
        
        # 发布者
        self.scan_pub = self.create_publisher(
            LaserScan, '/cleaning_robot/scan', 10)
        
        # 定时器
        self.timer = self.create_timer(
            1.0/self.scan_frequency, self.publish_scan)
        
        # 初始化雷达
        self.setup_lidar()
    
    def setup_lidar(self):
        """初始化激光雷达"""
        # 根据雷达型号添加初始化代码
        pass
    
    def publish_scan(self):
        """发布激光扫描数据"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "lidar_link"
        
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = (self.angle_max - self.angle_min) / 360
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.scan_frequency
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        # 从实际雷达读取数据
        ranges = self.read_lidar_data()
        scan_msg.ranges = ranges
        
        self.scan_pub.publish(scan_msg)
    
    def read_lidar_data(self):
        """从实际雷达读取数据 - 需要根据雷达协议实现"""
        # 示例：返回模拟数据
        ranges = []
        for i in range(360):
            # 这里应该是实际的雷达数据读取代码
            ranges.append(5.0)  # 示例：5米距离
        return ranges
```

### 3. 硬件配置文件

创建硬件配置文件来管理不同的硬件参数：

```yaml
# config/hardware_config.yaml
robot_hardware:
  # 底盘参数
  chassis:
    wheel_diameter: 0.1      # 轮子直径（米）
    wheel_separation: 0.35   # 轴距（米）
    max_linear_velocity: 1.0 # 最大线速度（米/秒）
    max_angular_velocity: 2.0 # 最大角速度（弧度/秒）
  
  # 电机控制器
  motor_controller:
    type: "arduino"          # arduino/raspberry_pi/custom
    port: "/dev/ttyUSB0"     # 串口设备
    baudrate: 115200         # 波特率
    protocol: "serial"       # serial/i2c/can
  
  # 激光雷达
  lidar:
    type: "rplidar_a1"       # rplidar_a1/rplidar_a2/hokuyo/velodyne
    port: "/dev/ttyUSB1"
    frame_id: "lidar_link"
    scan_frequency: 10.0
    angle_min: -3.14159
    angle_max: 3.14159
    range_min: 0.1
    range_max: 10.0
  
  # 摄像头
  camera:
    type: "usb_stereo"       # usb_stereo/csi/network
    left_device: "/dev/video0"
    right_device: "/dev/video1"
    width: 640
    height: 480
    fps: 30
    baseline: 0.12           # 双目基线距离（米）
  
  # 清扫装置
  cleaning:
    brush_motor_pin: 18      # PWM控制引脚
    suction_relay_pin: 19    # 吸尘器继电器控制引脚
    brush_speed_max: 255     # PWM最大值
```

### 4. 硬件适配启动文件

```python
# launch/hardware_robot.launch.py
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 加载硬件配置
    config = os.path.join(
        get_package_share_directory('cleaning_robot_description'),
        'config', 'hardware_config.yaml'
    )
    
    return LaunchDescription([
        # 电机控制器节点
        Node(
            package='cleaning_robot_hardware',
            executable='motor_controller',
            name='motor_controller',
            parameters=[config],
            output='screen'
        ),
        
        # 激光雷达驱动节点
        Node(
            package='cleaning_robot_hardware', 
            executable='lidar_driver',
            name='lidar_driver',
            parameters=[config],
            output='screen'
        ),
        
        # 摄像头驱动节点
        Node(
            package='cleaning_robot_hardware',
            executable='camera_driver', 
            name='camera_driver',
            parameters=[config],
            output='screen'
        ),
        
        # 清扫装置控制节点
        Node(
            package='cleaning_robot_hardware',
            executable='cleaning_hardware',
            name='cleaning_hardware', 
            parameters=[config],
            output='screen'
        ),
    ])
```

### 5. 提供硬件信息的方式

请通过以下方式提供您的实物硬件信息：

1. **硬件照片**: 整车照片、各个组件特写
2. **技术规格书**: 电机、雷达、摄像头的详细参数
3. **接线图**: 硬件连接示意图
4. **现有代码**: 如果已有控制代码，请提供
5. **测试视频**: 硬件运行状态视频

### 6. 接下来的步骤

1. **提供硬件信息**: 请详细描述您的硬件配置
2. **创建硬件包**: 我将为您创建专门的硬件驱动包
3. **接口适配**: 修改现有代码以适配实物硬件
4. **测试验证**: 逐步测试各个硬件组件
5. **系统集成**: 整合所有硬件形成完整系统

请先提供您的具体硬件信息，我将为您定制专门的驱动程序！ 