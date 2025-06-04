# 清扫机器人ROS2项目 🤖

一个基于ROS2的自主清扫机器人系统，集成了**镭神N10P激光雷达**（串口版和网络版）、双目立体视觉和智能路径规划功能。

## ✨ 主要特性

- 🤖 **完整的机器人系统**: 差分驱动底盘，支持仿真和实物部署
- 🎯 **多传感器融合**: 
  - **镭神N10P激光雷达**: 
    - 🔌 **串口版**: 串口连接，适用于USB转串口场景
    - 🌐 **网络版**: 以太网连接，更高数据传输速率和稳定性
    - 360°扫描，20米测距，10Hz刷新率
  - **双目立体视觉**: 深度感知和障碍物检测
  - **IMU传感器**: 姿态估计和里程计补偿
- 🗺️ **智能SLAM建图**: 多传感器SLAM，实时地图构建
- 🧭 **自主导航**: ROS2 Navigation2框架，动态路径规划
- 🧹 **智能清扫**: 
  - 弓字形路径规划，完整区域覆盖
  - 障碍物智能避让
  - 清扫进度实时跟踪
- 📊 **可视化系统**: RViz2实时监控，Web界面管理

## 🏗️ 系统架构

```
清扫机器人系统
├── 硬件层
│   ├── 镭神N10P激光雷达
│   │   ├── 串口版 (/dev/ttyUSB1)
│   │   └── 网络版 (192.168.1.200:2368/2369)
│   ├── 双目摄像头 (/dev/video0, /dev/video1)
│   ├── 差分驱动底盘 (/dev/ttyUSB0)
│   └── 清扫刷电机控制
├── 驱动层
│   ├── lslidar_driver (N10P激光雷达驱动)
│   │   ├── 串口驱动 (cleaning_robot_n10p.launch.py)
│   │   └── 网络驱动 (cleaning_robot_n10p_net.launch.py)
│   ├── camera_driver (双目摄像头驱动)
│   └── motor_controller (电机控制驱动)
├── 感知层
│   ├── stereo_processor (立体视觉处理)
│   ├── slam_processor (多传感器SLAM)
│   └── obstacle_detector (障碍物检测)
├── 决策层
│   ├── path_planner (路径规划)
│   ├── navigation_controller (导航控制)
│   └── cleaning_controller (清扫控制)
└── 应用层
    ├── mission_manager (任务管理)
    ├── web_interface (Web界面)
    └── rviz_visualization (可视化界面)
```

## 📦 包结构

- `cleaning_robot_description/` - 机器人URDF模型和Gazebo仿真
- `cleaning_robot_perception/` - 双目视觉感知系统
- `cleaning_robot_slam/` - 多传感器SLAM算法
- `cleaning_robot_control/` - 清扫控制算法
- `cleaning_robot_navigation/` - 导航和路径规划
- `lslidar_driver/` - 镭神N10P激光雷达驱动
- `lslidar_msgs/` - 激光雷达消息定义

## 🚀 快速开始

### 环境要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- 镭神N10P激光雷达（串口版或网络版）
- USB双目摄像头

### 1. 安装依赖

```bash
# 运行网络修复脚本（如果apt更新失败）
./fix_wsl_network.sh

# 安装ROS2和相关包
./install_ros2.sh
```

### 2. 编译项目

```bash
# 克隆项目（如果还没有）
cd ~/cleaning_robot_ws

# 编译所有包
colcon build

# 加载环境
source install/setup.bash
```

### 3. 硬件连接和配置

#### 方式1: 串口版N10P激光雷达

```bash
# 配置激光雷达串口权限
./setup_lslidar_serial.sh

# 检查设备连接
ls -la /dev/ttyUSB*  # 激光雷达和电机控制器
```

#### 方式2: 网络版N10P激光雷达（推荐）

```bash
# 配置网络连接
# 1. 设置本机IP地址（如需要）
sudo ip addr add 192.168.1.102/24 dev eth0

# 2. 检查激光雷达连接
ping 192.168.1.200  # 默认激光雷达IP

# 3. 运行网络诊断
./diagnose_n10p_network.sh
```

**网络版配置参数:**
- 激光雷达IP: `192.168.1.200`
- 本机IP: `192.168.1.102`
- 数据端口: `2368` (UDP)
- 控制端口: `2369` (UDP)

### 4. 启动系统

#### 方式1: 网络版N10P（推荐）

```bash
# 启动完整的N10P网络版系统
./start_cleaning_robot_n10p_net.sh

# 或者指定自定义IP
LIDAR_IP=192.168.1.200 HOST_IP=192.168.1.102 ./start_cleaning_robot_n10p_net.sh
```

#### 方式2: 串口版N10P

```bash
# 启动完整的N10P串口版系统
./start_cleaning_robot_n10p.sh
```

#### 方式3: 仿真模式

```bash
# 启动Gazebo仿真
ros2 launch cleaning_robot_description gazebo.launch.py

# 启动清扫控制
ros2 run cleaning_robot_control cleaning_controller_node
```

#### 方式4: 单独启动激光雷达

**网络版:**
```bash
# 仅启动N10P激光雷达（网络版）
ros2 launch lslidar_driver cleaning_robot_n10p_net.launch.py

# 启动RViz可视化
ros2 launch cleaning_robot_navigation cleaning_robot_n10p_net.launch.py
```

**串口版:**
```bash
# 仅启动N10P激光雷达（串口版）
ros2 launch lslidar_driver cleaning_robot_n10p.launch.py

# 启动RViz可视化
ros2 launch cleaning_robot_navigation cleaning_robot_n10p.launch.py
```

### 5. 测试验证

#### 网络版测试

```bash
# 测试N10P网络版集成
python3 test_n10p_network.py

# 网络诊断工具
./diagnose_n10p_network.sh
```

#### 串口版测试

```bash
# 测试N10P串口版集成
python3 test_n10p_integration.py
```

#### 通用测试

```bash
# 测试清扫功能
python3 test_cleaning.py

# 手动控制机器人
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cleaning_robot/cmd_vel
```

### 6. 完整功能演示

```bash
# 运行交互式演示菜单（包含串口版和网络版）
./demo_n10p_complete.sh
```

## 🎮 使用指南

### 启动清扫任务

#### 网络版启动流程

1. **网络连接检查**
   ```bash
   # 检查网络连接
   ping 192.168.1.200
   
   # 运行网络诊断
   ./diagnose_n10p_network.sh
   ```

2. **启动系统**
   ```bash
   # 启动完整网络版系统
   ./start_cleaning_robot_n10p_net.sh
   
   # 等待所有节点启动完成
   ros2 node list  # 检查节点状态
   ```

3. **在RViz中设置初始位置**
   - 点击 "2D Pose Estimate" 工具
   - 在地图上点击机器人的实际位置
   - 拖拽箭头设置机器人朝向

4. **开始清扫**
   ```bash
   # 启动自动清扫
   python3 test_cleaning.py
   
   # 或者手动发送清扫命令
   ros2 topic pub /cleaning_robot/start_cleaning std_msgs/Bool "data: true"
   ```

#### 串口版启动流程

1. **准备工作**
   ```bash
   # 检查串口连接
   ls -la /dev/ttyUSB*
   
   # 启动完整串口版系统
   ./start_cleaning_robot_n10p.sh
   ```

2. **其他步骤与网络版相同**

### 常用命令

```bash
# 查看话题列表
ros2 topic list

# 查看激光雷达数据
ros2 topic echo /cleaning_robot/scan

# 查看地图数据
ros2 topic echo /map

# 查看系统状态
ros2 node list
ros2 service list

# 保存地图
ros2 run nav2_map_server map_saver_cli -f my_map

# 网络版特有命令
# 监控网络流量
sudo iftop -i eth0

# 监控UDP数据包
sudo tcpdump -i eth0 host 192.168.1.200
```

## 🛠️ 配置说明

### 网络版激光雷达配置 (N10P)

配置文件: `src/lslidar_driver/params/lidar_net_ros2/cleaning_robot_n10p_net.yaml`

```yaml
/lslidar_driver_node:
  ros__parameters: 
    frame_id: lidar_link                    # 激光坐标系
    device_ip: 192.168.1.200               # 雷达IP地址
    device_ip_difop: 192.168.1.102         # 本机IP地址  
    msop_port: 2368                        # 数据端口号
    difop_port: 2369                       # 控制端口号
    lidar_name: N10_P                      # 雷达型号
    interface_selection: net               # 网络通讯
    min_range: 0.1                         # 最小距离
    max_range: 20.0                        # 最大距离
    scan_topic: /cleaning_robot/scan       # 扫描话题
    pubScan: true                          # 发布激光扫描
    pubPointCloud2: true                   # 发布点云数据
```

### 串口版激光雷达配置 (N10P)

配置文件: `src/lslidar_driver/params/lidar_uart_ros2/cleaning_robot_n10p.yaml`

```yaml
/lslidar_driver_node:
  ros__parameters: 
    lidar_name: N10_P                      # 雷达型号
    serial_port_: /dev/ttyUSB1            # 串口设备
    interface_selection: serial           # 串口通讯
    scan_topic: /cleaning_robot/scan       # 扫描话题
    min_range: 0.1                        # 最小距离
    max_range: 20.0                       # 最大距离
```

### 清扫参数配置

配置文件: `src/cleaning_robot_control/config/cleaning_params.yaml`

```yaml
cleaning_controller:
  ros__parameters:
    cleaning_speed: 0.3        # 清扫速度 (m/s)
    turn_speed: 0.5           # 转弯速度 (rad/s)
    line_spacing: 0.8         # 清扫线间距 (m)
    obstacle_distance: 0.5    # 障碍物安全距离 (m)
```

## 🔧 故障排除

### 网络版常见问题

1. **激光雷达无法连接**
   ```bash
   # 检查网络连接
   ping 192.168.1.200
   
   # 检查IP配置
   ip addr show
   
   # 运行诊断工具
   ./diagnose_n10p_network.sh
   ```

2. **UDP端口被占用**
   ```bash
   # 检查端口占用
   netstat -tuln | grep 2368
   
   # 杀死占用端口的进程
   sudo netstat -tulnp | grep 2368
   sudo kill <PID>
   ```

3. **防火墙阻止**
   ```bash
   # 临时关闭防火墙
   sudo ufw disable
   
   # 或添加端口规则
   sudo ufw allow 2368/udp
   sudo ufw allow 2369/udp
   ```

4. **网络配置问题**
   ```bash
   # 配置静态IP
   sudo ip addr add 192.168.1.102/24 dev eth0
   sudo ip link set eth0 up
   
   # 检查路由
   ip route
   ```

### 串口版常见问题

1. **激光雷达无数据**
   ```bash
   # 检查设备连接
   ls -la /dev/ttyUSB*
   
   # 检查串口权限
   sudo chmod 666 /dev/ttyUSB1
   
   # 重启激光雷达驱动
   ros2 launch lslidar_driver cleaning_robot_n10p.launch.py
   ```

### 通用问题

1. **编译错误**
   ```bash
   # 安装缺失依赖
   sudo apt install ros-humble-diagnostic-updater libpcap-dev
   
   # 清理重新编译
   rm -rf build/ install/ log/
   colcon build
   ```

2. **节点启动失败**
   ```bash
   # 检查ROS2环境
   echo $ROS_DISTRO
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   
   # 查看详细错误信息
   ros2 launch --ros-args --log-level DEBUG
   ```

## 📊 N10P激光雷达对比

| 特性 | 串口版 | 网络版 |
|------|--------|--------|
| 连接方式 | USB转串口 | 以太网 |
| 数据传输速率 | 115200 bps | 100 Mbps |
| 传输稳定性 | 良好 | 优秀 |
| 延迟 | 低 | 极低 |
| 安装复杂度 | 简单 | 中等 |
| 适用场景 | 小型机器人 | 工业级应用 |
| 推荐使用 | 开发测试 | 生产部署 |

## 📊 项目状态

- ✅ **镭神N10P激光雷达集成** - 完成
  - ✅ **串口版驱动** - 完成
  - ✅ **网络版驱动** - 完成
  - ✅ **网络诊断工具** - 完成
- ✅ **双目立体视觉系统** - 完成  
- ✅ **多传感器SLAM建图** - 完成
- ✅ **智能清扫控制** - 完成
- ✅ **RViz2可视化界面** - 完成
- ✅ **网络环境修复** - 完成
- ✅ **硬件集成脚本** - 完成
- ✅ **测试验证工具** - 完成

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📞 支持

如有问题，请创建 GitHub Issue 或联系项目维护者。

---

**注意**: 这是一个教育和研究项目，实际部署时请确保满足安全要求和法律法规。 