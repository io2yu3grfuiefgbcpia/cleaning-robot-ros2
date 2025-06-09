# 奥比中光深度相机集成指南

本指南将帮助您将奥比中光(Orbbec)深度相机完整集成到清扫机器人系统中。

## 📋 前提条件

- Ubuntu 20.04/22.04
- ROS2 Humble
- 奥比中光深度相机（如Gemini2、Astra系列等）
- USB 3.0 接口

## 🔧 安装步骤

### 1. 安装系统依赖

```bash
# 运行依赖安装脚本
bash install_orbbec_dependencies.sh

# 或手动安装
sudo apt update
sudo apt install -y \
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    libudev-dev \
    pkg-config \
    libusb-1.0-0-dev \
    libeigen3-dev
```

### 2. 安装Orbbec ROS2驱动包

```bash
# 进入工作空间源码目录
cd ~/cleaning_robot_ws/src

# 克隆Orbbec ROS2包
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git orbbec_camera

# 编译
cd ~/cleaning_robot_ws
colcon build --packages-select orbbec_camera
source install/setup.bash
```

### 3. 设置USB权限

udev规则已通过依赖安装脚本自动设置，如需手动设置：

```bash
# 创建udev规则文件
sudo nano /etc/udev/rules.d/56-orbbec-usb.rules

# 添加以下内容：
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666", GROUP="plugdev"

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 添加用户到plugdev组
sudo usermod -a -G plugdev $USER
```

### 4. 编译清扫机器人项目

```bash
cd ~/cleaning_robot_ws
colcon build
source install/setup.bash
```

## 🚀 使用方法

### 基础测试

#### 1. 检查相机连接

```bash
# 检查USB设备
lsusb | grep 2bc5

# 应该看到类似输出：
# Bus 001 Device 003: ID 2bc5:0501 Unknown
```

#### 2. 启动Orbbec相机驱动

```bash
# 启动基础相机驱动
ros2 launch orbbec_camera gemini2.launch.py

# 或启动清扫机器人集成版本
ros2 run cleaning_robot_perception orbbec_camera_node
```

#### 3. 查看话题列表

```bash
ros2 topic list | grep orbbec
```

应该看到以下话题：
- `/cleaning_robot/orbbec_camera/color/image_raw`
- `/cleaning_robot/orbbec_camera/depth/image_raw`
- `/cleaning_robot/orbbec_camera/pointcloud`
- `/cleaning_robot/orbbec_camera/color/camera_info`
- `/cleaning_robot/orbbec_camera/depth/camera_info`

#### 4. 测试相机功能

```bash
# 运行测试脚本
python3 test_orbbec_camera.py
```

### 完整系统启动

#### 1. 启动完整清扫机器人系统（包含Orbbec相机）

```bash
bash start_cleaning_robot_orbbec.sh
```

#### 2. 启动可视化界面

```bash
# 查看彩色图像
ros2 run rqt_image_view rqt_image_view /cleaning_robot/orbbec_camera/color/image_raw

# 查看深度图像
ros2 run rqt_image_view rqt_image_view /cleaning_robot/orbbec_camera/depth/image_raw

# 启动RViz查看所有数据
ros2 launch cleaning_robot_description cleaning_robot_with_orbbec.launch.py
```

## 📊 功能特性

### 深度相机数据处理

- **彩色图像**: 640x480@30fps RGB图像
- **深度图像**: 640x480@30fps 16位深度图
- **点云数据**: 彩色点云，支持障碍物检测
- **相机标定**: 自动获取内参和畸变参数

### 集成功能

- **坐标系变换**: 自动发布相机到机器人基座的TF变换
- **话题重映射**: 统一到`/cleaning_robot`命名空间
- **障碍物检测**: 基于深度信息的实时障碍物检测
- **SLAM集成**: 深度数据可用于增强SLAM精度

## 🛠️ 配置参数

### 相机参数配置

在启动文件中可以配置以下参数：

```python
parameters=[{
    'camera_name': 'orbbec',
    'enable_color': True,
    'enable_depth': True,  
    'enable_pointcloud': True,
    'color_width': 640,
    'color_height': 480,
    'depth_width': 640,
    'depth_height': 480,
    'color_fps': 30,
    'depth_fps': 30
}]
```

### 安装位置调整

在`orbbec_camera_node.py`中调整相机安装位置：

```python
# 相机安装位置（根据实际安装位置调整）
base_to_camera.transform.translation.x = 0.2  # 前方20cm
base_to_camera.transform.translation.y = 0.0  # 中央
base_to_camera.transform.translation.z = 0.15 # 高15cm
```

## 🔍 故障排除

### 常见问题

#### 1. 相机无法识别
```bash
# 检查USB连接
lsusb | grep 2bc5

# 检查权限
ls -la /dev/bus/usb/*/*

# 重新设置权限
sudo chmod 666 /dev/bus/usb/*/*
```

#### 2. 无数据输出
```bash
# 检查节点状态
ros2 node list | grep orbbec

# 检查话题数据
ros2 topic echo /cleaning_robot/orbbec_camera/color/image_raw --once

# 查看日志
tail -f logs/orbbec_camera.log
```

#### 3. 深度数据异常
- 检查环境光照条件
- 确认目标物体在有效检测范围内（0.05-8.0米）
- 检查相机镜头是否清洁

#### 4. 点云数据缺失
- 确认enable_pointcloud参数为true
- 检查彩色和深度图像是否正常
- 验证相机标定信息

### 性能优化

#### 降低计算负载
```python
# 在depth_camera_processor.py中调整采样率
step = 4  # 增大跳过像素数量
```

#### 调整检测范围
```python
# 修改障碍物检测参数
obstacle_threshold = 2.0  # 增大检测距离阈值
roi_height = int(height * 0.6)  # 减小检测区域
```

## 📈 集成效果

成功集成后，您的清扫机器人将具备：

1. **精确的深度感知**: 0.05-8米范围内的精确深度测量
2. **彩色点云生成**: 用于更好的环境理解
3. **实时障碍物检测**: 提高导航安全性
4. **增强SLAM性能**: 结合激光雷达提供更准确的建图
5. **可视化监控**: 完整的RViz集成显示

## 📞 技术支持

如遇到问题，请：

1. 查看日志文件：`tail -f logs/*.log`
2. 运行测试脚本：`python3 test_orbbec_camera.py`
3. 检查硬件连接和权限设置
4. 参考Orbbec官方文档

---

祝您成功集成Orbbec深度相机到清扫机器人系统中！🎉 