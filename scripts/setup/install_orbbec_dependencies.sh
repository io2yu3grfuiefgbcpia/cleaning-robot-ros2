#!/bin/bash

echo "🔧 安装Orbbec相机依赖"
echo "===================="

# 添加Orbbec源
echo "📦 添加Orbbec软件源..."
sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y ppa:orbbec/ros2

# 安装依赖包
echo "📥 安装依赖包..."
sudo apt-get update
sudo apt-get install -y \
    ros-humble-orbbec-camera \
    ros-humble-orbbec-camera-msgs \
    ros-humble-orbbec-camera-utils

# 安装udev规则
echo "🔑 配置udev规则..."
sudo bash -c 'cat > /etc/udev/rules.d/99-orbbec-camera.rules << EOF
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0402", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0403", MODE="0666"
EOF'

# 重新加载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 克隆Orbbec ROS2包
echo "📥 克隆Orbbec ROS2包..."
cd ~/cleaning_robot_ws/src
if [ ! -d "orbbec_camera_ros2" ]; then
    git clone https://github.com/orbbec/orbbec_camera_ros2.git
fi

# 编译工作空间
echo "🔨 编译工作空间..."
cd ~/cleaning_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "✅ Orbbec相机依赖安装完成！"
echo "请重新插拔相机设备，然后运行以下命令检查设备："
echo "ls -l /dev/video*"