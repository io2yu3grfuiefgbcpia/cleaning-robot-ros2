#!/bin/bash

echo "🎥 安装奥比中光(Orbbec)深度相机依赖"
echo "========================================"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "正在设置ROS2环境..."
    source /opt/ros/humble/setup.bash
fi

echo "📦 安装系统依赖..."

# 安装基础依赖
sudo apt update
sudo apt install -y \
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-publisher \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs

echo "✅ 基础依赖安装完成"

# 安装额外的深度相机相关依赖
sudo apt install -y \
    libudev-dev \
    pkg-config \
    libusb-1.0-0-dev \
    libeigen3-dev

echo "✅ 深度相机依赖安装完成"

# 创建udev规则目录（如果不存在）
sudo mkdir -p /etc/udev/rules.d/

echo "🔧 设置USB设备权限..."
# 创建Orbbec设备的udev规则
sudo tee /etc/udev/rules.d/56-orbbec-usb.rules > /dev/null << 'EOF'
# Orbbec depth cameras
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666", GROUP="plugdev"
# Orbbec Astra
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0403", MODE="0666", GROUP="plugdev"
# Orbbec Gemini
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0501", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0502", MODE="0666", GROUP="plugdev"
EOF

# 重新加载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "✅ USB设备权限设置完成"

# 将用户添加到plugdev组
sudo usermod -a -G plugdev $USER

echo "⚠️  请注销并重新登录以使组权限生效"
echo "�� Orbbec深度相机依赖安装完成！" 