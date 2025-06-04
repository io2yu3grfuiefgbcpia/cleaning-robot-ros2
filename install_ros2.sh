#!/bin/bash

# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装必要的工具
sudo apt install -y curl gnupg2 lsb-release software-properties-common

# 添加ROS2 GPG密钥
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加ROS2仓库
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 更新包列表
sudo apt update

# 安装ROS2 Humble桌面版
sudo apt install -y ros-humble-desktop

# 安装开发工具
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# 初始化rosdep
sudo rosdep init
rosdep update

# 安装清扫机器人相关的包
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-stereo-image-proc \
    ros-humble-depth-image-proc \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-laser-geometry \
    ros-humble-pcl-ros \
    ros-humble-octomap-ros \
    ros-humble-octomap-msgs

# 设置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/cleaning_robot_ws/install/setup.bash" >> ~/.bashrc

echo "ROS2 Humble安装完成！请重新启动终端或运行 'source ~/.bashrc'"
