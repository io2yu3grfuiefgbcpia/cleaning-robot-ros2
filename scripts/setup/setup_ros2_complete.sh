#!/bin/bash

# =============================================================================
# ROS2 Humble 完整环境搭建脚本
# 适用于: Ubuntu 22.04
# 功能: 自动安装和配置ROS2 Humble + Python客户端库 + 开发工具
# =============================================================================

set -e  # 遇到错误立即退出

echo "🚀 开始安装ROS2 Humble完整环境..."
echo "=============================================="

# 检查Ubuntu版本
echo "📋 检查系统版本..."
if [ "$(lsb_release -cs)" != "jammy" ]; then
    echo "⚠️  警告: 此脚本为Ubuntu 22.04 (jammy)设计"
    echo "您的系统版本: $(lsb_release -ds)"
    read -p "是否继续安装? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "❌ 安装已取消"
        exit 1
    fi
fi

# 1. 更新系统
echo ""
echo "📦 更新系统包..."
sudo apt update
sudo apt upgrade -y

# 2. 安装必要工具
echo ""
echo "🔧 安装基础工具..."
sudo apt install -y \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    wget \
    ca-certificates

# 3. 添加ROS2仓库
echo ""
echo "🔑 添加ROS2官方仓库..."

# 添加ROS2 GPG密钥
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加ROS2仓库源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 更新包列表
sudo apt update

# 4. 安装ROS2 Humble
echo ""
echo "🤖 安装ROS2 Humble完整版..."
sudo apt install -y ros-humble-desktop-full

# 5. 安装Python客户端库
echo ""
echo "🐍 安装ROS2 Python客户端库..."
sudo apt install -y \
    python3-rclpy \
    python3-rclpy-common-interfaces \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs

# 6. 安装开发工具
echo ""
echo "🛠️  安装ROS2开发工具..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro

# 7. 安装可视化工具依赖
echo ""
echo "📊 安装Python可视化库..."
sudo apt install -y \
    python3-pip \
    python3-matplotlib \
    python3-numpy \
    python3-scipy

pip3 install --user matplotlib numpy scipy

# 8. 初始化rosdep
echo ""
echo "🔄 初始化rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 9. 设置环境变量
echo ""
echo "⚙️  配置环境变量..."

# 添加到bashrc
BASHRC_CONTENT="
# ROS2 Humble 环境配置
source /opt/ros/humble/setup.bash

# 自动补全
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# ROS2域ID (避免冲突)
export ROS_DOMAIN_ID=42

# 禁用DDS日志
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

echo \"✅ ROS2 Humble环境已自动加载\"
"

# 检查bashrc中是否已经有ROS2配置
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "📝 添加ROS2环境到~/.bashrc..."
    echo "$BASHRC_CONTENT" >> ~/.bashrc
else
    echo "✅ ~/.bashrc中已包含ROS2配置"
fi

# 10. 创建工作空间结构
echo ""
echo "📁 配置工作空间..."
cd ~/cleaning_robot_ws

# 如果没有src目录，创建它
if [ ! -d "src" ]; then
    mkdir -p src
fi

# 构建工作空间
echo "🔨 构建工作空间..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 11. 测试安装
echo ""
echo "🧪 测试ROS2安装..."

# 临时设置环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 测试Python导入
echo "测试Python ROS2模块..."
python3 -c "
import rclpy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
print('✅ 所有ROS2 Python模块导入成功')
"

# 测试命令行工具
echo "测试ROS2命令行工具..."
timeout 5s ros2 topic list || echo "ROS2命令测试完成"

# 12. 创建便捷脚本
echo ""
echo "📜 创建便捷启动脚本..."

cat > ~/start_ros2_env.sh << 'EOF'
#!/bin/bash
# ROS2环境快速启动脚本

echo "🚀 启动ROS2环境..."
source /opt/ros/humble/setup.bash

if [ -f ~/cleaning_robot_ws/install/setup.bash ]; then
    source ~/cleaning_robot_ws/install/setup.bash
    echo "✅ 工作空间环境已加载"
else
    echo "⚠️  工作空间未构建，请先运行: cd ~/cleaning_robot_ws && colcon build"
fi

echo "📡 可用话题列表:"
ros2 topic list

echo ""
echo "🎯 常用命令:"
echo "  ros2 topic list                    # 查看话题"
echo "  ros2 topic echo /cleaning_robot/scan # 查看激光雷达数据"
echo "  rviz2                              # 启动可视化工具"
echo "  python3 real_lidar_rviz.py        # 启动自定义可视化"
EOF

chmod +x ~/start_ros2_env.sh

# 13. 安装完成
echo ""
echo "=============================================="
echo "🎉 ROS2 Humble环境安装完成!"
echo "=============================================="
echo ""
echo "📋 安装内容:"
echo "  ✅ ROS2 Humble Desktop Full"
echo "  ✅ Python客户端库 (rclpy)"
echo "  ✅ 开发工具 (colcon, rosdep)"
echo "  ✅ 可视化工具 (RViz2)"
echo "  ✅ Python可视化库 (matplotlib, numpy)"
echo "  ✅ 自动环境配置"
echo ""
echo "🚀 使用方法:"
echo "  1. 重启终端或运行: source ~/.bashrc"
echo "  2. 或使用快捷脚本: ~/start_ros2_env.sh"
echo "  3. 测试: ros2 topic list"
echo "  4. 查看激光雷达: python3 real_lidar_rviz.py"
echo ""
echo "📁 工作空间位置: ~/cleaning_robot_ws"
echo "🔧 快捷脚本: ~/start_ros2_env.sh"
echo ""
echo "⚠️  注意: 请重启终端以使环境变量生效!"
echo "=============================================="

# 询问是否立即重新加载环境
echo ""
read -p "是否现在重新加载环境? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🔄 重新加载环境..."
    exec bash
fi

echo "✅ 安装脚本执行完成!" 