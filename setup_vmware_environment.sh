#!/bin/bash

# VMware Ubuntu 22.04 清扫机器人环境自动安装脚本
# 适用于从WSL迁移到VMware的环境配置

echo "🚀 VMware Ubuntu 22.04 清扫机器人环境安装"
echo "=================================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then 
    echo -e "${RED}请不要使用root用户运行此脚本${NC}"
    exit 1
fi

# 创建日志目录
mkdir -p ~/setup_logs
LOG_FILE=~/setup_logs/vmware_setup_$(date +%Y%m%d_%H%M%S).log

# 记录日志函数
log() {
    echo -e "$1" | tee -a "$LOG_FILE"
}

log "${GREEN}开始VMware Ubuntu环境配置...${NC}"
log "日志文件: $LOG_FILE"

# 步骤1: 系统更新
log "\n${BLUE}📦 步骤1: 更新系统包${NC}"
sudo apt update && sudo apt upgrade -y 2>&1 | tee -a "$LOG_FILE"

# 步骤2: 安装VMware Tools
log "\n${BLUE}🛠️ 步骤2: 安装VMware Tools${NC}"
sudo apt install -y open-vm-tools open-vm-tools-desktop 2>&1 | tee -a "$LOG_FILE"

# 步骤3: 安装基础开发工具
log "\n${BLUE}🔧 步骤3: 安装基础开发工具${NC}"
sudo apt install -y \
    curl \
    wget \
    git \
    vim \
    nano \
    htop \
    tree \
    unzip \
    tar \
    software-properties-common \
    build-essential \
    cmake \
    python3-pip \
    python3-dev \
    python3-venv \
    2>&1 | tee -a "$LOG_FILE"

# 步骤4: 设置Git（如果需要）
log "\n${BLUE}🔧 步骤4: 配置Git${NC}"
read -p "是否配置Git用户信息？(y/n): " setup_git
if [ "$setup_git" = "y" ]; then
    read -p "输入Git用户名: " git_username
    read -p "输入Git邮箱: " git_email
    git config --global user.name "$git_username"
    git config --global user.email "$git_email"
    log "Git配置完成: $git_username <$git_email>"
fi

# 步骤5: 安装ROS2 Humble
log "\n${BLUE}🤖 步骤5: 安装ROS2 Humble${NC}"

# 添加ROS2源
sudo add-apt-repository universe -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list

# 更新包列表并安装ROS2
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-base 2>&1 | tee -a "$LOG_FILE"

# 安装colcon编译工具
sudo apt install -y python3-colcon-common-extensions 2>&1 | tee -a "$LOG_FILE"

# 步骤6: 安装ROS2依赖包
log "\n${BLUE}📦 步骤6: 安装ROS2项目依赖${NC}"
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-geometry \
    ros-humble-camera-calibration-parsers \
    ros-humble-launch-xml \
    ros-humble-launch-yaml \
    ros-humble-diagnostic-updater \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    2>&1 | tee -a "$LOG_FILE"

# 步骤7: 安装OpenCV和图像处理依赖
log "\n${BLUE}📷 步骤7: 安装OpenCV和图像处理库${NC}"
sudo apt install -y \
    python3-opencv \
    libopencv-dev \
    python3-numpy \
    python3-matplotlib \
    libpcap-dev \
    libeigen3-dev \
    libboost-all-dev \
    2>&1 | tee -a "$LOG_FILE"

# 步骤8: 设置串口权限
log "\n${BLUE}🔌 步骤8: 设置硬件设备权限${NC}"
sudo usermod -a -G dialout,video,audio,plugdev "$USER"
log "用户 $USER 已添加到硬件设备组"

# 步骤9: 设置ROS2环境变量
log "\n${BLUE}🌐 步骤9: 配置ROS2环境变量${NC}"
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    log "ROS2环境变量已添加到~/.bashrc"
fi

# 设置ROS域ID
if ! grep -q "export ROS_DOMAIN_ID" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
    log "ROS_DOMAIN_ID=0 已设置"
fi

# 步骤10: 创建工作目录
log "\n${BLUE}📁 步骤10: 创建项目工作目录${NC}"
mkdir -p ~/cleaning_robot_ws/src
log "创建工作目录: ~/cleaning_robot_ws"

# 步骤11: 安装网络工具
log "\n${BLUE}🌐 步骤11: 安装网络诊断工具${NC}"
sudo apt install -y \
    net-tools \
    netcat \
    tcpdump \
    iftop \
    nmap \
    wireshark-common \
    2>&1 | tee -a "$LOG_FILE"

# 步骤12: 安装多媒体工具
log "\n${BLUE}🎥 步骤12: 安装多媒体工具${NC}"
sudo apt install -y \
    cheese \
    v4l-utils \
    ffmpeg \
    2>&1 | tee -a "$LOG_FILE"

# 步骤13: 安装可选的性能监控工具
log "\n${BLUE}📊 步骤13: 安装系统监控工具${NC}"
sudo apt install -y \
    iotop \
    iftop \
    nethogs \
    glances \
    2>&1 | tee -a "$LOG_FILE"

# 步骤14: 配置防火墙
log "\n${BLUE}🔥 步骤14: 配置防火墙规则${NC}"
read -p "是否配置防火墙规则以支持激光雷达网络通信？(y/n): " setup_firewall
if [ "$setup_firewall" = "y" ]; then
    sudo ufw allow 2368/udp  # N10P数据端口
    sudo ufw allow 2369/udp  # N10P控制端口
    sudo ufw allow ssh       # SSH访问
    log "防火墙规则已配置"
fi

# 步骤15: 创建桌面快捷方式
log "\n${BLUE}🖥️ 步骤15: 创建桌面快捷方式${NC}"

# 创建终端快捷方式
cat > ~/Desktop/ROS2_Terminal.desktop << EOF
[Desktop Entry]
Name=ROS2 Terminal
Comment=Open terminal with ROS2 environment
Exec=gnome-terminal --working-directory=/home/$USER/cleaning_robot_ws -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash 2>/dev/null; exec bash"
Icon=utilities-terminal
Terminal=false
Type=Application
Categories=Development;
EOF

chmod +x ~/Desktop/ROS2_Terminal.desktop

# 创建项目目录快捷方式
cat > ~/Desktop/Cleaning_Robot_Workspace.desktop << EOF
[Desktop Entry]
Name=Cleaning Robot Workspace
Comment=Open cleaning robot workspace
Exec=nautilus /home/$USER/cleaning_robot_ws
Icon=folder
Terminal=false
Type=Application
Categories=Utility;
EOF

chmod +x ~/Desktop/Cleaning_Robot_Workspace.desktop

log "桌面快捷方式已创建"

# 步骤16: 系统优化设置
log "\n${BLUE}⚡ 步骤16: 系统性能优化${NC}"

# 增加交换文件大小（如果需要）
if [ $(free -m | awk 'NR==3{print $2}') -lt 2048 ]; then
    read -p "检测到内存较小，是否创建2GB交换文件？(y/n): " create_swap
    if [ "$create_swap" = "y" ]; then
        sudo fallocate -l 2G /swapfile
        sudo chmod 600 /swapfile
        sudo mkswap /swapfile
        sudo swapon /swapfile
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
        log "2GB交换文件已创建"
    fi
fi

# 设置网络优化
echo 'net.core.rmem_default = 262144' | sudo tee -a /etc/sysctl.conf
echo 'net.core.rmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_default = 262144' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' | sudo tee -a /etc/sysctl.conf

# 步骤17: 硬件检测脚本
log "\n${BLUE}🔍 步骤17: 创建硬件检测脚本${NC}"

cat > ~/cleaning_robot_ws/check_hardware.sh << 'EOF'
#!/bin/bash

echo "🔍 硬件设备检测报告"
echo "===================="

echo "📡 串口设备:"
ls -la /dev/ttyUSB* 2>/dev/null || echo "未发现串口设备"

echo ""
echo "📷 摄像头设备:"
ls -la /dev/video* 2>/dev/null || echo "未发现摄像头设备"

echo ""
echo "🌐 网络接口:"
ip addr show | grep -E '^[0-9]+:' | awk '{print $2}' | sed 's/://'

echo ""
echo "🔌 USB设备:"
lsusb

echo ""
echo "💾 磁盘空间:"
df -h | grep -E '^/dev/'

echo ""
echo "🧠 内存使用:"
free -h

echo ""
echo "🖥️ CPU信息:"
lscpu | grep "Model name"

echo ""
echo "🌐 网络连通性测试:"
ping -c 1 8.8.8.8 >/dev/null 2>&1 && echo "✅ 互联网连接正常" || echo "❌ 互联网连接异常"

echo ""
echo "🤖 ROS2环境:"
source /opt/ros/humble/setup.bash 2>/dev/null
if command -v ros2 >/dev/null 2>&1; then
    echo "✅ ROS2已安装: $(ros2 --version 2>/dev/null | head -n1)"
else
    echo "❌ ROS2未找到"
fi
EOF

chmod +x ~/cleaning_robot_ws/check_hardware.sh

# 步骤18: 项目迁移助手脚本
log "\n${BLUE}📦 步骤18: 创建项目迁移助手${NC}"

cat > ~/cleaning_robot_ws/migrate_project.sh << 'EOF'
#!/bin/bash

echo "📦 清扫机器人项目迁移助手"
echo "=========================="

echo "选择迁移方式:"
echo "1. 从共享文件夹复制"
echo "2. 从Git仓库克隆"
echo "3. 从tar.gz包解压"
echo "4. 手动指定源目录"

read -p "请选择 [1-4]: " choice

case $choice in
    1)
        echo "📁 从共享文件夹复制..."
        if [ -d "/mnt/hgfs" ]; then
            echo "共享文件夹可用，请指定源路径:"
            ls /mnt/hgfs/
            read -p "输入项目路径: " source_path
            cp -r "/mnt/hgfs/$source_path"/* .
            chmod +x *.sh
            echo "✅ 复制完成"
        else
            echo "❌ 共享文件夹未挂载，请先设置VMware共享文件夹"
        fi
        ;;
    2)
        echo "🌐 从Git仓库克隆..."
        read -p "输入Git仓库URL: " git_url
        git clone "$git_url" temp_clone
        mv temp_clone/* .
        mv temp_clone/.* . 2>/dev/null
        rmdir temp_clone
        chmod +x *.sh
        echo "✅ 克隆完成"
        ;;
    3)
        echo "📦 从tar.gz包解压..."
        read -p "输入tar.gz文件路径: " tar_path
        tar -xzf "$tar_path"
        chmod +x *.sh
        echo "✅ 解压完成"
        ;;
    4)
        echo "📁 从指定目录复制..."
        read -p "输入源目录路径: " source_dir
        cp -r "$source_dir"/* .
        chmod +x *.sh
        echo "✅ 复制完成"
        ;;
esac

echo ""
echo "🔨 开始编译项目..."
colcon build
source install/setup.bash

echo ""
echo "✅ 项目迁移完成！"
echo "运行 ./check_hardware.sh 检查硬件状态"
EOF

chmod +x ~/cleaning_robot_ws/migrate_project.sh

# 完成安装
log "\n${GREEN}🎉 VMware Ubuntu环境安装完成！${NC}"
log "=================================="

log "\n📋 安装总结:"
log "✅ Ubuntu 22.04系统已更新"
log "✅ VMware Tools已安装"
log "✅ ROS2 Humble已安装"
log "✅ 开发工具已安装"
log "✅ 硬件设备权限已配置"
log "✅ 网络工具已安装"
log "✅ 桌面快捷方式已创建"
log "✅ 辅助脚本已创建"

log "\n🚀 下一步操作:"
log "1. 重新登录或重启系统以使权限生效"
log "2. 进入项目目录: cd ~/cleaning_robot_ws"
log "3. 运行迁移助手: ./migrate_project.sh"
log "4. 检查硬件状态: ./check_hardware.sh"
log "5. 连接硬件设备并开始调试"

log "\n📁 重要文件位置:"
log "• 工作目录: ~/cleaning_robot_ws"
log "• 硬件检测: ~/cleaning_robot_ws/check_hardware.sh"
log "• 迁移助手: ~/cleaning_robot_ws/migrate_project.sh"
log "• 安装日志: $LOG_FILE"

log "\n🔧 VMware硬件配置提醒:"
log "• USB控制器: 设置为USB 3.1"
log "• 网络适配器: 桥接网络（用于激光雷达）"
log "• 串口设备: 虚拟机->可移动设备->连接"
log "• 摄像头: 虚拟机->可移动设备->连接"

echo ""
echo -e "${YELLOW}⚠️  重要提醒: 需要重新登录系统以使硬件权限生效！${NC}"
echo -e "${GREEN}安装完成后请重启虚拟机，然后运行迁移助手。${NC}"

# 询问是否立即重启
read -p "是否现在重启系统？(y/n): " restart_now
if [ "$restart_now" = "y" ]; then
    log "正在重启系统..."
    sudo reboot
fi 