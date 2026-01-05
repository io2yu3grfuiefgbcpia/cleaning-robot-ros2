#!/bin/bash

# 清扫机器人完整系统一键启动脚本
# 功能：启动建图、导航、摄像头、路径规划、可视化界面

echo "🤖 清扫机器人完整系统启动"
echo "=================================="

# 设置颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# 设置工作目录
WORKSPACE_DIR="/home/yys/cleaning_robot_ws"
cd "$WORKSPACE_DIR"

# 设置ROS2环境
echo -e "${YELLOW}📦 设置ROS2环境...${NC}"
source /opt/ros/humble/setup.bash

if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo -e "${GREEN}✅ 工作空间环境已加载${NC}"
else
    echo -e "${RED}❌ 未找到install/setup.bash，请先编译项目${NC}"
    echo "   运行: colcon build"
    exit 1
fi

# 环境变量
export ROS_DOMAIN_ID=0

# 显示选项
echo ""
echo -e "${BLUE}选择启动模式:${NC}"
echo "1. 完整系统 + RViz可视化（推荐）"
echo "2. 完整系统 + 控制面板"
echo "3. 完整系统 + RViz + 控制面板"
echo "4. 仅启动激光雷达和建图"
echo "5. 仅启动可视化控制面板"
echo ""
read -p "请选择 [1-5，默认1]: " choice
choice=${choice:-1}

case $choice in
    1)
        echo -e "${GREEN}🚀 启动完整系统 + RViz...${NC}"
        ros2 launch cleaning_robot_description cleaning_robot_full_system.launch.py \
            use_rviz:=true \
            use_dashboard:=false
        ;;
    2)
        echo -e "${GREEN}🚀 启动完整系统 + 控制面板...${NC}"
        ros2 launch cleaning_robot_description cleaning_robot_full_system.launch.py \
            use_rviz:=false \
            use_dashboard:=true
        ;;
    3)
        echo -e "${GREEN}🚀 启动完整系统 + RViz + 控制面板...${NC}"
        ros2 launch cleaning_robot_description cleaning_robot_full_system.launch.py \
            use_rviz:=true \
            use_dashboard:=true
        ;;
    4)
        echo -e "${GREEN}🚀 仅启动激光雷达和建图...${NC}"
        ros2 launch cleaning_robot_navigation cleaning_robot_n10p_net.launch.py
        ;;
    5)
        echo -e "${GREEN}🚀 启动可视化控制面板...${NC}"
        ros2 run cleaning_robot_control cleaning_dashboard
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac

