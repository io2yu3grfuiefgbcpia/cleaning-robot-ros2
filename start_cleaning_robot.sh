#!/bin/bash

# 清扫机器人启动脚本
# 使用方法: ./start_cleaning_robot.sh [选项]
# 选项:
#   complete    - 启动完整的仿真系统（Gazebo + RViz2）
#   hardware    - 启动实物硬件系统（镭神N10P激光雷达）
#   test        - 启动清扫测试
#   rviz        - 仅启动RViz2可视化

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🤖 清扫机器人系统启动脚本${NC}"
echo "=========================================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}正在设置ROS2环境...${NC}"
    source /opt/ros/humble/setup.bash
    source install/setup.bash 2>/dev/null || echo -e "${YELLOW}注意：工作空间尚未编译${NC}"
else
    echo -e "${GREEN}✓ ROS2环境已就绪 ($ROS_DISTRO)${NC}"
    source install/setup.bash 2>/dev/null || echo -e "${YELLOW}注意：工作空间尚未编译${NC}"
fi

# 函数：检查激光雷达连接
check_lidar_connection() {
    echo -e "${BLUE}检查镭神N10P激光雷达连接...${NC}"
    
    if [ -e /dev/ttyUSB0 ] || [ -e /dev/ttyUSB1 ] || [ -e /dev/lslidar_n10p ]; then
        echo -e "${GREEN}✓ 检测到串口设备${NC}"
        ls -la /dev/ttyUSB* 2>/dev/null
        ls -la /dev/lslidar_n10p 2>/dev/null
        return 0
    else
        echo -e "${RED}✗ 未检测到激光雷达设备${NC}"
        echo "请检查："
        echo "1. 激光雷达是否连接并通电"
        echo "2. USB串口线是否正常"
        echo "3. 运行串口配置脚本: ./setup_lslidar_serial.sh"
        return 1
    fi
}

# 函数：启动完整仿真系统
start_complete_simulation() {
    echo -e "${GREEN}🚀 启动完整仿真系统（Gazebo + RViz2）${NC}"
    ros2 launch cleaning_robot_description cleaning_robot_complete.launch.py
}

# 函数：启动实物硬件系统
start_hardware_system() {
    echo -e "${GREEN}🔧 启动实物硬件系统（镭神N10P激光雷达）${NC}"
    
    if ! check_lidar_connection; then
        echo -e "${RED}激光雷达连接检查失败，请先解决硬件连接问题${NC}"
        exit 1
    fi
    
    ros2 launch cleaning_robot_description cleaning_robot_with_lslidar.launch.py
}

# 函数：启动清扫测试
start_cleaning_test() {
    echo -e "${GREEN}🧹 启动清扫测试${NC}"
    sleep 3
    ros2 run cleaning_robot_control test_cleaning
}

# 函数：仅启动RViz2
start_rviz_only() {
    echo -e "${GREEN}📊 启动RViz2可视化${NC}"
    ros2 run rviz2 rviz2 -d src/cleaning_robot_description/rviz/cleaning_robot.rviz
}

# 函数：显示帮助信息
show_help() {
    echo "使用方法: $0 [选项]"
    echo ""
    echo "可用选项:"
    echo "  complete    启动完整的仿真系统（Gazebo + RViz2）"
    echo "  hardware    启动实物硬件系统（镭神N10P激光雷达）"
    echo "  test        启动清扫测试（需要先启动主系统）"
    echo "  rviz        仅启动RViz2可视化"
    echo "  help        显示此帮助信息"
    echo ""
    echo "硬件系统使用前准备:"
    echo "1. 连接镭神N10P激光雷达到USB串口"
    echo "2. 运行串口配置: ./setup_lslidar_serial.sh"
    echo "3. 启动硬件系统: $0 hardware"
    echo ""
    echo "示例："
    echo "  $0 complete    # 仿真测试"
    echo "  $0 hardware    # 实物测试"
    echo "  $0 test        # 清扫测试（在新终端运行）"
}

# 主程序
case "$1" in
    "complete")
        start_complete_simulation
        ;;
    "hardware")
        start_hardware_system
        ;;
    "test")
        start_cleaning_test
        ;;
    "rviz")
        start_rviz_only
        ;;
    "help")
        show_help
        ;;
    "")
        echo -e "${YELLOW}未指定选项，显示帮助信息：${NC}"
        echo ""
        show_help
        ;;
    *)
        echo -e "${RED}未知选项: $1${NC}"
        echo ""
        show_help
        exit 1
        ;;
esac 