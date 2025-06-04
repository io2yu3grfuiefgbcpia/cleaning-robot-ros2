#!/bin/bash

# N10P激光雷达完整功能演示脚本
# 展示清扫机器人的各项功能（包括串口版和网络版）

echo "🎯 镭神N10P激光雷达 - 完整功能演示"
echo "=================================================="
echo ""

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}正在设置ROS2环境...${NC}"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
fi

echo -e "${GREEN}✅ ROS2环境: $ROS_DISTRO${NC}"

# 功能菜单
show_menu() {
    echo ""
    echo -e "${BLUE}🎮 选择演示功能:${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "${PURPLE}📡 串口版 N10P (原版功能)${NC}"
    echo "1. 🔍 检查硬件连接状态"
    echo "2. 📡 启动N10P激光雷达(串口)"
    echo "3. 🗺️  启动SLAM建图(串口版)"
    echo "4. 🎯 启动完整系统(串口版)"
    echo "5. 🧪 运行集成测试(串口版)"
    echo ""
    echo -e "${PURPLE}🌐 网络版 N10P (新功能)${NC}"
    echo "6. 🌐 检查网络连接状态"
    echo "7. 📡 启动N10P激光雷达(网络版)"
    echo "8. 🗺️  启动SLAM建图(网络版)"
    echo "9. 🎯 启动完整系统(网络版)"
    echo "10. 🧪 运行集成测试(网络版)"
    echo "11. 🔧 网络诊断工具"
    echo ""
    echo -e "${PURPLE}🧹 清扫功能测试${NC}"
    echo "12. 🧹 演示清扫功能"
    echo "13. 📊 查看系统状态"
    echo ""
    echo -e "${PURPLE}🛠️  系统管理${NC}"
    echo "14. 🛑 停止所有进程"
    echo "15. ❌ 退出"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -n "请选择 [1-15]: "
}

# 检查硬件连接
check_hardware() {
    echo -e "${BLUE}🔍 检查硬件连接状态${NC}"
    echo "=============================="
    
    # 检查激光雷达
    if [ -e /dev/ttyUSB1 ]; then
        echo -e "${GREEN}✅ N10P激光雷达(串口): /dev/ttyUSB1${NC}"
        sudo chmod 666 /dev/ttyUSB1
    else
        echo -e "${RED}❌ N10P激光雷达(串口): 未发现 /dev/ttyUSB1${NC}"
    fi
    
    # 检查电机控制器
    if [ -e /dev/ttyUSB0 ]; then
        echo -e "${GREEN}✅ 电机控制器: /dev/ttyUSB0${NC}"
        sudo chmod 666 /dev/ttyUSB0
    else
        echo -e "${RED}❌ 电机控制器: 未发现 /dev/ttyUSB0${NC}"
    fi
    
    # 检查摄像头
    if ls /dev/video* 1> /dev/null 2>&1; then
        echo -e "${GREEN}✅ 摄像头设备:${NC}"
        ls -la /dev/video*
    else
        echo -e "${RED}❌ 摄像头设备: 未发现${NC}"
    fi
    
    echo ""
    echo -e "${YELLOW}💡 提示: 如果设备未发现，请检查硬件连接${NC}"
}

# 检查网络连接
check_network() {
    echo -e "${BLUE}🌐 检查网络连接状态${NC}"
    echo "=============================="
    
    LIDAR_IP="192.168.1.200"
    HOST_IP="192.168.1.102"
    
    echo -e "${YELLOW}📍 网络配置:${NC}"
    echo "   激光雷达IP: $LIDAR_IP"
    echo "   本机IP: $HOST_IP"
    echo "   数据端口: 2368 (UDP)"
    echo "   控制端口: 2369 (UDP)"
    echo ""
    
    # 检查本机IP
    if ip addr show | grep -q "$HOST_IP"; then
        echo -e "${GREEN}✅ 本机IP地址已配置: $HOST_IP${NC}"
    else
        echo -e "${RED}❌ 本机IP地址未配置: $HOST_IP${NC}"
        echo -e "${YELLOW}💡 可用IP地址:${NC}"
        ip addr show | grep 'inet ' | grep -v '127.0.0.1' | sed 's/^/   /'
    fi
    
    # Ping测试
    echo ""
    echo "📡 测试激光雷达连接..."
    if timeout 3 ping -c 1 "$LIDAR_IP" > /dev/null 2>&1; then
        echo -e "${GREEN}✅ 成功连接到N10P激光雷达(网络版): $LIDAR_IP${NC}"
    else
        echo -e "${RED}❌ 无法连接到N10P激光雷达(网络版): $LIDAR_IP${NC}"
        echo -e "${YELLOW}💡 请检查:${NC}"
        echo "   1. 激光雷达是否开机"
        echo "   2. 网线连接是否正常"
        echo "   3. IP地址设置是否正确"
    fi
}

# 启动N10P激光雷达(串口版)
start_lidar_serial() {
    echo -e "${BLUE}📡 启动N10P激光雷达(串口版)${NC}"
    echo "=============================="
    
    gnome-terminal --title="N10P LiDAR (Serial)" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在启动N10P激光雷达驱动(串口版)...'
        ros2 launch lslidar_driver cleaning_robot_n10p.launch.py
        exec bash
    " &
    
    sleep 3
    echo -e "${GREEN}✅ N10P激光雷达(串口版)已在新终端启动${NC}"
    echo -e "${YELLOW}💡 检查激光雷达数据: ros2 topic echo /cleaning_robot/scan${NC}"
}

# 启动N10P激光雷达(网络版)
start_lidar_network() {
    echo -e "${BLUE}🌐 启动N10P激光雷达(网络版)${NC}"
    echo "=============================="
    
    gnome-terminal --title="N10P LiDAR (Network)" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在启动N10P激光雷达驱动(网络版)...'
        ros2 launch lslidar_driver cleaning_robot_n10p_net.launch.py
        exec bash
    " &
    
    sleep 3
    echo -e "${GREEN}✅ N10P激光雷达(网络版)已在新终端启动${NC}"
    echo -e "${YELLOW}💡 检查激光雷达数据: ros2 topic echo /cleaning_robot/scan${NC}"
}

# 启动SLAM建图(串口版)
start_slam_serial() {
    echo -e "${BLUE}🗺️  启动SLAM建图(串口版)${NC}"
    echo "=============================="
    
    gnome-terminal --title="SLAM + RViz (Serial)" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在启动SLAM建图和RViz可视化(串口版)...'
        ros2 launch cleaning_robot_navigation cleaning_robot_n10p.launch.py
        exec bash
    " &
    
    sleep 5
    echo -e "${GREEN}✅ SLAM建图(串口版)已在新终端启动${NC}"
    echo -e "${YELLOW}💡 RViz2可视化界面应该已打开${NC}"
}

# 启动SLAM建图(网络版)
start_slam_network() {
    echo -e "${BLUE}🌐 启动SLAM建图(网络版)${NC}"
    echo "=============================="
    
    gnome-terminal --title="SLAM + RViz (Network)" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在启动SLAM建图和RViz可视化(网络版)...'
        ros2 launch cleaning_robot_navigation cleaning_robot_n10p_net.launch.py
        exec bash
    " &
    
    sleep 5
    echo -e "${GREEN}✅ SLAM建图(网络版)已在新终端启动${NC}"
    echo -e "${YELLOW}💡 RViz2可视化界面应该已打开${NC}"
}

# 启动完整系统(串口版)
start_complete_serial() {
    echo -e "${BLUE}🎯 启动完整清扫机器人系统(串口版)${NC}"
    echo "=============================="
    
    gnome-terminal --title="Complete System (Serial)" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在启动完整清扫机器人系统(串口版)...'
        ./start_cleaning_robot_n10p.sh
        exec bash
    " &
    
    sleep 5
    echo -e "${GREEN}✅ 完整系统(串口版)已在新终端启动${NC}"
    echo -e "${YELLOW}💡 所有组件都在运行中${NC}"
}

# 启动完整系统(网络版)
start_complete_network() {
    echo -e "${BLUE}🌐 启动完整清扫机器人系统(网络版)${NC}"
    echo "=============================="
    
    gnome-terminal --title="Complete System (Network)" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在启动完整清扫机器人系统(网络版)...'
        ./start_cleaning_robot_n10p_net.sh
        exec bash
    " &
    
    sleep 5
    echo -e "${GREEN}✅ 完整系统(网络版)已在新终端启动${NC}"
    echo -e "${YELLOW}💡 所有组件都在运行中${NC}"
}

# 运行集成测试(串口版)
run_test_serial() {
    echo -e "${BLUE}🧪 运行N10P集成测试(串口版)${NC}"
    echo "=============================="
    
    gnome-terminal --title="N10P Integration Test (Serial)" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在运行N10P集成测试(串口版)...'
        python3 test_n10p_integration.py
        exec bash
    " &
    
    sleep 2
    echo -e "${GREEN}✅ 集成测试(串口版)已在新终端启动${NC}"
    echo -e "${YELLOW}💡 观察测试结果和数据统计${NC}"
}

# 运行集成测试(网络版)
run_test_network() {
    echo -e "${BLUE}🌐 运行N10P集成测试(网络版)${NC}"
    echo "=============================="
    
    gnome-terminal --title="N10P Integration Test (Network)" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在运行N10P集成测试(网络版)...'
        python3 test_n10p_network.py
        exec bash
    " &
    
    sleep 2
    echo -e "${GREEN}✅ 集成测试(网络版)已在新终端启动${NC}"
    echo -e "${YELLOW}💡 观察网络测试结果和性能统计${NC}"
}

# 网络诊断工具
run_network_diagnosis() {
    echo -e "${BLUE}🔧 N10P网络诊断工具${NC}"
    echo "=============================="
    
    gnome-terminal --title="N10P Network Diagnosis" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在运行N10P网络诊断工具...'
        ./diagnose_n10p_network.sh
        exec bash
    " &
    
    sleep 2
    echo -e "${GREEN}✅ 网络诊断工具已在新终端启动${NC}"
    echo -e "${YELLOW}💡 查看详细的网络连接和配置信息${NC}"
}

# 演示清扫功能
demo_cleaning() {
    echo -e "${BLUE}🧹 演示清扫功能${NC}"
    echo "=============================="
    
    gnome-terminal --title="Cleaning Demo" -- bash -c "
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '正在演示清扫功能...'
        echo '请确保完整系统已启动!'
        sleep 3
        python3 test_cleaning.py
        exec bash
    " &
    
    sleep 2
    echo -e "${GREEN}✅ 清扫演示已在新终端启动${NC}"
    echo -e "${YELLOW}💡 在RViz中观察清扫路径${NC}"
}

# 查看系统状态
check_status() {
    echo -e "${BLUE}📊 系统状态检查${NC}"
    echo "=============================="
    
    echo "🔍 运行中的ROS2节点:"
    ros2 node list 2>/dev/null || echo "未发现运行中的节点"
    
    echo ""
    echo "📡 激光雷达话题:"
    ros2 topic list | grep scan 2>/dev/null || echo "未发现激光雷达话题"
    
    echo ""
    echo "🗺️  地图话题:"
    ros2 topic list | grep map 2>/dev/null || echo "未发现地图话题"
    
    echo ""
    echo "🎯 清扫相关话题:"
    ros2 topic list | grep cleaning 2>/dev/null || echo "未发现清扫话题"
    
    echo ""
    echo "🌐 网络状态:"
    if ros2 topic list | grep -q "/cleaning_robot/scan"; then
        echo "激光雷达数据: ✅ 正在接收"
        # 检查数据频率
        timeout 3 ros2 topic hz /cleaning_robot/scan 2>/dev/null | head -n 3
    else
        echo "激光雷达数据: ❌ 未接收"
    fi
}

# 停止所有进程
stop_all() {
    echo -e "${RED}🛑 停止所有ROS2进程${NC}"
    echo "=============================="
    
    # 杀死所有ros2进程
    pkill -f ros2
    pkill -f rviz2
    pkill -f gazebo
    
    sleep 2
    echo -e "${GREEN}✅ 所有进程已停止${NC}"
}

# 主循环
while true; do
    show_menu
    read choice
    
    case $choice in
        1)
            check_hardware
            ;;
        2)
            start_lidar_serial
            ;;
        3)
            start_slam_serial
            ;;
        4)
            start_complete_serial
            ;;
        5)
            run_test_serial
            ;;
        6)
            check_network
            ;;
        7)
            start_lidar_network
            ;;
        8)
            start_slam_network
            ;;
        9)
            start_complete_network
            ;;
        10)
            run_test_network
            ;;
        11)
            run_network_diagnosis
            ;;
        12)
            demo_cleaning
            ;;
        13)
            check_status
            ;;
        14)
            stop_all
            ;;
        15)
            echo -e "${GREEN}👋 感谢使用N10P完整演示系统！${NC}"
            echo -e "${BLUE}🎉 现在支持串口版和网络版N10P激光雷达${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}无效选择，请重新输入${NC}"
            ;;
    esac
    
    echo ""
    echo -e "${YELLOW}按任意键继续...${NC}"
    read -n 1
done 