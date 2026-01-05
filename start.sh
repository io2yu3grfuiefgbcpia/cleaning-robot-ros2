#!/bin/bash

# 清扫机器人系统 - 主启动脚本
# 提供便捷的系统启动入口

echo "🤖 清扫机器人系统 - 主启动菜单"
echo "=================================="
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$SCRIPT_DIR/scripts"

# 设置ROS2环境
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

echo "请选择启动模式:"
echo ""
echo "━━━━━━━ 🚀 完整系统 ━━━━━━━"
echo "1. 完整系统 + 可视化界面（建图+摄像头+路径规划）【推荐】"
echo "2. 完整系统 + 控制面板（Python GUI）"
echo ""
echo "━━━━━━━ 📡 激光雷达版 ━━━━━━━"
echo "3. N10P激光雷达 - 网络版"
echo "4. N10P激光雷达 - 串口版"
echo ""
echo "━━━━━━━ 📷 深度相机版 ━━━━━━━"
echo "5. Orbbec深度相机版"
echo ""
echo "━━━━━━━ 🛠️ 工具 ━━━━━━━"
echo "6. 激光雷达可视化工具"
echo "7. 完整功能演示菜单"
echo "8. 网络诊断工具"
echo "9. 编译项目"
echo ""
echo "0. 退出"
echo ""
read -p "请选择 [0-9，默认1]: " choice
choice=${choice:-1}

case $choice in
    1)
        echo "🚀 启动完整系统 + RViz可视化..."
        bash "$SCRIPTS_DIR/launch/start_full_system.sh"
        ;;
    2)
        echo "🚀 启动完整系统 + 控制面板..."
        cd "$SCRIPT_DIR"
        source install/setup.bash
        ros2 launch cleaning_robot_description cleaning_robot_full_system.launch.py \
            use_rviz:=false use_dashboard:=true
        ;;
    3)
        echo "📡 启动N10P网络版系统..."
        bash "$SCRIPTS_DIR/launch/start_cleaning_robot_n10p_net.sh"
        ;;
    4)
        echo "📡 启动N10P串口版系统..."
        bash "$SCRIPTS_DIR/launch/start_cleaning_robot_n10p.sh"
        ;;
    5)
        echo "📷 启动Orbbec深度相机版系统..."
        bash "$SCRIPTS_DIR/launch/start_cleaning_robot_orbbec.sh"
        ;;
    6)
        echo "📊 启动激光雷达可视化..."
        bash "$SCRIPTS_DIR/utils/start_lidar_viz.sh"
        ;;
    7)
        echo "🎮 启动完整功能演示..."
        bash "$SCRIPTS_DIR/utils/demo_n10p_complete.sh"
        ;;
    8)
        echo "🔧 启动网络诊断工具..."
        bash "$SCRIPTS_DIR/diagnostic/diagnose_n10p_network.sh"
        ;;
    9)
        echo "🔨 编译项目..."
        cd "$SCRIPT_DIR"
        colcon build --symlink-install
        echo ""
        echo "✅ 编译完成！请运行: source install/setup.bash"
        ;;
    0)
        echo "👋 退出"
        exit 0
        ;;
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac

