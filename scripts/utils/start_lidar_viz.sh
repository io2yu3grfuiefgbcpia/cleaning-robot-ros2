#!/bin/bash

echo "🤖 清扫机器人激光雷达可视化系统"
echo "=================================="
echo ""

# 检查Python环境
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 未安装"
    echo "请安装: sudo apt install python3"
    exit 1
fi

# 检查matplotlib
echo "🔍 检查依赖..."
python3 -c "import matplotlib.pyplot as plt" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "📦 安装matplotlib..."
    sudo apt update
    sudo apt install -y python3-matplotlib python3-tk python3-numpy
fi

echo "✅ 环境检查完成"
echo ""

echo "🎯 选择激光雷达可视化模式:"
echo "1. 完整RViz风格 (4个窗口) - 包含极坐标扫描、SLAM地图、机器人视图、数据统计"
echo "2. 简化双窗口版本 - 极坐标扫描 + 机器人局部视图"
echo "3. 查看系统状态"
echo "4. 连接真实激光雷达数据"
echo "5. 退出"
echo ""

read -p "请选择 [1-5]: " choice

case $choice in
    1)
        echo ""
        echo "🚀 启动完整激光雷达RViz..."
        echo "功能包含:"
        echo "  • 左上: 360°激光雷达扫描"
        echo "  • 右上: SLAM地图构建"
        echo "  • 左下: 机器人5米范围视图"
        echo "  • 右下: 实时数据统计"
        echo ""
        echo "🎮 控制说明:"
        echo "  • 关闭窗口停止程序"
        echo "  • 按Ctrl+C强制退出"
        echo ""
        sleep 2
        python3 lidar_rviz.py
        ;;
    2)
        echo ""
        echo "🚀 启动简化激光雷达查看器..."
        echo "功能包含:"
        echo "  • 左侧: 360°极坐标扫描"
        echo "  • 右侧: 8米范围机器人视图"
        echo ""
        echo "🎨 颜色说明:"
        echo "  • 红色: 激光扫描/障碍物"
        echo "  • 蓝色: 机器人位置"
        echo "  • 橙色: 危险区域(<1米)"
        echo "  • 黄色: 近距离物体(<2米)"
        echo ""
        sleep 2
        python3 simple_lidar_viewer.py
        ;;
    3)
        echo ""
        echo "📊 清扫机器人系统状态"
        echo "======================"
        
        # 检查ROS2环境
        source /opt/ros/humble/setup.bash 2>/dev/null
        source install/setup.bash 2>/dev/null
        
        echo "🤖 ROS2节点状态:"
        if command -v ros2 &> /dev/null; then
            ros2 node list 2>/dev/null | grep -E "(robot|lidar|slam|controller)" | sed 's/^/   /'
            if [ $? -ne 0 ]; then
                echo "   ❌ 未发现运行中的ROS节点"
            fi
        else
            echo "   ❌ ROS2命令不可用"
        fi
        
        echo ""
        echo "📡 激光雷达话题:"
        if command -v ros2 &> /dev/null; then
            ros2 topic list 2>/dev/null | grep -E "(scan|laser|pointcloud)" | sed 's/^/   /'
            if [ $? -ne 0 ]; then
                echo "   ❌ 未发现激光雷达话题"
            fi
        else
            echo "   ❌ 无法检查话题"
        fi
        
        echo ""
        echo "🌐 网络状态:"
        ping -c 1 192.168.1.200 &>/dev/null
        if [ $? -eq 0 ]; then
            echo "   ✅ 激光雷达(192.168.1.200)网络连通"
        else
            echo "   ❌ 激光雷达网络不通"
        fi
        
        echo ""
        echo "📁 项目文件:"
        ls -la *.py 2>/dev/null | sed 's/^/   /'
        ;;
    4)
        echo ""
        echo "🔗 连接真实激光雷达数据"
        echo "======================="
        
        # 加载ROS2环境
        source /opt/ros/humble/setup.bash 2>/dev/null
        source install/setup.bash 2>/dev/null
        
        echo "🔍 检查激光雷达话题..."
        if command -v ros2 &> /dev/null; then
            topics=$(ros2 topic list 2>/dev/null | grep -E "(scan|laser)")
            if [ -n "$topics" ]; then
                echo "✅ 发现激光雷达话题:"
                echo "$topics" | sed 's/^/   /'
                echo ""
                echo "📊 话题数据预览:"
                timeout 3 ros2 topic echo /cleaning_robot/scan --once 2>/dev/null | head -20 | sed 's/^/   /'
            else
                echo "❌ 未发现激光雷达话题"
                echo "请确保激光雷达驱动已启动"
            fi
        else
            echo "❌ ROS2环境未正确设置"
        fi
        
        echo ""
        echo "💡 启动激光雷达驱动:"
        echo "   ros2 launch lslidar_driver cleaning_robot_n10p_net.launch.py"
        ;;
    5)
        echo "👋 再见！"
        exit 0
        ;;
    *)
        echo "❌ 无效选择，请重新运行脚本"
        exit 1
        ;;
esac

echo ""
echo "✅ 程序已结束" 