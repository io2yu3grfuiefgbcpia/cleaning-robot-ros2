#!/bin/bash

# 清扫机器人Orbbec深度相机版启动脚本
# 适用于配备奥比中光深度相机的清扫机器人系统

echo "🎥 清扫机器人Orbbec深度相机版系统启动脚本"
echo "=============================================="

# 设置环境变量
export ROS_DOMAIN_ID=0

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "正在设置ROS2环境..."
    source /opt/ros/humble/setup.bash
    source install/setup.bash
else
    echo "✅ ROS2环境已设置: $ROS_DISTRO"
fi

# 检查是否安装了Orbbec相机依赖
echo "🔍 检查Orbbec相机依赖..."

# 检查Orbbec包是否存在
if ros2 pkg list | grep -q "orbbec_camera"; then
    echo "✅ Orbbec相机包已安装"
else
    echo "⚠️  未找到Orbbec相机包"
    echo "   请按照以下步骤安装:"
    echo "   1. 运行: bash install_orbbec_dependencies.sh"
    echo "   2. 克隆Orbbec ROS2包到工作空间"
    echo "   3. 编译工作空间"
fi

# 检查USB设备权限
echo "🔌 检查硬件连接..."

# 检查Orbbec设备
if lsusb | grep -q "2bc5"; then
    echo "✅ 找到Orbbec深度相机设备"
else
    echo "⚠️  未找到Orbbec深度相机设备"
    echo "   请检查:"
    echo "   1. 相机是否连接"
    echo "   2. USB权限设置"
    echo "   3. udev规则是否正确安装"
fi

# 检查激光雷达
if [ -e /dev/ttyUSB1 ]; then
    echo "✅ 找到激光雷达: /dev/ttyUSB1"
    sudo chmod 666 /dev/ttyUSB1
else
    echo "⚠️  未找到激光雷达在 /dev/ttyUSB1"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "   未发现任何USB串口设备"
fi

# 创建日志目录
mkdir -p logs

# 函数：启动节点并记录日志
start_node() {
    local name=$1
    local command=$2
    echo "启动 $name..."
    eval "$command" > logs/${name}.log 2>&1 &
    local pid=$!
    echo "$name PID: $pid"
    sleep 3
}

# 函数：检查节点状态
check_node() {
    local node_name=$1
    if ros2 node list | grep -q "$node_name"; then
        echo "✅ $node_name 运行正常"
        return 0
    else
        echo "❌ $node_name 启动失败"
        return 1
    fi
}

echo ""
echo "🚀 开始启动系统组件..."
echo "=================================="

# 1. 启动机器人状态发布器
echo "1. 启动机器人模型..."
start_node "robot_state_publisher" "ros2 launch cleaning_robot_description robot_state_publisher.launch.py"

# 2. 启动Orbbec深度相机
echo "2. 启动Orbbec深度相机..."
start_node "orbbec_camera" "ros2 run cleaning_robot_perception orbbec_camera_node"

# 3. 启动深度相机处理器
echo "3. 启动深度相机处理器..."
start_node "depth_processor" "ros2 run cleaning_robot_perception depth_camera_processor_node"

# 4. 启动激光雷达
echo "4. 启动激光雷达..."
start_node "lidar" "ros2 launch lslidar_driver cleaning_robot_n10p.launch.py"

# 5. 启动SLAM建图
echo "5. 启动SLAM建图..."
start_node "slam_toolbox" "ros2 launch slam_toolbox online_async_launch.py params_file:=src/cleaning_robot_slam/config/cleaning_robot_slam_params.yaml use_sim_time:=false"

# 6. 启动导航系统
echo "6. 启动导航系统..."
start_node "navigation2" "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false"

# 7. 启动清扫控制器
echo "7. 启动清扫控制器..."
start_node "cleaning_controller" "ros2 run cleaning_robot_control cleaning_controller_node"

# 8. 启动立体视觉处理（如果需要）
echo "8. 启动立体视觉..."
start_node "stereo_processor" "ros2 run cleaning_robot_perception stereo_processor_node"

echo ""
echo "⏳ 等待所有节点启动完成..."
sleep 15

echo ""
echo "🔍 检查节点状态..."
echo "=================================="

# 检查关键节点
check_node "robot_state_publisher"
check_node "orbbec_camera_node"
check_node "depth_camera_processor"
check_node "lslidar_driver_node"
check_node "slam_toolbox"
check_node "controller_server"
check_node "cleaning_controller"

echo ""
echo "📊 当前运行的节点:"
ros2 node list

echo ""
echo "📡 当前话题列表:"
ros2 topic list | grep -E "(orbbec|depth|cleaning)"

echo ""
echo "🎯 系统启动完成！"
echo "=================================="
echo ""
echo "🎥 Orbbec深度相机特有功能："
echo "1. 查看彩色图像:"
echo "   ros2 run rqt_image_view rqt_image_view /cleaning_robot/orbbec_camera/color/image_raw"
echo ""
echo "2. 查看深度图像:"
echo "   ros2 run rqt_image_view rqt_image_view /cleaning_robot/orbbec_camera/depth/image_raw"
echo ""
echo "3. 查看点云数据:"
echo "   ros2 topic echo /cleaning_robot/orbbec_camera/pointcloud --once"
echo ""
echo "4. 查看障碍物检测:"
echo "   ros2 run rqt_image_view rqt_image_view /cleaning_robot/depth_camera/obstacles"
echo ""
echo "5. 启动完整可视化:"
echo "   ros2 launch cleaning_robot_description cleaning_robot_with_orbbec.launch.py"
echo ""
echo "🔧 常用控制命令："
echo "1. 手动控制机器人:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cleaning_robot/cmd_vel"
echo ""
echo "2. 开始清扫任务:"
echo "   python3 test_cleaning.py"
echo ""
echo "3. 查看实时日志:"
echo "   tail -f logs/*.log"
echo ""
echo "🛑 停止系统: Ctrl+C"

# 等待用户中断
wait 