#!/bin/bash

# 清扫机器人N10P实物硬件启动脚本
# 适用于实际的清扫机器人硬件系统

echo "🤖 清扫机器人N10P系统启动脚本"
echo "=================================="

# 设置环境变量
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "正在设置ROS2环境..."
    source /opt/ros/humble/setup.bash
    source install/setup.bash
else
    echo "✅ ROS2环境已设置: $ROS_DISTRO"
fi

# 检查串口权限
echo "🔌 检查硬件连接..."

# 检查激光雷达串口
if [ -e /dev/ttyUSB1 ]; then
    echo "✅ 找到N10P激光雷达: /dev/ttyUSB1"
    sudo chmod 666 /dev/ttyUSB1
else
    echo "⚠️  未找到N10P激光雷达在 /dev/ttyUSB1"
    echo "   请检查设备连接或修改配置文件中的串口路径"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "   未发现任何USB串口设备"
fi

# 检查电机控制器串口
if [ -e /dev/ttyUSB0 ]; then
    echo "✅ 找到电机控制器: /dev/ttyUSB0"
    sudo chmod 666 /dev/ttyUSB0
else
    echo "⚠️  未找到电机控制器在 /dev/ttyUSB0"
    echo "   请检查设备连接"
fi

# 检查摄像头
if ls /dev/video* 1> /dev/null 2>&1; then
    echo "✅ 找到摄像头设备:"
    ls -la /dev/video*
else
    echo "⚠️  未找到摄像头设备"
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
    sleep 2
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

# 2. 启动N10P激光雷达
echo "2. 启动N10P激光雷达..."
start_node "n10p_lidar" "ros2 launch lslidar_driver cleaning_robot_n10p.launch.py"

# 3. 启动SLAM建图
echo "3. 启动SLAM建图..."
start_node "slam_toolbox" "ros2 launch slam_toolbox online_async_launch.py params_file:=src/cleaning_robot_slam/config/cleaning_robot_slam_params.yaml use_sim_time:=false"

# 4. 启动导航系统
echo "4. 启动导航系统..."
start_node "navigation2" "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false"

# 5. 启动清扫控制器
echo "5. 启动清扫控制器..."
start_node "cleaning_controller" "ros2 run cleaning_robot_control cleaning_controller_node"

# 6. 启动立体视觉处理
echo "6. 启动立体视觉..."
start_node "stereo_processor" "ros2 run cleaning_robot_perception stereo_processor_node"

echo ""
echo "⏳ 等待所有节点启动完成..."
sleep 10

echo ""
echo "🔍 检查节点状态..."
echo "=================================="

# 检查关键节点
check_node "robot_state_publisher"
check_node "lslidar_driver_node"
check_node "slam_toolbox"
check_node "controller_server"
check_node "cleaning_controller"

echo ""
echo "📊 当前运行的节点:"
ros2 node list

echo ""
echo "📡 当前话题列表:"
ros2 topic list

echo ""
echo "🎯 系统启动完成！"
echo "=================================="
echo ""
echo "🔧 常用控制命令："
echo "1. 查看激光雷达数据:"
echo "   ros2 topic echo /cleaning_robot/scan"
echo ""
echo "2. 查看地图构建:"
echo "   ros2 topic echo /map"
echo ""
echo "3. 手动控制机器人:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cleaning_robot/cmd_vel"
echo ""
echo "4. 启动RViz可视化:"
echo "   ros2 launch cleaning_robot_navigation cleaning_robot_n10p.launch.py"
echo ""
echo "5. 开始清扫任务:"
echo "   python3 test_cleaning.py"
echo ""
echo "6. 查看实时日志:"
echo "   tail -f logs/*.log"
echo ""
echo "🛑 停止系统: Ctrl+C"

# 等待用户中断
wait 