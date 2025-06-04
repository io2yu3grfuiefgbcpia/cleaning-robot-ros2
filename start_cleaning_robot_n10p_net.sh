#!/bin/bash

# 清扫机器人N10P网络版启动脚本
# 适用于以太网连接的N10P激光雷达

echo "🌐 清扫机器人N10P网络版系统启动脚本"
echo "========================================"

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

# 网络配置参数
LIDAR_IP=${LIDAR_IP:-"192.168.1.200"}      # N10P激光雷达IP地址
HOST_IP=${HOST_IP:-"192.168.1.102"}        # 本机IP地址
MSOP_PORT=${MSOP_PORT:-"2368"}             # 数据端口
DIFOP_PORT=${DIFOP_PORT:-"2369"}           # 控制端口

echo "🌐 网络配置检查..."
echo "=================================="
echo "📍 激光雷达IP: $LIDAR_IP"
echo "📍 本机IP: $HOST_IP"
echo "📍 数据端口: $MSOP_PORT"
echo "📍 控制端口: $DIFOP_PORT"

# 检查网络连接
echo ""
echo "🔍 检查网络连接..."

# 检查本机IP是否可用
if ip addr show | grep -q "$HOST_IP"; then
    echo "✅ 本机IP地址 $HOST_IP 可用"
else
    echo "⚠️  本机IP地址 $HOST_IP 未配置"
    echo "   请检查网络配置或修改 HOST_IP 参数"
    # 显示可用的IP地址
    echo "   可用的IP地址:"
    ip addr show | grep 'inet ' | grep -v '127.0.0.1' | awk '{print "   " $2}'
fi

# Ping测试激光雷达
echo "📡 测试与N10P激光雷达的连接..."
if timeout 3 ping -c 1 "$LIDAR_IP" > /dev/null 2>&1; then
    echo "✅ 成功连接到N10P激光雷达 ($LIDAR_IP)"
else
    echo "❌ 无法连接到N10P激光雷达 ($LIDAR_IP)"
    echo "   请检查:"
    echo "   1. 激光雷达是否开机"
    echo "   2. 网线连接是否正常"
    echo "   3. IP地址设置是否正确"
    echo "   4. 防火墙设置"
fi

# 检查端口是否被占用
echo ""
echo "🔌 检查端口状态..."
if netstat -tuln | grep -q ":$MSOP_PORT "; then
    echo "⚠️  端口 $MSOP_PORT 已被占用"
    echo "   可能有其他程序正在使用该端口"
else
    echo "✅ 端口 $MSOP_PORT 可用"
fi

if netstat -tuln | grep -q ":$DIFOP_PORT "; then
    echo "⚠️  端口 $DIFOP_PORT 已被占用"
else
    echo "✅ 端口 $DIFOP_PORT 可用"
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

# 2. 启动N10P激光雷达（网络版）
echo "2. 启动N10P激光雷达（网络版）..."
start_node "n10p_lidar_net" "ros2 launch lslidar_driver cleaning_robot_n10p_net.launch.py device_ip:=$LIDAR_IP host_ip:=$HOST_IP"

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
echo "🌐 网络版N10P特有功能："
echo "1. 查看激光雷达网络状态:"
echo "   ros2 topic echo /cleaning_robot/scan --once"
echo ""
echo "2. 监控网络数据流量:"
echo "   iftop -i eth0"
echo ""
echo "3. 检查UDP数据包:"
echo "   tcpdump -i eth0 host $LIDAR_IP"
echo ""
echo "4. 启动网络版RViz可视化:"
echo "   ros2 launch cleaning_robot_navigation cleaning_robot_n10p_net.launch.py"
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
echo "4. 开始清扫任务:"
echo "   python3 test_cleaning.py"
echo ""
echo "5. 查看实时日志:"
echo "   tail -f logs/*.log"
echo ""
echo "6. 网络诊断:"
echo "   ./diagnose_n10p_network.sh"
echo ""
echo "🛑 停止系统: Ctrl+C"

# 等待用户中断
wait 