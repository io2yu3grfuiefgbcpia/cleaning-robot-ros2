#!/bin/bash

# 清扫机器人节点启动问题修复脚本
# 解决系统启动时多个节点失败的问题

echo "🔧 清扫机器人节点启动问题修复工具"
echo "================================="

# 设置工作目录
WORKSPACE_DIR="/home/yys/cleaning_robot_ws"
cd "$WORKSPACE_DIR"

echo "💡 检测到的问题："
echo "1. robot_state_publisher 启动失败"
echo "2. orbbec_camera_node 启动失败"
echo "3. depth_camera_processor 启动失败"
echo "4. lslidar_driver_node 启动失败"
echo "5. cleaning_controller 启动失败"
echo ""

echo "🔍 问题分析和修复..."
echo "================================="

# 1. 检查是否需要构建项目
echo "📦 1. 检查项目构建状态..."
if [ ! -f "install/setup.bash" ]; then
    echo "❌ 未找到 install/setup.bash，需要重新构建项目"
    
    echo "🧹 清理旧构建文件..."
    rm -rf build/ install/ log/
    
    echo "🔨 开始构建ROS2包..."
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    
    if [ $? -eq 0 ]; then
        echo "✅ 项目构建成功"
    else
        echo "❌ 项目构建失败，请检查错误信息"
        exit 1
    fi
else
    echo "✅ 找到构建文件"
fi

# 2. 检查和修复节点可执行文件
echo ""
echo "🎯 2. 检查节点可执行文件..."

# 检查cleaning_controller可执行文件
if [ ! -f "install/cleaning_robot_control/lib/cleaning_robot_control/cleaning_controller" ]; then
    echo "❌ cleaning_controller 可执行文件缺失"
    
    # 检查setup.py配置
    if grep -q "cleaning_controller_node" src/cleaning_robot_control/setup.py; then
        echo "📝 修复setup.py中的入口点..."
        sed -i 's/cleaning_controller_node/cleaning_controller/g' src/cleaning_robot_control/setup.py
        
        echo "🔨 重新构建 cleaning_robot_control 包..."
        source install/setup.bash
        colcon build --packages-select cleaning_robot_control --symlink-install
    fi
else
    echo "✅ cleaning_controller 可执行文件存在"
fi

# 3. 检查依赖包
echo ""
echo "📚 3. 检查依赖包..."

MISSING_DEPS=()

# 检查关键依赖
if ! ros2 pkg list | grep -q "robot_state_publisher"; then
    MISSING_DEPS+=("ros-humble-robot-state-publisher")
fi

if ! ros2 pkg list | grep -q "joint_state_publisher"; then
    MISSING_DEPS+=("ros-humble-joint-state-publisher")
fi

if ! ros2 pkg list | grep -q "xacro"; then
    MISSING_DEPS+=("ros-humble-xacro")
fi

if ! ros2 pkg list | grep -q "slam_toolbox"; then
    MISSING_DEPS+=("ros-humble-slam-toolbox")
fi

if ! ros2 pkg list | grep -q "nav2_bringup"; then
    MISSING_DEPS+=("ros-humble-nav2-bringup")
fi

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo "❌ 缺少依赖包，正在安装..."
    sudo apt update
    sudo apt install -y "${MISSING_DEPS[@]}"
    echo "✅ 依赖包安装完成"
else
    echo "✅ 所有依赖包已安装"
fi

# 4. 修复启动脚本中的节点名称
echo ""
echo "📝 4. 修复启动脚本..."

# 备份原始脚本
cp start_cleaning_robot_orbbec.sh start_cleaning_robot_orbbec.sh.backup

# 修复节点启动命令
echo "修复节点可执行文件名称..."

# 修复清扫控制器启动命令
sed -i 's/cleaning_controller_node/cleaning_controller/g' start_cleaning_robot_orbbec.sh

# 修复其他节点名称错误（如果有）
sed -i 's/depth_camera_processor_node/depth_camera_processor/g' start_cleaning_robot_orbbec.sh
sed -i 's/stereo_processor_node/stereo_processor/g' start_cleaning_robot_orbbec.sh

echo "✅ 启动脚本已修复"

# 5. 创建简化的测试启动脚本
echo ""
echo "🧪 5. 创建测试启动脚本..."

cat > test_node_startup.sh << 'EOF'
#!/bin/bash

# 简化的节点启动测试脚本
echo "🧪 测试节点启动..."

# 设置环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "1. 测试 robot_state_publisher..."
timeout 10s ros2 launch cleaning_robot_description robot_state_publisher.launch.py &
sleep 5

echo "2. 检查节点状态..."
if ros2 node list | grep -q "robot_state_publisher"; then
    echo "✅ robot_state_publisher 启动成功"
else
    echo "❌ robot_state_publisher 启动失败"
fi

echo "3. 测试 cleaning_controller..."
timeout 10s ros2 run cleaning_robot_control cleaning_controller &
sleep 5

if ros2 node list | grep -q "cleaning_controller"; then
    echo "✅ cleaning_controller 启动成功"
else
    echo "❌ cleaning_controller 启动失败"
fi

echo "停止测试节点..."
pkill -f "ros2"
echo "测试完成"
EOF

chmod +x test_node_startup.sh

# 6. 创建URDF检查脚本
echo ""
echo "📄 6. 检查URDF文件..."

if [ -f "src/cleaning_robot_description/urdf/cleaning_robot.urdf.xacro" ]; then
    echo "✅ URDF文件存在"
    
    # 测试xacro文件是否有语法错误
    source /opt/ros/humble/setup.bash
    if xacro src/cleaning_robot_description/urdf/cleaning_robot.urdf.xacro > /tmp/test_robot.urdf 2>/dev/null; then
        echo "✅ URDF文件语法正确"
        rm -f /tmp/test_robot.urdf
    else
        echo "❌ URDF文件语法错误"
        echo "请检查 src/cleaning_robot_description/urdf/cleaning_robot.urdf.xacro"
    fi
else
    echo "❌ URDF文件缺失"
fi

# 7. 检查Python模块导入
echo ""
echo "🐍 7. 检查Python模块..."

python3 -c "
try:
    import rclpy
    print('✅ rclpy 模块正常')
except ImportError as e:
    print(f'❌ rclpy 导入错误: {e}')

try:
    import numpy
    print('✅ numpy 模块正常')
except ImportError as e:
    print(f'❌ numpy 导入错误: {e}')

try:
    import cv2
    print('✅ opencv 模块正常')
except ImportError as e:
    print(f'❌ opencv 导入错误: {e}')
"

echo ""
echo "✅ 节点启动问题修复完成！"
echo "================================="
echo ""
echo "📋 接下来的步骤："
echo "1. 运行测试脚本: ./test_node_startup.sh"
echo "2. 如果测试通过，尝试完整启动: ./start_cleaning_robot_orbbec.sh"
echo "3. 查看详细日志: tail -f logs/*.log"
echo ""
echo "🔧 手动测试单个节点："
echo "• 测试robot_state_publisher:"
echo "  ros2 launch cleaning_robot_description robot_state_publisher.launch.py"
echo ""
echo "• 测试cleaning_controller:"
echo "  ros2 run cleaning_robot_control cleaning_controller"
echo ""
echo "• 检查可用节点:"
echo "  ros2 pkg executables cleaning_robot_control"
echo "  ros2 pkg executables cleaning_robot_description" 