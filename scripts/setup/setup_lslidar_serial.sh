#!/bin/bash

# 镭神激光雷达N10P串口配置脚本
echo "正在配置镭神激光雷达N10P串口权限..."

# 检查串口设备
echo "检查可用的串口设备："
ls -la /dev/ttyUSB* 2>/dev/null || echo "未发现 /dev/ttyUSB* 设备"
ls -la /dev/ttyACM* 2>/dev/null || echo "未发现 /dev/ttyACM* 设备"

# 添加用户到dialout组
echo "添加当前用户到dialout组..."
sudo usermod -a -G dialout $USER

# 设置串口权限
if [ -e /dev/ttyUSB0 ]; then
    echo "设置 /dev/ttyUSB0 权限..."
    sudo chmod 666 /dev/ttyUSB0
fi

if [ -e /dev/ttyUSB1 ]; then
    echo "设置 /dev/ttyUSB1 权限..."
    sudo chmod 666 /dev/ttyUSB1
fi

# 创建udev规则（如果激光雷达设备固定）
echo "创建udev规则..."
sudo tee /etc/udev/rules.d/99-lslidar.rules > /dev/null << 'EOF'
# 镭神激光雷达N10P设备规则
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lslidar_n10p"
SUBSYSTEM=="tty", ATTRS{product}=="CP2102 USB to UART Bridge Controller", SYMLINK+="lslidar_n10p"
EOF

# 重新加载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "配置完成！"
echo ""
echo "使用说明："
echo "1. 请重新登录或重启终端以使组权限生效"
echo "2. 连接激光雷达后检查设备："
echo "   ls -la /dev/ttyUSB* 或 ls -la /dev/lslidar_n10p"
echo "3. 如果设备路径不是 /dev/ttyUSB1，请修改配置文件："
echo "   src/lslidar_driver/params/lidar_uart_ros2/cleaning_robot_n10p.yaml"
echo "   中的 serial_port_ 参数"
echo ""
echo "测试激光雷达："
echo "ros2 launch lslidar_driver cleaning_robot_n10p.launch.py" 