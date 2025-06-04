#!/bin/bash

# N10P激光雷达网络版诊断脚本
# 用于排查网络连接问题

echo "🌐 N10P激光雷达网络诊断工具"
echo "=================================="

# 默认配置
LIDAR_IP=${LIDAR_IP:-"192.168.1.200"}
HOST_IP=${HOST_IP:-"192.168.1.102"}
MSOP_PORT=${MSOP_PORT:-"2368"}
DIFOP_PORT=${DIFOP_PORT:-"2369"}

echo "📍 诊断配置:"
echo "   激光雷达IP: $LIDAR_IP"
echo "   本机IP: $HOST_IP"
echo "   数据端口: $MSOP_PORT"
echo "   控制端口: $DIFOP_PORT"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 诊断函数
diagnose_network_interface() {
    echo -e "${BLUE}🔍 1. 网络接口诊断${NC}"
    echo "=================================="
    
    # 检查网络接口
    echo "📡 可用网络接口:"
    ip addr show | grep -E '^[0-9]+:' | sed 's/^/   /'
    
    echo ""
    echo "🔌 网络接口状态:"
    ip link show | grep -E 'state (UP|DOWN)' | sed 's/^/   /'
    
    echo ""
    echo "📍 IP地址配置:"
    ip addr show | grep 'inet ' | grep -v '127.0.0.1' | sed 's/^/   /'
    
    # 检查本机IP
    if ip addr show | grep -q "$HOST_IP"; then
        echo -e "   ${GREEN}✅ 本机IP $HOST_IP 已配置${NC}"
    else
        echo -e "   ${RED}❌ 本机IP $HOST_IP 未配置${NC}"
        echo -e "   ${YELLOW}💡 建议配置IP地址:${NC}"
        echo "      sudo ip addr add $HOST_IP/24 dev eth0"
    fi
}

diagnose_network_connectivity() {
    echo -e "${BLUE}🔍 2. 网络连通性诊断${NC}"
    echo "=================================="
    
    # Ping测试
    echo "📡 Ping测试激光雷达..."
    if timeout 5 ping -c 3 "$LIDAR_IP" > /dev/null 2>&1; then
        echo -e "   ${GREEN}✅ 成功ping通 $LIDAR_IP${NC}"
        
        # 显示延迟信息
        echo "📊 网络延迟测试:"
        ping -c 3 "$LIDAR_IP" | grep 'time=' | sed 's/^/   /'
    else
        echo -e "   ${RED}❌ 无法ping通 $LIDAR_IP${NC}"
        echo -e "   ${YELLOW}💡 可能的原因:${NC}"
        echo "      1. 激光雷达未开机"
        echo "      2. 网线连接问题"
        echo "      3. IP地址配置错误"
        echo "      4. 不在同一网段"
    fi
    
    # 路由检查
    echo ""
    echo "🛣️  路由表检查:"
    ip route | grep "$LIDAR_IP" | sed 's/^/   /' || echo "   未找到相关路由"
}

diagnose_port_status() {
    echo -e "${BLUE}🔍 3. 端口状态诊断${NC}"
    echo "=================================="
    
    # 检查端口占用
    echo "🔌 检查端口占用状态:"
    
    if netstat -tuln | grep -q ":$MSOP_PORT "; then
        echo -e "   ${YELLOW}⚠️  数据端口 $MSOP_PORT 已被占用${NC}"
        netstat -tuln | grep ":$MSOP_PORT " | sed 's/^/      /'
    else
        echo -e "   ${GREEN}✅ 数据端口 $MSOP_PORT 可用${NC}"
    fi
    
    if netstat -tuln | grep -q ":$DIFOP_PORT "; then
        echo -e "   ${YELLOW}⚠️  控制端口 $DIFOP_PORT 已被占用${NC}"
        netstat -tuln | grep ":$DIFOP_PORT " | sed 's/^/      /'
    else
        echo -e "   ${GREEN}✅ 控制端口 $DIFOP_PORT 可用${NC}"
    fi
    
    # 测试UDP端口连接
    echo ""
    echo "📡 UDP端口测试:"
    
    # 测试数据端口
    timeout 3 nc -u -z "$LIDAR_IP" "$MSOP_PORT" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "   ${GREEN}✅ UDP端口 $MSOP_PORT 连接成功${NC}"
    else
        echo -e "   ${RED}❌ UDP端口 $MSOP_PORT 连接失败${NC}"
    fi
    
    # 测试控制端口
    timeout 3 nc -u -z "$LIDAR_IP" "$DIFOP_PORT" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "   ${GREEN}✅ UDP端口 $DIFOP_PORT 连接成功${NC}"
    else
        echo -e "   ${RED}❌ UDP端口 $DIFOP_PORT 连接失败${NC}"
    fi
}

diagnose_firewall() {
    echo -e "${BLUE}🔍 4. 防火墙诊断${NC}"
    echo "=================================="
    
    # 检查ufw状态
    if command -v ufw >/dev/null 2>&1; then
        echo "🔥 UFW防火墙状态:"
        ufw_status=$(sudo ufw status 2>/dev/null || echo "inactive")
        echo "   $ufw_status"
        
        if echo "$ufw_status" | grep -q "Status: active"; then
            echo -e "   ${YELLOW}⚠️  防火墙已启用，可能阻止UDP通信${NC}"
            echo -e "   ${YELLOW}💡 建议临时关闭防火墙测试:${NC}"
            echo "      sudo ufw disable"
            echo ""
            echo -e "   ${YELLOW}💡 或添加端口规则:${NC}"
            echo "      sudo ufw allow $MSOP_PORT/udp"
            echo "      sudo ufw allow $DIFOP_PORT/udp"
        fi
    fi
    
    # 检查iptables
    echo ""
    echo "🔥 iptables规则检查:"
    if iptables -L INPUT | grep -q "DROP\|REJECT"; then
        echo -e "   ${YELLOW}⚠️  发现阻止规则，可能影响UDP通信${NC}"
        iptables -L INPUT | grep -E "DROP|REJECT" | sed 's/^/      /'
    else
        echo -e "   ${GREEN}✅ 未发现阻止规则${NC}"
    fi
}

diagnose_traffic() {
    echo -e "${BLUE}🔍 5. 网络流量诊断${NC}"
    echo "=================================="
    
    echo "📊 实时网络流量监控:"
    
    # 检查tcpdump是否可用
    if command -v tcpdump >/dev/null 2>&1; then
        echo -e "   ${GREEN}✅ tcpdump 可用${NC}"
        echo -e "   ${YELLOW}💡 监控激光雷达数据包:${NC}"
        echo "      sudo tcpdump -i any host $LIDAR_IP"
        echo "      sudo tcpdump -i any port $MSOP_PORT"
    else
        echo -e "   ${YELLOW}⚠️  tcpdump 未安装${NC}"
        echo "      sudo apt install tcpdump"
    fi
    
    # 检查iftop是否可用
    if command -v iftop >/dev/null 2>&1; then
        echo -e "   ${GREEN}✅ iftop 可用${NC}"
        echo -e "   ${YELLOW}💡 监控网络流量:${NC}"
        echo "      sudo iftop -i eth0"
    else
        echo -e "   ${YELLOW}⚠️  iftop 未安装${NC}"
        echo "      sudo apt install iftop"
    fi
    
    # 简单的网络统计
    echo ""
    echo "📈 网络接口统计:"
    cat /proc/net/dev | grep -E 'eth0|enp' | sed 's/^/   /'
}

diagnose_ros_nodes() {
    echo -e "${BLUE}🔍 6. ROS节点诊断${NC}"
    echo "=================================="
    
    # 检查ROS2环境
    if [ -n "$ROS_DISTRO" ]; then
        echo -e "   ${GREEN}✅ ROS2环境: $ROS_DISTRO${NC}"
        
        # 检查激光雷达驱动节点
        if ros2 node list 2>/dev/null | grep -q "lslidar_driver_node"; then
            echo -e "   ${GREEN}✅ 激光雷达驱动节点运行中${NC}"
            
            # 检查话题
            echo "📡 激光雷达话题:"
            ros2 topic list 2>/dev/null | grep -E 'scan|pointcloud' | sed 's/^/      /'
            
            # 检查话题频率
            echo ""
            echo "📊 话题频率测试 (5秒):"
            timeout 5 ros2 topic hz /cleaning_robot/scan 2>/dev/null | sed 's/^/      /' &
            wait
        else
            echo -e "   ${RED}❌ 激光雷达驱动节点未运行${NC}"
            echo -e "   ${YELLOW}💡 启动驱动节点:${NC}"
            echo "      ros2 launch lslidar_driver cleaning_robot_n10p_net.launch.py"
        fi
    else
        echo -e "   ${RED}❌ ROS2环境未设置${NC}"
        echo -e "   ${YELLOW}💡 设置ROS2环境:${NC}"
        echo "      source /opt/ros/humble/setup.bash"
        echo "      source install/setup.bash"
    fi
}

provide_solutions() {
    echo -e "${BLUE}💡 常见问题解决方案${NC}"
    echo "=================================="
    
    echo "🔧 网络配置问题:"
    echo "   1. 配置静态IP:"
    echo "      sudo ip addr add $HOST_IP/24 dev eth0"
    echo "      sudo ip link set eth0 up"
    echo ""
    echo "   2. 检查网线连接"
    echo "   3. 确认激光雷达IP设置"
    echo ""
    
    echo "🔧 端口问题:"
    echo "   1. 关闭占用端口的程序:"
    echo "      sudo netstat -tulnp | grep :$MSOP_PORT"
    echo "      sudo kill <PID>"
    echo ""
    echo "   2. 修改端口配置"
    echo ""
    
    echo "🔧 防火墙问题:"
    echo "   1. 临时关闭防火墙:"
    echo "      sudo ufw disable"
    echo ""
    echo "   2. 添加端口规则:"
    echo "      sudo ufw allow $MSOP_PORT/udp"
    echo "      sudo ufw allow $DIFOP_PORT/udp"
    echo ""
    
    echo "🔧 ROS2问题:"
    echo "   1. 重新编译包:"
    echo "      colcon build --packages-select lslidar_driver"
    echo ""
    echo "   2. 检查配置文件:"
    echo "      cat src/lslidar_driver/params/lidar_net_ros2/cleaning_robot_n10p_net.yaml"
}

# 主诊断流程
main() {
    echo "开始全面诊断..."
    echo ""
    
    diagnose_network_interface
    echo ""
    
    diagnose_network_connectivity
    echo ""
    
    diagnose_port_status
    echo ""
    
    diagnose_firewall
    echo ""
    
    diagnose_traffic
    echo ""
    
    diagnose_ros_nodes
    echo ""
    
    provide_solutions
    
    echo ""
    echo -e "${GREEN}🎯 诊断完成！${NC}"
    echo "如需进一步帮助，请查看上述建议或联系技术支持。"
}

# 运行主函数
main 