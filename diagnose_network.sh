#!/bin/bash

# WSL网络诊断和修复脚本
# 用于解决apt更新连接问题

echo "🔍 WSL网络诊断和修复工具"
echo "=================================="

# 检查网络连通性
echo "1. 检查基础网络连通性..."
ping -c 2 8.8.8.8 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ 基础网络连通正常"
else
    echo "❌ 基础网络连通失败"
    exit 1
fi

# 检查DNS解析
echo "2. 检查DNS解析..."
nslookup archive.ubuntu.com > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ DNS解析正常"
else
    echo "❌ DNS解析失败"
fi

# 检查apt源连接
echo "3. 测试apt源连接..."
curl -s --connect-timeout 5 http://mirrors.aliyun.com/ubuntu/dists/jammy/Release > /dev/null
if [ $? -eq 0 ]; then
    echo "✅ 阿里云镜像源连接正常"
else
    echo "⚠️  阿里云镜像源连接异常"
fi

# 检查IPv6状态
echo "4. 检查IPv6状态..."
if [ "$(cat /proc/sys/net/ipv6/conf/all/disable_ipv6)" = "1" ]; then
    echo "✅ IPv6已禁用（推荐）"
else
    echo "⚠️  IPv6未禁用，可能导致连接问题"
    echo "   运行以下命令禁用IPv6："
    echo "   sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1"
    echo "   sudo sysctl -w net.ipv6.conf.default.disable_ipv6=1"
fi

# 检查软件源配置
echo "5. 检查软件源配置..."
if grep -q "mirrors.aliyun.com" /etc/apt/sources.list 2>/dev/null; then
    echo "✅ 已配置国内镜像源"
else
    echo "⚠️  未配置国内镜像源"
    echo "   建议更换为阿里云镜像源以提高下载速度"
fi

# 提供修复选项
echo ""
echo "🔧 快速修复选项："
echo "1. 如果apt更新失败，运行："
echo "   ./fix_wsl_network.sh"
echo "2. 如果需要重新配置镜像源，运行："
echo "   sudo cp /etc/apt/sources.list.backup /etc/apt/sources.list"
echo "   然后重新配置镜像源"

echo ""
echo "诊断完成！" 