#!/bin/bash

# WSL网络问题快速修复脚本

echo "🔧 WSL网络问题快速修复工具"
echo "=================================="

# 检查是否以root身份运行
if [ "$EUID" -eq 0 ]; then
    echo "请不要以root身份运行此脚本"
    exit 1
fi

echo "1. 禁用IPv6..."
sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1
sudo sysctl -w net.ipv6.conf.default.disable_ipv6=1

echo "2. 备份当前软件源配置..."
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup.$(date +%Y%m%d_%H%M%S)

echo "3. 配置阿里云镜像源..."
sudo tee /etc/apt/sources.list > /dev/null << 'EOF'
# 阿里云镜像源 - Ubuntu 22.04 (jammy)
deb http://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse

deb http://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse

deb http://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse

deb http://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse
EOF

echo "4. 永久禁用IPv6..."
if ! grep -q "net.ipv6.conf.all.disable_ipv6" /etc/sysctl.conf; then
    echo "net.ipv6.conf.all.disable_ipv6 = 1" | sudo tee -a /etc/sysctl.conf
    echo "net.ipv6.conf.default.disable_ipv6 = 1" | sudo tee -a /etc/sysctl.conf
fi

echo "5. 测试软件源..."
sudo apt update

if [ $? -eq 0 ]; then
    echo "✅ 网络修复成功！"
    echo ""
    echo "现在您可以正常使用以下命令："
    echo "  sudo apt update"
    echo "  sudo apt install <package_name>"
else
    echo "❌ 修复失败，请检查网络连接"
    exit 1
fi

echo ""
echo "修复完成！" 