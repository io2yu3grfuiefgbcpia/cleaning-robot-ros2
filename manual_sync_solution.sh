#!/bin/bash

# 手动同步解决方案脚本
# 当GitHub连接有问题时的备用方案

echo "🔧 GitHub连接问题解决方案"
echo "=================================="

WORKSPACE_DIR="/home/yys/cleaning_robot_ws"
cd "$WORKSPACE_DIR"

echo "当前情况分析："
echo "1. GitHub连接超时"
echo "2. 本地代码需要同步"
echo "3. 项目需要重新构建"

echo ""
echo "解决方案选择："
echo "1) 跳过远程同步，直接构建现有代码"
echo "2) 使用现有的远程代码进行本地合并"
echo "3) 重新配置网络设置"
echo "4) 手动处理本地更改"

read -p "请选择方案 (1/2/3/4): " choice

case $choice in
    1)
        echo "🔨 方案1: 直接构建现有代码"
        echo "=================================="
        
        # 保存当前更改
        echo "💾 保存当前工作状态..."
        git add -A
        git stash push -m "手动保存 - $(date '+%Y-%m-%d %H:%M:%S')"
        
        # 强制重置到已知状态
        echo "🔄 重置到稳定状态..."
        git reset --hard HEAD
        
        # 清理构建目录
        echo "🧹 清理构建目录..."
        rm -rf build/ install/ log/
        
        # 重新构建
        echo "🔨 开始构建..."
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install
        
        if [ $? -eq 0 ]; then
            echo "✅ 构建成功！"
            echo "现在可以运行: source install/setup.bash"
        else
            echo "❌ 构建失败"
            # 恢复更改
            git stash pop
        fi
        ;;
        
    2)
        echo "🔄 方案2: 本地合并处理"
        echo "=================================="
        
        # 检查本地状态
        echo "📊 当前本地状态:"
        git status --short
        
        echo ""
        echo "🔀 尝试本地合并..."
        
        # 如果有已获取的远程更改，尝试合并
        if git log HEAD..origin/main --oneline | head -5; then
            echo "发现远程更改，准备合并..."
            
            # 保存本地更改
            git stash push -m "合并前保存 - $(date '+%Y-%m-%d %H:%M:%S')"
            
            # 合并远程更改
            git merge origin/main
            
            # 恢复本地更改
            if git stash list | grep -q "合并前保存"; then
                git stash pop
            fi
            
            # 重新构建
            echo "🔨 重新构建项目..."
            rm -rf build/ install/ log/
            source /opt/ros/humble/setup.bash
            colcon build --symlink-install
        else
            echo "未找到远程更改，直接构建现有代码..."
            rm -rf build/ install/ log/
            source /opt/ros/humble/setup.bash
            colcon build --symlink-install
        fi
        ;;
        
    3)
        echo "🌐 方案3: 网络配置建议"
        echo "=================================="
        
        echo "网络连接问题可能的解决方法："
        echo ""
        echo "1. 检查DNS设置:"
        echo "   sudo nano /etc/systemd/resolved.conf"
        echo "   添加: DNS=8.8.8.8 1.1.1.1"
        echo "   重启: sudo systemctl restart systemd-resolved"
        echo ""
        echo "2. 配置Git代理 (如果有代理):"
        echo "   git config --global http.proxy http://proxy:port"
        echo "   git config --global https.proxy https://proxy:port"
        echo ""
        echo "3. 或者取消代理:"
        echo "   git config --global --unset http.proxy"
        echo "   git config --global --unset https.proxy"
        echo ""
        echo "4. 增加Git超时时间:"
        echo "   git config --global http.lowSpeedLimit 1000"
        echo "   git config --global http.lowSpeedTime 300"
        echo ""
        echo "5. 使用SSH替代HTTPS (需要配置SSH密钥):"
        echo "   git remote set-url origin git@github.com:io2yu3grfuiefgbcpia/cleaning-robot-ros2.git"
        
        read -p "是否现在配置这些设置？(y/n): " configure
        if [ "$configure" = "y" ]; then
            echo "🔧 配置Git网络设置..."
            git config --global http.lowSpeedLimit 1000
            git config --global http.lowSpeedTime 300
            git config --global http.postBuffer 524288000
            echo "✅ Git网络设置已配置"
        fi
        ;;
        
    4)
        echo "✋ 方案4: 手动处理本地更改"
        echo "=================================="
        
        echo "📋 当前未提交的更改:"
        git status
        
        echo ""
        echo "处理选项:"
        echo "a) 查看具体更改内容"
        echo "b) 提交所有更改"
        echo "c) 丢弃所有更改"
        echo "d) 选择性处理"
        
        read -p "请选择 (a/b/c/d): " handle_choice
        
        case $handle_choice in
            a)
                echo "📄 显示更改内容:"
                git diff --stat
                git diff
                ;;
            b)
                echo "💾 提交所有更改..."
                git add -A
                git commit -m "手动提交本地更改 - $(date '+%Y-%m-%d %H:%M:%S')"
                echo "✅ 更改已提交"
                ;;
            c)
                echo "🗑️  丢弃所有更改..."
                git reset --hard HEAD
                git clean -fd
                echo "✅ 更改已丢弃"
                ;;
            d)
                echo "🎯 进入交互模式..."
                git add -i
                ;;
        esac
        
        # 重新构建
        echo "🔨 重新构建项目..."
        rm -rf build/ install/ log/
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install
        ;;
        
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac

echo ""
echo "🎉 处理完成！"
echo ""
echo "📋 接下来的步骤:"
echo "1. source install/setup.bash"
echo "2. 测试系统: ros2 launch cleaning_robot_description robot_state_publisher.launch.py"
echo "3. 如果需要硬件，连接Orbbec相机和激光雷达" 