#!/bin/bash

# 清洁机器人工作空间代码同步脚本
# 自动从远程仓库拉取最新代码到本地工作空间

set -e  # 遇到错误时停止执行

echo "🤖 清洁机器人工作空间代码同步脚本"
echo "================================================"

# 检查是否在正确的工作空间目录
WORKSPACE_DIR="/home/yys/cleaning_robot_ws"
if [ "$(pwd)" != "$WORKSPACE_DIR" ]; then
    echo "📁 切换到工作空间目录: $WORKSPACE_DIR"
    cd "$WORKSPACE_DIR"
fi

# 检查是否是git仓库
if [ ! -d ".git" ]; then
    echo "❌ 错误: 当前目录不是git仓库"
    exit 1
fi

echo "📡 检查远程仓库连接..."
git remote -v

echo ""
echo "📥 获取远程仓库最新更改..."
git fetch origin

echo ""
echo "📊 检查本地状态..."
git status --porcelain

# 检查是否有本地未提交的更改
if [ -n "$(git status --porcelain)" ]; then
    echo ""
    echo "⚠️  检测到本地有未提交的更改"
    echo "选择处理方式:"
    echo "1) 暂存(stash)本地更改并拉取远程代码"
    echo "2) 放弃本地更改并强制拉取远程代码"
    echo "3) 取消同步，请手动处理冲突"
    
    read -p "请选择 (1/2/3): " choice
    
    case $choice in
        1)
            echo "💾 暂存本地更改..."
            git stash push -m "自动暂存 - $(date '+%Y-%m-%d %H:%M:%S')"
            echo "⬇️  拉取远程代码..."
            git pull origin main
            echo "📦 恢复本地更改..."
            if git stash list | grep -q "自动暂存"; then
                git stash pop
                echo "✅ 本地更改已恢复，请检查是否有冲突需要解决"
            fi
            ;;
        2)
            echo "🗑️  丢弃本地更改..."
            git reset --hard HEAD
            git clean -fd
            echo "⬇️  强制拉取远程代码..."
            git pull origin main
            ;;
        3)
            echo "❌ 取消同步"
            echo "请手动解决冲突后再运行此脚本"
            exit 1
            ;;
        *)
            echo "❌ 无效选择，取消同步"
            exit 1
            ;;
    esac
else
    echo "⬇️  拉取远程代码..."
    git pull origin main
fi

echo ""
echo "🧹 清理不需要的目录..."
# 删除可能存在的重复目录
if [ -d "~/cleaning-robot-ros2" ]; then
    echo "🗑️  删除重复的 cleaning-robot-ros2 目录..."
    rm -rf ~/cleaning-robot-ros2
fi

echo ""
echo "🔨 重新构建项目..."
echo "清理旧的构建文件..."
rm -rf build/ install/ log/

echo "构建ROS2包..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ 代码同步和构建完成！"
    echo ""
    echo "📋 下一步操作:"
    echo "1. source install/setup.bash"
    echo "2. 运行您的清洁机器人程序"
    echo ""
    echo "🚀 快速启动命令:"
    echo "   source install/setup.bash && ros2 launch cleaning_robot_description cleaning_robot_complete.launch.py"
else
    echo ""
    echo "❌ 构建失败，请检查错误信息"
    exit 1
fi

echo ""
echo "📈 显示最新提交信息:"
git log --oneline -5

echo ""
echo "�� 同步完成！工作空间已更新到最新版本" 