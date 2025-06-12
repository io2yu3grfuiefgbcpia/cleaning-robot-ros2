#!/bin/bash

# 清洁机器人工作空间代码同步脚本（支持镜像站）
# 自动从远程仓库拉取最新代码到本地工作空间

set -e  # 遇到错误时停止执行

echo "🤖 清洁机器人工作空间代码同步脚本（镜像版）"
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

echo "📡 检查当前远程仓库配置..."
git remote -v

# GitHub镜像站列表
MIRRORS=(
    "https://ghproxy.com/https://github.com"
    "https://github.com.cnpmjs.org"
    "https://hub.fastgit.xyz"
    "https://github.com"
)

REPO_PATH="io2yu3grfuiefgbcpia/cleaning-robot-ros2.git"
ORIGINAL_URL="https://github.com/$REPO_PATH"

# 测试网络连接函数
test_connection() {
    local url=$1
    echo "🔍 测试连接: $url"
    if timeout 10 git ls-remote "$url/$REPO_PATH" > /dev/null 2>&1; then
        echo "✅ 连接成功"
        return 0
    else
        echo "❌ 连接失败"
        return 1
    fi
}

# 寻找可用的镜像站
echo ""
echo "🌐 正在寻找可用的GitHub镜像站..."
WORKING_MIRROR=""

for mirror in "${MIRRORS[@]}"; do
    if test_connection "$mirror"; then
        WORKING_MIRROR="$mirror"
        break
    fi
done

if [ -z "$WORKING_MIRROR" ]; then
    echo ""
    echo "❌ 所有镜像站都无法连接，请检查网络设置"
    echo "💡 建议:"
    echo "   1. 检查网络连接"
    echo "   2. 配置代理服务器"
    echo "   3. 使用VPN"
    echo "   4. 手动下载代码包"
    exit 1
fi

echo ""
echo "🎯 使用镜像站: $WORKING_MIRROR"

# 临时更改远程仓库地址
MIRROR_URL="$WORKING_MIRROR/$REPO_PATH"
if [ "$WORKING_MIRROR" != "https://github.com" ]; then
    echo "🔄 临时切换到镜像站..."
    git remote set-url origin "$MIRROR_URL"
fi

echo ""
echo "📥 获取远程仓库最新更改..."
if git fetch origin; then
    echo "✅ 成功获取远程更改"
else
    echo "❌ 获取远程更改失败"
    # 恢复原始URL
    git remote set-url origin "$ORIGINAL_URL"
    exit 1
fi

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
            # 恢复原始URL
            git remote set-url origin "$ORIGINAL_URL"
            exit 1
            ;;
        *)
            echo "❌ 无效选择，取消同步"
            # 恢复原始URL
            git remote set-url origin "$ORIGINAL_URL"
            exit 1
            ;;
    esac
else
    echo "⬇️  拉取远程代码..."
    git pull origin main
fi

# 恢复原始URL
if [ "$WORKING_MIRROR" != "https://github.com" ]; then
    echo ""
    echo "🔄 恢复原始仓库地址..."
    git remote set-url origin "$ORIGINAL_URL"
fi

echo ""
echo "🧹 清理不需要的目录..."
# 删除可能存在的重复目录
if [ -d "$HOME/cleaning-robot-ros2" ]; then
    echo "🗑️  删除重复的 cleaning-robot-ros2 目录..."
    rm -rf "$HOME/cleaning-robot-ros2"
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