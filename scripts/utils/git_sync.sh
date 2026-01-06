#!/bin/bash

# Git代码同步脚本
# 从远程仓库获取最新版本并更新本地代码

set -e  # 遇到错误时停止执行

# 配置文件路径
CONFIG_FILE="$HOME/.cleaning_robot_git_config"
WORKSPACE_DIR="/home/yys/cleaning_robot_ws"

echo "🔄 Git代码同步脚本"
echo "================================================"

# 切换到工作空间目录
if [ "$(pwd)" != "$WORKSPACE_DIR" ]; then
    echo "📁 切换到工作空间目录: $WORKSPACE_DIR"
    cd "$WORKSPACE_DIR"
fi

# 检查是否是git仓库
if [ ! -d ".git" ]; then
    echo "❌ 错误: 当前目录不是git仓库"
    echo "💡 提示: 如果需要初始化git仓库，请先运行: git init"
    exit 1
fi

# 读取或创建配置文件
load_git_config() {
    if [ -f "$CONFIG_FILE" ]; then
        echo "📋 读取配置文件: $CONFIG_FILE"
        source "$CONFIG_FILE"
    else
        echo "📝 首次运行，需要配置Git信息"
        echo ""
        read -p "请输入远程仓库地址 (remote URL): " REMOTE_URL
        read -p "请输入远程仓库名称 (默认: origin): " REMOTE_NAME
        REMOTE_NAME=${REMOTE_NAME:-origin}
        
        read -p "请输入分支名称 (默认: main): " BRANCH_NAME
        BRANCH_NAME=${BRANCH_NAME:-main}
        
        read -p "是否强制覆盖本地更改? (y/n, 默认: n): " FORCE_OVERWRITE
        FORCE_OVERWRITE=${FORCE_OVERWRITE:-n}
        
        # 保存配置
        cat > "$CONFIG_FILE" << EOF
# 清洁机器人Git同步配置
# 生成时间: $(date '+%Y-%m-%d %H:%M:%S')

REMOTE_URL="$REMOTE_URL"
REMOTE_NAME="$REMOTE_NAME"
BRANCH_NAME="$BRANCH_NAME"
FORCE_OVERWRITE="$FORCE_OVERWRITE"
EOF
        chmod 600 "$CONFIG_FILE"
        echo "✅ 配置已保存到: $CONFIG_FILE"
        echo ""
        
        # 设置远程仓库（如果不存在）
        if ! git remote | grep -q "^${REMOTE_NAME}$"; then
            echo "🔗 添加远程仓库: $REMOTE_NAME -> $REMOTE_URL"
            git remote add "$REMOTE_NAME" "$REMOTE_URL"
        else
            echo "🔗 更新远程仓库地址: $REMOTE_NAME -> $REMOTE_URL"
            git remote set-url "$REMOTE_NAME" "$REMOTE_URL"
        fi
    fi
}

# 支持命令行参数覆盖配置
while [[ $# -gt 0 ]]; do
    case $1 in
        --remote-url)
            REMOTE_URL="$2"
            shift 2
            ;;
        --remote-name)
            REMOTE_NAME="$2"
            shift 2
            ;;
        --branch)
            BRANCH_NAME="$2"
            shift 2
            ;;
        --force)
            FORCE_OVERWRITE="y"
            shift
            ;;
        --auto)
            AUTO_MODE="y"
            shift
            ;;
        --reset-config)
            echo "🗑️  删除配置文件..."
            rm -f "$CONFIG_FILE"
            echo "✅ 配置已删除，下次运行将重新配置"
            exit 0
            ;;
        --help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --remote-url URL     远程仓库地址"
            echo "  --remote-name NAME   远程仓库名称 (默认: origin)"
            echo "  --branch BRANCH      分支名称 (默认: main)"
            echo "  --force              强制覆盖本地更改"
            echo "  --auto               自动模式（忽略未跟踪文件，暂存已修改文件）"
            echo "  --reset-config       重置配置文件"
            echo "  --help               显示帮助信息"
            echo ""
            echo "示例:"
            echo "  $0"
            echo "  $0 --remote-url https://github.com/user/repo.git --branch main"
            echo "  $0 --force"
            exit 0
            ;;
        *)
            echo "❌ 未知参数: $1"
            echo "使用 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# 加载配置
load_git_config

# 显示当前配置
echo ""
echo "📊 当前Git配置:"
echo "  远程仓库: $REMOTE_NAME -> $REMOTE_URL"
echo "  分支: $BRANCH_NAME"
echo "  强制覆盖: $FORCE_OVERWRITE"
echo ""

# 检查远程仓库连接
echo "📡 检查远程仓库连接..."
if ! git remote | grep -q "^${REMOTE_NAME}$"; then
    echo "❌ 错误: 远程仓库 '$REMOTE_NAME' 不存在"
    echo "💡 提示: 使用 --reset-config 重新配置，或手动添加: git remote add $REMOTE_NAME $REMOTE_URL"
    exit 1
fi

git remote -v | grep "^${REMOTE_NAME}"

# 检查本地状态
echo ""
echo "📊 检查本地状态..."
MODIFIED_FILES=$(git status --porcelain | grep -E "^[ MARC]" || true)
UNTRACKED_FILES=$(git status --porcelain | grep "^??" || true)

if [ -n "$MODIFIED_FILES" ] || [ -n "$UNTRACKED_FILES" ]; then
    if [ -n "$MODIFIED_FILES" ]; then
        echo "⚠️  检测到已修改的文件:"
        echo "$MODIFIED_FILES" | sed 's/^/  /'
    fi
    
    if [ -n "$UNTRACKED_FILES" ]; then
        echo "📝 检测到未跟踪的文件（将不影响同步）:"
        echo "$UNTRACKED_FILES" | sed 's/^/  /'
    fi
    
    if [ -n "$MODIFIED_FILES" ]; then
        if [ "$FORCE_OVERWRITE" = "y" ]; then
            echo ""
            echo "🗑️  强制模式: 丢弃本地更改..."
            git reset --hard HEAD
            git clean -fd
        elif [ "$AUTO_MODE" = "y" ]; then
            echo ""
            echo "🤖 自动模式: 暂存已修改的文件..."
            git stash push -m "自动暂存 - $(date '+%Y-%m-%d %H:%M:%S')"
            STASHED=true
        else
            echo ""
            echo "选择处理方式:"
            echo "1) 暂存(stash)已修改文件并拉取远程代码（未跟踪文件不受影响）"
            echo "2) 放弃本地更改并强制拉取远程代码"
            echo "3) 取消同步，请手动处理冲突"
            
            read -p "请选择 (1/2/3): " choice
            
            case $choice in
                1)
                    echo "💾 暂存已修改文件..."
                    git stash push -m "自动暂存 - $(date '+%Y-%m-%d %H:%M:%S')"
                    STASHED=true
                    ;;
                2)
                    echo "🗑️  丢弃本地更改..."
                    git reset --hard HEAD
                    git clean -fd
                    ;;
                3)
                    echo "❌ 取消同步"
                    exit 1
                    ;;
                *)
                    echo "❌ 无效选择，取消同步"
                    exit 1
                    ;;
            esac
        fi
    else
        echo "✅ 只有未跟踪文件，不影响同步"
    fi
else
    echo "✅ 工作区干净，无未提交的更改"
fi

# 获取远程最新代码
echo ""
echo "📥 从远程获取最新代码..."
git fetch "$REMOTE_NAME"

# 检查远程分支是否存在
if ! git ls-remote --heads "$REMOTE_NAME" "$BRANCH_NAME" | grep -q "$BRANCH_NAME"; then
    echo "⚠️  警告: 远程分支 '$BRANCH_NAME' 不存在"
    echo "可用的远程分支:"
    git ls-remote --heads "$REMOTE_NAME" | sed 's/.*refs\/heads\///'
    read -p "是否继续使用本地分支? (y/n): " continue_choice
    if [ "$continue_choice" != "y" ]; then
        exit 1
    fi
fi

# 切换到目标分支（如果不在该分支）
CURRENT_BRANCH=$(git branch --show-current)
if [ "$CURRENT_BRANCH" != "$BRANCH_NAME" ]; then
    echo "🔄 切换到分支: $BRANCH_NAME"
    if git show-ref --verify --quiet refs/heads/"$BRANCH_NAME"; then
        git checkout "$BRANCH_NAME"
    else
        echo "📌 创建本地分支并跟踪远程分支..."
        git checkout -b "$BRANCH_NAME" "$REMOTE_NAME/$BRANCH_NAME"
    fi
fi

# 拉取或重置到远程版本
echo ""
echo "⬇️  更新本地代码到远程最新版本..."
if [ "$FORCE_OVERWRITE" = "y" ]; then
    echo "🔄 强制重置到远程分支..."
    git reset --hard "$REMOTE_NAME/$BRANCH_NAME"
else
    echo "🔄 合并远程更改..."
    git pull "$REMOTE_NAME" "$BRANCH_NAME" || {
        echo "❌ 拉取失败，尝试强制重置..."
        read -p "是否强制重置到远程版本? (y/n): " force_reset
        if [ "$force_reset" = "y" ]; then
            git reset --hard "$REMOTE_NAME/$BRANCH_NAME"
        else
            exit 1
        fi
    }
fi

# 恢复暂存的更改
if [ "$STASHED" = "true" ]; then
    echo ""
    echo "📦 恢复暂存的本地更改..."
    if git stash list | grep -q "自动暂存"; then
        git stash pop || {
            echo "⚠️  恢复时发生冲突，请手动解决:"
            echo "   git stash list    # 查看暂存列表"
            echo "   git stash show    # 查看暂存内容"
            echo "   解决冲突后运行: git stash drop"
        }
    fi
fi

# 显示更新结果
echo ""
echo "✅ 代码同步完成！"
echo ""
echo "📈 最新提交信息:"
git log --oneline -5

echo ""
echo "📊 当前状态:"
git status --short

echo ""
echo "🎉 本地代码已更新到远程最新版本"

