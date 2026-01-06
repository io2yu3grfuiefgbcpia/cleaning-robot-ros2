# Git同步脚本使用指南

## 📋 功能说明

`git_sync.sh` 是一个自动化脚本，用于从远程Git仓库获取最新代码并更新本地工作空间。

## 🚀 快速使用

### 基本用法（推荐）

```bash
cd ~/cleaning_robot_ws
./scripts/utils/git_sync.sh --auto
```

`--auto` 模式会自动处理：
- ✅ 忽略未跟踪文件（不影响同步）
- ✅ 自动暂存已修改的文件
- ✅ 从远程拉取最新代码

### 其他用法

#### 1. 交互式模式（首次使用或需要选择）

```bash
./scripts/utils/git_sync.sh
```

当检测到本地有修改时，会提示你选择处理方式。

#### 2. 强制覆盖模式

```bash
./scripts/utils/git_sync.sh --force
```

⚠️ **警告**: 这会丢弃所有本地更改！

#### 3. 查看帮助

```bash
./scripts/utils/git_sync.sh --help
```

#### 4. 重置配置

```bash
./scripts/utils/git_sync.sh --reset-config
```

## ⚙️ 当前配置

配置文件位置: `~/.cleaning_robot_git_config`

当前配置：
- **远程仓库**: `origin` → `https://github.com/io2yu3grfuiefgbcpia/cleaning-robot-ros2.git`
- **分支**: `main`
- **强制覆盖**: `否`

## 📝 工作流程

1. **检查配置**: 读取配置文件或提示输入
2. **检查远程连接**: 验证远程仓库是否可访问
3. **检查本地状态**: 
   - 区分已修改文件和未跟踪文件
   - 未跟踪文件不影响同步
4. **处理本地更改**:
   - 自动模式: 暂存已修改文件
   - 强制模式: 丢弃所有更改
   - 交互模式: 提示选择
5. **获取远程代码**: `git fetch`
6. **更新本地代码**: `git pull` 或 `git reset --hard`
7. **恢复暂存**: 如果有暂存的文件，自动恢复

## 🔧 高级选项

### 命令行参数

```bash
./scripts/utils/git_sync.sh \
  --remote-url <URL> \
  --remote-name <NAME> \
  --branch <BRANCH> \
  --force \
  --auto
```

### 示例

```bash
# 使用不同的远程仓库
./scripts/utils/git_sync.sh \
  --remote-url https://github.com/user/repo.git \
  --branch develop \
  --auto

# 强制同步（丢弃本地更改）
./scripts/utils/git_sync.sh --force
```

## ⚠️ 注意事项

1. **未跟踪文件**: 脚本不会删除未跟踪的文件，它们不会影响同步
2. **暂存的文件**: 使用自动模式时，已修改的文件会被暂存，同步后会自动恢复
3. **冲突处理**: 如果恢复暂存时发生冲突，需要手动解决
4. **配置文件**: 首次运行会创建配置文件，后续运行会自动读取

## 🐛 故障排除

### 问题1: 远程仓库连接失败

```bash
# 检查网络连接
ping github.com

# 检查远程仓库配置
git remote -v

# 重新配置
./scripts/utils/git_sync.sh --reset-config
```

### 问题2: 本地有冲突

```bash
# 查看暂存列表
git stash list

# 查看暂存内容
git stash show

# 手动恢复
git stash pop

# 或丢弃暂存
git stash drop
```

### 问题3: 需要强制同步

```bash
# 强制覆盖所有本地更改
./scripts/utils/git_sync.sh --force
```

## 📞 支持

如有问题，请查看脚本帮助：
```bash
./scripts/utils/git_sync.sh --help
```

