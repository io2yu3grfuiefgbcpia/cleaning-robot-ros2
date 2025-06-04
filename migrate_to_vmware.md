# 从WSL迁移到VMware Ubuntu 22.04 硬件调试指南

## 📋 概述

本指南将帮助你将清扫机器人项目从WSL环境迁移到VMware Ubuntu 22.04，以便进行硬件调试和开发。

## 🎯 迁移的优势

| 特性 | WSL | VMware Ubuntu |
|------|-----|---------------|
| USB设备支持 | 有限制 | 完全支持 |
| 串口设备 | 需要USB/IP | 直接支持 |
| 网络硬件 | 虚拟网络 | 可直通物理网卡 |
| 激光雷达连接 | 复杂配置 | 原生支持 |
| 摄像头支持 | 有限制 | 完全支持 |
| 实时性能 | 较差 | 更好 |

## 🛠️ 准备工作

### 1. VMware虚拟机配置

#### 基础配置建议
- **内存**: 最少4GB，推荐8GB+
- **磁盘**: 最少50GB，推荐100GB+
- **CPU核心**: 最少2核，推荐4核+
- **网络**: 桥接网络（用于激光雷达网络版）

#### 硬件虚拟化设置
```
处理器 -> 虚拟化引擎:
☑️ 虚拟化Intel VT-x/EPT或AMD-V/RVI(V)
☑️ 虚拟化CPU性能计数器(U)
☑️ 虚拟化IOMMU (IO内存管理单元)(I)
```

#### USB设备支持
```
USB控制器:
☑️ USB兼容性: USB 3.1
☑️ 显示所有USB输入设备
```

### 2. Ubuntu 22.04安装

1. **下载Ubuntu 22.04.3 LTS Desktop**
   ```bash
   # 推荐下载地址
   https://ubuntu.com/download/desktop
   ```

2. **安装配置**
   - 用户名建议与WSL保持一致
   - 启用自动登录（调试方便）
   - 安装期间选择最小安装

## 📦 项目迁移步骤

### 步骤1: 创建项目目录

```bash
# 创建与WSL相同的目录结构
mkdir -p ~/cleaning_robot_ws
cd ~/cleaning_robot_ws
```

### 步骤2: 传输项目文件

#### 方法1: 使用共享文件夹（推荐）

1. **安装VMware Tools**
   ```bash
   sudo apt update
   sudo apt install open-vm-tools open-vm-tools-desktop
   ```

2. **设置共享文件夹**
   - VMware菜单: 虚拟机 -> 设置 -> 选项 -> 共享文件夹
   - 启用共享文件夹功能
   - 添加主机路径：指向WSL项目目录
   - 挂载点：`/mnt/hgfs/cleaning_robot_ws`

3. **复制项目文件**
   ```bash
   # 挂载共享文件夹
   sudo mkdir -p /mnt/hgfs
   sudo mount -t fuse.vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other
   
   # 复制项目文件
   cp -r /mnt/hgfs/cleaning_robot_ws/* ~/cleaning_robot_ws/
   
   # 设置正确权限
   chmod +x ~/cleaning_robot_ws/*.sh
   ```

#### 方法2: 使用SCP/SFTP

```bash
# 在WSL中打包项目
cd /home/yys/cleaning_robot_ws
tar -czf cleaning_robot_ws.tar.gz .

# 传输到VMware Ubuntu（假设Ubuntu IP为192.168.1.100）
scp cleaning_robot_ws.tar.gz username@192.168.1.100:~/

# 在VMware Ubuntu中解压
cd ~/cleaning_robot_ws
tar -xzf ../cleaning_robot_ws.tar.gz
chmod +x *.sh
```

#### 方法3: 使用Git（推荐用于版本控制）

```bash
# 在WSL中初始化Git仓库
cd /home/yys/cleaning_robot_ws
git init
git add .
git commit -m "Initial commit for VMware migration"

# 推送到远程仓库（GitHub/GitLab等）
git remote add origin https://github.com/your-username/cleaning_robot_ws.git
git push -u origin main

# 在VMware Ubuntu中克隆
cd ~
git clone https://github.com/your-username/cleaning_robot_ws.git
cd cleaning_robot_ws
chmod +x *.sh
```

## 🔧 硬件配置

### 1. 激光雷达连接配置

#### 串口版N10P激光雷达

```bash
# 连接USB转串口适配器
# 在VMware中添加USB设备：虚拟机 -> 可移动设备 -> USB串口适配器 -> 连接

# 检查设备识别
ls -la /dev/ttyUSB*

# 设置权限
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB*
```

#### 网络版N10P激光雷达

```bash
# 配置网络桥接
# VMware设置：网络适配器 -> 桥接网络 -> 复制物理网络连接状态

# 设置静态IP（与激光雷达同网段）
sudo ip addr add 192.168.1.102/24 dev ens33
sudo ip link set ens33 up

# 测试连接
ping 192.168.1.200
```

### 2. 摄像头配置

```bash
# 连接USB摄像头
# VMware：虚拟机 -> 可移动设备 -> USB摄像头 -> 连接

# 检查摄像头设备
ls -la /dev/video*

# 测试摄像头
sudo apt install cheese
cheese  # 测试摄像头是否正常工作
```

### 3. 电机控制器配置

```bash
# 连接电机控制器USB设备
# VMware：虚拟机 -> 可移动设备 -> 电机控制器 -> 连接

# 检查设备
ls -la /dev/ttyUSB*

# 通常电机控制器为/dev/ttyUSB0，激光雷达为/dev/ttyUSB1
```

## 🚀 环境安装

### 自动安装脚本

运行项目中的安装脚本：

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装ROS2和依赖
./install_ros2.sh

# 如果网络有问题，先运行网络修复
./fix_wsl_network.sh  # 这个脚本在Ubuntu中也适用

# 设置激光雷达权限
./setup_lslidar_serial.sh
```

### 手动安装关键组件

```bash
# 安装ROS2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install python3-colcon-common-extensions -y

# 安装项目依赖
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    python3-opencv \
    libpcap-dev \
    libeigen3-dev

# 设置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/cleaning_robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🔨 编译和测试

### 1. 编译项目

```bash
cd ~/cleaning_robot_ws

# 编译所有包
colcon build

# 加载环境
source install/setup.bash
```

### 2. 硬件连接测试

#### 测试激光雷达连接

```bash
# 串口版测试
ros2 launch lslidar_driver cleaning_robot_n10p.launch.py

# 网络版测试
ros2 launch lslidar_driver cleaning_robot_n10p_net.launch.py

# 检查数据
ros2 topic echo /cleaning_robot/scan
```

#### 测试摄像头

```bash
# 启动立体视觉处理
ros2 run cleaning_robot_perception stereo_processor_node

# 检查图像话题
ros2 topic list | grep image
```

## 🧪 完整系统测试

### 1. 硬件集成测试

```bash
# 运行硬件检查
./demo_n10p_complete.sh

# 选择对应的测试选项：
# 1. 检查硬件连接状态
# 6. 检查网络连接状态（网络版）
```

### 2. 功能测试

```bash
# 串口版完整测试
python3 test_n10p_integration.py

# 网络版完整测试
python3 test_n10p_network.py

# 清扫功能测试
python3 test_cleaning.py
```

## 🔧 常见问题解决

### 1. USB设备不识别

```bash
# 检查VMware USB控制器设置
# 虚拟机 -> 设置 -> 硬件 -> USB控制器
# 确保选择了USB 3.1兼容性

# 手动连接设备
# 虚拟机 -> 可移动设备 -> 选择对应设备 -> 连接

# 检查设备权限
sudo dmesg | tail -20  # 查看设备连接日志
```

### 2. 网络激光雷达连接问题

```bash
# 检查网络配置
ip addr show
ping 192.168.1.200

# 如果ping不通，检查VMware网络设置
# 确保使用桥接网络，不是NAT

# 运行网络诊断
./diagnose_n10p_network.sh
```

### 3. 权限问题

```bash
# 添加用户到相关组
sudo usermod -a -G dialout,video,audio $USER

# 重新登录或重启虚拟机
sudo reboot
```

### 4. 编译错误

```bash
# 清理编译缓存
rm -rf build/ install/ log/

# 安装缺失依赖
sudo apt install ros-humble-diagnostic-updater

# 重新编译
colcon build --symlink-install
```

## 📊 性能优化

### 1. VMware设置优化

```bash
# 编辑VMware配置文件
echo 'mainMem.useNamedFile = "FALSE"' >> ~/.vmware/preferences
echo 'prefvmx.useRecommendedLockedMemSize = "TRUE"' >> ~/.vmware/preferences
echo 'MemTrimRate = "0"' >> ~/.vmware/preferences
```

### 2. 系统性能调优

```bash
# 关闭不必要的服务
sudo systemctl disable snapd
sudo systemctl disable bluetooth

# 设置CPU性能模式
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

## ✅ 迁移检查清单

- [ ] VMware虚拟机配置完成
- [ ] Ubuntu 22.04安装完成
- [ ] 项目文件迁移完成
- [ ] ROS2环境安装完成
- [ ] 激光雷达硬件连接正常
- [ ] 摄像头设备识别正常
- [ ] 电机控制器连接正常
- [ ] 项目编译成功
- [ ] 激光雷达数据正常
- [ ] 立体视觉功能正常
- [ ] 清扫功能测试通过

## 🎉 完成后的优势

使用VMware Ubuntu进行硬件调试后，你将获得：

1. **完整的硬件支持** - 所有USB设备、串口设备都可以直接使用
2. **更好的实时性能** - 减少虚拟化层的延迟
3. **原生的Linux环境** - 更好的ROS2兼容性
4. **灵活的网络配置** - 支持各种网络模式的激光雷达
5. **便于调试** - 可以直接访问硬件资源

## 📞 后续支持

迁移完成后，你可以：

1. 使用所有原有的脚本和工具
2. 在真实硬件上测试和调试
3. 开发更复杂的硬件集成功能
4. 准备实际部署的系统

如有问题，请参考项目中的其他文档或创建issue。 