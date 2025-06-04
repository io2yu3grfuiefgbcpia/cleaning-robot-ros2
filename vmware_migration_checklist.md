# VMware Ubuntu 迁移检查清单 ✅

## 📋 迁移前准备

### WSL环境备份
- [ ] 备份当前项目文件
  ```bash
  cd /home/yys/cleaning_robot_ws
  tar -czf cleaning_robot_backup_$(date +%Y%m%d).tar.gz .
  ```
- [ ] 记录重要配置（IP地址、设备路径等）
- [ ] 导出Git配置（如果使用）
  ```bash
  git config --list > git_config_backup.txt
  ```

### VMware虚拟机准备
- [ ] 创建新的Ubuntu 22.04虚拟机
- [ ] 分配足够资源：
  - [ ] 内存: ≥8GB
  - [ ] 磁盘: ≥100GB  
  - [ ] CPU: ≥4核
- [ ] 配置硬件虚拟化
- [ ] 设置USB 3.1控制器
- [ ] 配置桥接网络

## 🛠️ 系统安装配置

### Ubuntu基础配置
- [ ] 安装Ubuntu 22.04.3 LTS
- [ ] 创建用户账户（建议与WSL用户名相同）
- [ ] 启用自动登录
- [ ] 系统更新
  ```bash
  sudo apt update && sudo apt upgrade -y
  ```

### VMware Tools安装
- [ ] 安装VMware Tools
  ```bash
  sudo apt install open-vm-tools open-vm-tools-desktop
  ```
- [ ] 启用共享文件夹功能
- [ ] 配置剪贴板共享
- [ ] 测试拖拽文件功能

## 🚀 环境安装

### 自动化安装
- [ ] 运行环境安装脚本
  ```bash
  # 将setup_vmware_environment.sh传输到虚拟机
  chmod +x setup_vmware_environment.sh
  ./setup_vmware_environment.sh
  ```

### 手动验证安装
- [ ] ROS2环境测试
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 --version
  ```
- [ ] Python依赖验证
  ```bash
  python3 -c "import cv2, numpy; print('OpenCV OK')"
  ```
- [ ] 编译工具验证
  ```bash
  colcon --version
  ```

## 📦 项目迁移

### 方法1: 共享文件夹（推荐）
- [ ] 在VMware中配置共享文件夹
- [ ] 挂载共享文件夹
  ```bash
  sudo mkdir -p /mnt/hgfs
  sudo mount -t fuse.vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other
  ```
- [ ] 复制项目文件
  ```bash
  cd ~/cleaning_robot_ws
  cp -r /mnt/hgfs/cleaning_robot_ws/* .
  chmod +x *.sh
  ```

### 方法2: Git迁移
- [ ] 在WSL中初始化Git仓库并推送
- [ ] 在VMware中克隆项目
  ```bash
  git clone <your-repo-url> ~/cleaning_robot_ws
  ```

### 方法3: 文件传输
- [ ] 使用SCP/SFTP传输tar包
- [ ] 在VMware中解压
  ```bash
  tar -xzf cleaning_robot_backup.tar.gz
  ```

## 🔧 硬件配置

### 串口设备配置
- [ ] 连接USB转串口适配器
- [ ] 在VMware中添加USB设备
- [ ] 验证设备识别
  ```bash
  ls -la /dev/ttyUSB*
  ```
- [ ] 设置权限
  ```bash
  sudo usermod -a -G dialout $USER
  sudo chmod 666 /dev/ttyUSB*
  ```

### 网络激光雷达配置
- [ ] 配置桥接网络
- [ ] 设置静态IP
  ```bash
  sudo ip addr add 192.168.1.102/24 dev ens33
  ```
- [ ] 测试激光雷达连接
  ```bash
  ping 192.168.1.200
  ```

### 摄像头配置
- [ ] 连接USB摄像头
- [ ] 在VMware中添加摄像头设备
- [ ] 验证摄像头
  ```bash
  ls -la /dev/video*
  cheese  # 测试摄像头
  ```

### 电机控制器配置
- [ ] 连接电机控制器
- [ ] 验证串口设备
- [ ] 测试通信

## 🔨 项目编译

### 编译项目
- [ ] 进入工作目录
  ```bash
  cd ~/cleaning_robot_ws
  ```
- [ ] 编译所有包
  ```bash
  colcon build
  ```
- [ ] 加载环境
  ```bash
  source install/setup.bash
  ```
- [ ] 验证编译结果
  ```bash
  ros2 pkg list | grep cleaning_robot
  ```

### 解决编译问题
- [ ] 安装缺失依赖
  ```bash
  sudo apt install ros-humble-diagnostic-updater libpcap-dev
  ```
- [ ] 清理重新编译（如需要）
  ```bash
  rm -rf build/ install/ log/
  colcon build
  ```

## 🧪 硬件测试

### 激光雷达测试
#### 串口版测试
- [ ] 启动串口版驱动
  ```bash
  ros2 launch lslidar_driver cleaning_robot_n10p.launch.py
  ```
- [ ] 检查激光数据
  ```bash
  ros2 topic echo /cleaning_robot/scan --once
  ```

#### 网络版测试
- [ ] 启动网络版驱动
  ```bash
  ros2 launch lslidar_driver cleaning_robot_n10p_net.launch.py
  ```
- [ ] 检查激光数据
  ```bash
  ros2 topic echo /cleaning_robot/scan --once
  ```

### 摄像头测试
- [ ] 启动立体视觉
  ```bash
  ros2 run cleaning_robot_perception stereo_processor_node
  ```
- [ ] 检查图像话题
  ```bash
  ros2 topic list | grep image
  ```

### 系统集成测试
- [ ] 运行硬件检查
  ```bash
  ./check_hardware.sh
  ```
- [ ] 运行集成测试
  ```bash
  python3 test_n10p_integration.py  # 串口版
  python3 test_n10p_network.py      # 网络版
  ```

## 🎯 功能验证

### SLAM建图测试
- [ ] 启动SLAM系统
  ```bash
  ros2 launch cleaning_robot_navigation cleaning_robot_n10p.launch.py
  ```
- [ ] 验证RViz显示
- [ ] 测试地图构建

### 导航功能测试
- [ ] 设置初始位置
- [ ] 测试目标点导航
- [ ] 验证避障功能

### 清扫功能测试
- [ ] 启动清扫控制器
  ```bash
  ros2 run cleaning_robot_control cleaning_controller_node
  ```
- [ ] 运行清扫测试
  ```bash
  python3 test_cleaning.py
  ```

## 📊 性能优化

### VMware优化
- [ ] 调整虚拟内存设置
- [ ] 禁用不必要的视觉效果
- [ ] 配置网络缓冲区
- [ ] 设置CPU性能模式

### 系统优化
- [ ] 关闭不必要的服务
  ```bash
  sudo systemctl disable snapd bluetooth
  ```
- [ ] 设置交换文件（如需要）
- [ ] 配置实时性能

## ✅ 最终验证

### 完整系统测试
- [ ] 运行完整演示
  ```bash
  ./demo_n10p_complete.sh
  ```
- [ ] 测试所有功能模块
- [ ] 验证硬件响应时间
- [ ] 检查资源占用

### 文档和配置
- [ ] 更新README文档
- [ ] 记录硬件配置
- [ ] 保存网络设置
- [ ] 备份工作环境

## 🚨 常见问题排查

### USB设备问题
- [ ] 检查VMware USB设置
- [ ] 验证设备驱动
- [ ] 测试设备权限

### 网络连接问题
- [ ] 检查桥接网络配置
- [ ] 验证IP地址设置
- [ ] 测试防火墙规则

### 编译错误
- [ ] 检查ROS2环境
- [ ] 验证依赖包
- [ ] 清理重新编译

### 性能问题
- [ ] 监控资源使用
- [ ] 调整虚拟机配置
- [ ] 优化系统设置

## 📞 完成确认

### 功能确认清单
- [ ] ✅ 激光雷达数据正常
- [ ] ✅ 摄像头图像正常
- [ ] ✅ 电机控制正常
- [ ] ✅ SLAM建图正常
- [ ] ✅ 导航功能正常
- [ ] ✅ 清扫功能正常
- [ ] ✅ RViz可视化正常
- [ ] ✅ 所有测试通过

### 迁移成功标志
- [ ] 所有硬件设备正常工作
- [ ] 项目编译无错误
- [ ] 功能测试全部通过
- [ ] 性能满足要求
- [ ] 可以进行实际调试

---

## 🎉 迁移完成！

恭喜！你已经成功将清扫机器人项目从WSL迁移到VMware Ubuntu 22.04。现在你可以：

1. **进行硬件调试** - 所有USB设备都可以直接连接使用
2. **测试真实硬件** - 激光雷达、摄像头、电机控制器等
3. **开发新功能** - 在真实环境中测试和优化算法
4. **准备实际部署** - 为真实机器人系统做准备

**下一步建议：**
- 熟悉VMware环境下的硬件操作
- 优化系统性能参数
- 开始真实硬件的调试和测试
- 考虑将代码提交到版本控制系统

**技术支持：**
如果在迁移过程中遇到问题，请：
1. 查看安装日志：`~/setup_logs/`
2. 运行硬件检测：`./check_hardware.sh`
3. 运行网络诊断：`./diagnose_n10p_network.sh`
4. 参考迁移指南：`migrate_to_vmware.md` 