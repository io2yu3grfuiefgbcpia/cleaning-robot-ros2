# 脚本工具说明

本目录包含清扫机器人系统的所有工具脚本，按功能分类组织。

## 📁 目录结构

### `launch/` - 启动脚本
系统启动相关的脚本，用于启动不同类型的机器人系统。

- **start_cleaning_robot_n10p_net.sh** - N10P激光雷达网络版启动脚本（推荐）
- **start_cleaning_robot_n10p.sh** - N10P激光雷达串口版启动脚本
- **start_cleaning_robot_orbbec.sh** - Orbbec深度相机版启动脚本
- **start_cleaning_robot.sh** - 通用启动脚本（支持仿真和硬件模式）

### `setup/` - 安装和配置脚本
系统环境安装和硬件配置脚本。

- **setup_ros2_complete.sh** - ROS2 Humble完整环境安装脚本
- **setup_lslidar_serial.sh** - 镭神激光雷达串口权限配置
- **install_orbbec_dependencies.sh** - Orbbec深度相机依赖安装

### `diagnostic/` - 诊断工具
系统诊断和故障排查工具。

- **diagnose_n10p_network.sh** - N10P激光雷达网络连接诊断工具

### `utils/` - 实用工具
其他实用工具脚本。

- **start_lidar_viz.sh** - 激光雷达数据可视化工具（调用Python脚本）
- **demo_n10p_complete.sh** - 完整功能演示脚本（交互式菜单）
- **sync_remote_code.sh** - Git代码同步工具
- **lidar_rviz.py** - 激光雷达RViz风格可视化（模拟数据）
- **real_lidar_rviz.py** - 真实激光雷达数据可视化
- **simple_lidar_viewer.py** - 简化版激光雷达可视化

### `test/` - 测试脚本
系统功能测试脚本。

- **test_cleaning.py** - 清扫功能测试
- **test_n10p_integration.py** - N10P激光雷达集成测试（串口版）
- **test_n10p_network.py** - N10P激光雷达网络版测试
- **test_orbbec_camera.py** - Orbbec深度相机测试

## 🚀 快速使用

### 使用主启动脚本（推荐）

```bash
# 从项目根目录运行
cd ~/cleaning_robot_ws
./start.sh
```

### 直接使用脚本

```bash
# 启动N10P网络版
./scripts/launch/start_cleaning_robot_n10p_net.sh

# 配置串口权限
./scripts/setup/setup_lslidar_serial.sh

# 运行网络诊断
./scripts/diagnostic/diagnose_n10p_network.sh
```

## 📝 注意事项

1. 所有脚本都需要执行权限，如果遇到权限问题，运行：
   ```bash
   chmod +x scripts/**/*.sh
   ```

2. 启动脚本会自动检查ROS2环境，但建议先运行：
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

3. 网络版启动脚本支持环境变量配置：
   ```bash
   LIDAR_IP=192.168.1.200 HOST_IP=192.168.1.102 ./scripts/launch/start_cleaning_robot_n10p_net.sh
   ```

