# 清扫机器人可视化指南

## 🖼️ 查看机器人图形和路径

### 方法1: 使用RViz2 (推荐)

1. **启动完整系统**
```bash
./start_cleaning_robot.sh complete
```

2. **在RViz2中查看的内容**
   - **机器人模型**: 3D机器人外观（蓝色底盘、红色激光雷达、黑色摄像头、绿色清扫刷）
   - **激光雷达数据**: 红点显示激光扫描点
   - **地图**: 黑白占用栅格地图
   - **清扫路径**: 绿色线条显示规划的清扫路径
   - **点云数据**: 彩色点云（双目摄像头生成）
   - **摄像头图像**: 左摄像头的实时图像

3. **RViz2界面说明**
   - 左侧面板：显示组件列表，可以开关各个显示项
   - 主视图：3D可视化窗口
   - 右下角：摄像头图像窗口

### 方法2: 使用命令行查看话题数据

```bash
# 查看地图数据
ros2 topic echo /map --once

# 查看清扫路径
ros2 topic echo /cleaning_robot/cleaning_path --once

# 查看激光雷达数据
ros2 topic echo /cleaning_robot/scan --once

# 查看机器人位置
ros2 topic echo /cleaning_robot/pose --once
```

### 方法3: 保存和导出可视化数据

```bash
# 保存地图为图像文件
ros2 run nav2_map_server map_saver_cli -f my_map

# 录制bag文件用于回放
ros2 bag record -a -o cleaning_session

# 回放数据
ros2 bag play cleaning_session
```

## 🎮 交互式操作

### 启动清扫任务查看路径
```bash
# 在新终端运行
./start_cleaning_robot.sh test
```

### 手动控制机器人（用于测试）
```bash
# 安装键盘控制工具
sudo apt install ros-humble-teleop-twist-keyboard

# 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cleaning_robot/cmd_vel
```

## 📊 实时监控界面

### 查看系统状态
```bash
# 查看所有运行的节点
ros2 node list

# 查看节点信息
ros2 node info /multi_sensor_slam

# 查看话题频率
ros2 topic hz /cleaning_robot/scan
ros2 topic hz /map
```

### 调试和故障排除
```bash
# 检查TF坐标变换
ros2 run tf2_tools view_frames

# 查看日志
ros2 node list
ros2 logs show <node_name>
```

## 🎯 具体可视化步骤

1. **启动系统**: `./start_cleaning_robot.sh complete`
2. **等待RViz2打开**: 会自动显示机器人模型
3. **启动清扫**: 在新终端运行 `./start_cleaning_robot.sh test`
4. **观察变化**: 
   - 地图逐渐构建（黑白栅格）
   - 绿色路径线条出现
   - 机器人沿路径移动
   - 激光点不断更新

## 📱 Web界面（可选扩展）

可以添加Web可视化界面：

```bash
# 安装web工具
sudo apt install ros-humble-rosbridge-suite

# 启动web服务器
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

然后可以通过浏览器访问可视化界面。 