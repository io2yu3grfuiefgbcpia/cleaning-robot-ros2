# 旧项目功能合并记录

本次只合并能直接服务当前 ROS2 工程的部分，没有把旧项目目录整体复制进来。

来源目录：

- `/mnt/d/mew_wss/0000Car_CV`
- `/mnt/d/mew_wss/000Gemini2`

## 已合并内容

### 香橙派履带电机驱动

旧来源：

- `0000Car_CV/car_client/motors.py`
- `000Gemini2/motor_driver.py` 的引脚和电机控制思路

新位置：

- `src/cleaning_robot_control/cleaning_robot_control/track_motors.py`
- `src/cleaning_robot_control/cleaning_robot_control/track_motor_driver.py`
- `src/cleaning_robot_control/config/track_motor_driver.yaml`
- `src/cleaning_robot_control/launch/track_motor_driver.launch.py`

用途：

- 订阅 `/cleaning_robot/cmd_vel` 和可选 `/cmd_vel`
- 转成左右履带速度百分比
- 支持 `pwm_brake`、`l298n`
- 支持 `OPi.GPIO`、`RPi.GPIO`、`sysfs`、`mock`

虚拟机默认配置为：

```yaml
mock: true
```

香橙派接线确认后再改成：

```yaml
mock: false
```

单独测试：

```bash
ros2 launch cleaning_robot_control track_motor_driver.launch.py
```

### Gemini2 直接采集节点

旧来源：

- `0000Car_CV/car_client/gemini2.py`
- `0000Car_CV/car_client/orbbec_utils.py`
- `0000Car_CV/docs/04-Gemini2相机配置.md`

新位置：

- `src/cleaning_robot_perception/cleaning_robot_perception/gemini2_camera.py`
- `src/cleaning_robot_perception/cleaning_robot_perception/gemini2_camera_node.py`
- `src/cleaning_robot_perception/cleaning_robot_perception/orbbec_utils.py`
- `src/cleaning_robot_perception/config/gemini2_camera.yaml`
- `src/cleaning_robot_perception/launch/gemini2_camera.launch.py`

用途：

- 不依赖 `orbbec_camera` ROS 驱动时，也能通过 `pyorbbecsdk`、OpenCV OBSENSOR 或 UVC 读取 Gemini2
- 发布 `/cleaning_robot/gemini2/image_raw`
- 同时兼容发布到 `/cleaning_robot/camera/left/image_raw`，方便当前控制面板显示

单独测试：

```bash
ros2 launch cleaning_robot_perception gemini2_camera.launch.py
```

若香橙派上彩色流不在 `/dev/video0`，先查设备：

```bash
v4l2-ctl --list-devices
```

然后修改：

```yaml
device_id: 4
```

或：

```yaml
device_path: "/dev/video4"
```

## 完整系统启用方式

新增节点已接入 `cleaning_robot_full_system.launch.py`，但默认关闭。

香橙派实车可这样启动：

```bash
ros2 launch cleaning_robot_description cleaning_robot_full_system.launch.py \
  use_gemini2:=true \
  use_hardware_motors:=true
```

## 未原样合并内容

以下内容没有直接复制进当前 ROS2 工程：

- `0000Car_CV.zip`、`0000Car_CV.rar` 等归档文件
- `0000Car_CV/server` 的公网 WebSocket 中继服务器
- `000Gemini2/app.py` 的 Flask 单体 Web 服务
- 旧项目中的 Windows 打包脚本和串口粘贴部署脚本

原因：

- 当前工程已经转向 ROS2 包结构，直接复制旧单体服务会造成入口分散
- 旧服务器/网页控制逻辑更适合后续单独做成 `cleaning_robot_remote` 包
- 归档和部署中间产物不适合进入 Git

## 建议的下一步

1. 在虚拟机先构建确认包结构没有问题。
2. 推送 Git 到仓库。
3. 在香橙派拉取后先单独测试 Gemini2。
4. 架空履带或断开电机电源，测试 `track_motor_driver` 的 GPIO 输出方向。
5. 方向正确后再把 `mock` 改成 `false` 并低速实车测试。
