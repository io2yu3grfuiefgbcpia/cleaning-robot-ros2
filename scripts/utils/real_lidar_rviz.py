#!/usr/bin/env python3

import subprocess
import sys
import os
import time

def main():
    """启动RViz2进行激光雷达实时可视化"""
    print("🎯 启动RViz2激光雷达可视化...")
    
    # RViz配置文件路径
    rviz_config_paths = [
        "src/cleaning_robot_navigation/rviz/cleaning_robot_n10p_net.rviz",
        "src/cleaning_robot_description/rviz/cleaning_robot.rviz",
        None  # 使用默认配置
    ]
    
    # 查找可用的RViz配置文件
    rviz_config = None
    for config_path in rviz_config_paths:
        if config_path is None:
            break
        if os.path.exists(config_path):
            rviz_config = config_path
            print(f"✅ 找到RViz配置文件: {rviz_config}")
            break
    
    try:
        # 构建RViz2启动命令
        if rviz_config:
            cmd = ["ros2", "run", "rviz2", "rviz2", "-d", rviz_config]
        else:
            print("⚠️  未找到专用RViz配置文件，使用默认配置")
            cmd = ["ros2", "run", "rviz2", "rviz2"]
        
        print(f"🚀 启动命令: {' '.join(cmd)}")
        
        # 启动RViz2
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        print(f"✅ RViz2已启动，PID: {process.pid}")
        print("📊 RViz2配置建议:")
        print("   1. 添加 LaserScan 显示，话题: /cleaning_robot/scan")
        print("   2. 添加 PointCloud2 显示，话题: /cleaning_robot/pointcloud")
        print("   3. 添加 Map 显示，话题: /map")
        print("   4. 添加 RobotModel 显示")
        print("   5. 设置 Fixed Frame 为: map 或 base_link")
        
        # 等待进程结束
        try:
            return_code = process.wait()
            if return_code != 0:
                print(f"❌ RViz2退出，返回码: {return_code}")
            else:
                print("✅ RViz2正常退出")
        except KeyboardInterrupt:
            print("\n🛑 收到中断信号，正在关闭RViz2...")
            process.terminate()
            process.wait()
            
    except FileNotFoundError:
        print("❌ 未找到RViz2，请确保已安装ROS2和RViz2")
        print("   安装命令: sudo apt install ros-humble-rviz2")
        return 1
    except Exception as e:
        print(f"❌ 启动RViz2时发生错误: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 