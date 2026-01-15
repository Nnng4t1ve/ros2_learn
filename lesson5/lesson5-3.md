# Lesson 5-3: SLAM 建图与地图保存

## 学习目标

- 理解 SLAM Toolbox（2D 激光 SLAM）
- 了解 RTAB-Map（RGBD/视觉 SLAM）
- 掌握 TF2 坐标体系：map → odom → base_link
- 使用 map_server 保存和加载地图
- 实践：TurtleBot3 自主建图并保存地图

## 前置条件

- 完成 Lesson 5-1 和 5-2
- TurtleBot3 仿真环境正常运行
- 已安装 SLAM 相关包

---

## 第一部分：SLAM 概述

### 1.1 什么是 SLAM

**SLAM**（Simultaneous Localization and Mapping）= 同时定位与建图

机器人在未知环境中：
- **定位**：确定自己在哪里
- **建图**：构建环境地图

这是一个"鸡生蛋"问题：
- 定位需要地图
- 建图需要知道位置

SLAM 算法同时解决这两个问题。

### 1.2 常见 SLAM 方案

| 方案 | 传感器 | 特点 | ROS 2 包 |
|------|--------|------|----------|
| SLAM Toolbox | 2D 激光雷达 | 轻量、稳定、适合室内 | slam_toolbox |
| Cartographer | 2D/3D 激光 | Google 开源、高精度 | cartographer_ros |
| RTAB-Map | RGBD/双目/激光 | 视觉 SLAM、支持 3D | rtabmap_ros |
| ORB-SLAM3 | 单目/双目/RGBD | 特征点视觉 SLAM | 需手动编译 |

---

## 第二部分：TF2 坐标体系

### 2.1 核心坐标系

```
map → odom → base_footprint → base_link → [sensors]
```

| 坐标系 | 说明 | 发布者 |
|--------|------|--------|
| `map` | 全局固定坐标系，地图原点 | SLAM / AMCL |
| `odom` | 里程计坐标系，会漂移 | 里程计节点 |
| `base_footprint` | 机器人投影到地面 | robot_state_publisher |
| `base_link` | 机器人底盘中心 | robot_state_publisher |
| `base_scan` | 激光雷达坐标系 | robot_state_publisher |

### 2.2 坐标系关系图

```
        ┌─────────────────────────────────────┐
        │              map                     │  ← 全局固定（SLAM 提供）
        │   (全局地图坐标系)                    │
        └──────────────┬──────────────────────┘
                       │ SLAM 修正漂移
                       ▼
        ┌─────────────────────────────────────┐
        │              odom                    │  ← 会累积漂移
        │   (里程计坐标系)                      │
        └──────────────┬──────────────────────┘
                       │ 里程计
                       ▼
        ┌─────────────────────────────────────┐
        │         base_footprint              │  ← 机器人地面投影
        └──────────────┬──────────────────────┘
                       │
                       ▼
        ┌─────────────────────────────────────┐
        │          base_link                  │  ← 机器人本体
        │    ┌──────┐  ┌──────┐               │
        │    │ LiDAR│  │Camera│               │
        │    └──────┘  └──────┘               │
        └─────────────────────────────────────┘
```

### 2.3 查看 TF 树

```bash
# 生成 TF 树 PDF
ros2 run tf2_tools view_frames

# 实时查看两个坐标系之间的变换
ros2 run tf2_ros tf2_echo map base_link

# 查看 TF 话题
ros2 topic echo /tf --once
```

### 2.4 map → odom 变换的意义

- **odom → base_link**：里程计提供，连续但会漂移
- **map → odom**：SLAM 提供，修正漂移，可能跳变

SLAM 通过调整 `map → odom` 变换来修正里程计漂移。

---

## 第三部分：SLAM Toolbox（2D 激光 SLAM）

### 3.1 安装

```bash
sudo apt install ros-humble-slam-toolbox
```

### 3.2 SLAM Toolbox 模式

| 模式 | 说明 | 使用场景 |
|------|------|----------|
| Online Async | 异步在线建图 | 实时建图（推荐） |
| Online Sync | 同步在线建图 | 需要精确时间同步 |
| Offline | 离线建图 | 从 rosbag 建图 |
| Localization | 纯定位模式 | 已有地图，只定位 |

### 3.3 启动 SLAM Toolbox

```bash
# 终端 1：启动仿真
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端 2：启动 SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# 或者使用 slam_toolbox（如果安装了）
# ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

### 3.4 使用键盘控制建图

```bash
# 终端 3：键盘遥控
ros2 run turtlebot3_teleop teleop_keyboard
```

控制机器人在环境中移动，地图会逐渐构建。

### 3.5 在 RViz2 中查看建图过程

```bash
# 终端 4：启动 RViz2
rviz2
```

添加显示：
1. **Fixed Frame** 设为 `map`
2. 添加 **Map** → 话题 `/map`
3. 添加 **LaserScan** → 话题 `/scan`
4. 添加 **TF** 查看坐标系
5. 添加 **RobotModel** 查看机器人

---

## 第四部分：RTAB-Map（视觉 SLAM，可选）

### 4.1 简介

**RTAB-Map**（Real-Time Appearance-Based Mapping）：
- 支持 RGBD 相机、双目相机、激光雷达
- 可构建 2D 和 3D 地图
- 支持回环检测
- 适合视觉 SLAM 场景

### 4.2 安装

```bash
sudo apt install ros-humble-rtabmap-ros
```

### 4.3 传感器要求

| 模式 | 需要的传感器 |
|------|-------------|
| RGBD | 深度相机（RGB + Depth） |
| Stereo | 双目相机 |
| RGB + LiDAR | 彩色相机 + 激光雷达 |

### 4.4 基本启动（需要深度相机）

```bash
# 如果有深度相机话题
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/camera/image_raw \
    depth_topic:=/camera/depth/image_raw \
    camera_info_topic:=/camera/camera_info \
    use_sim_time:=true
```

> 注：TurtleBot3 waffle 在 Humble 仿真中可能没有深度话题，RTAB-Map 可能无法直接使用。

---

## 第五部分：地图保存与加载

### 5.1 安装 map_server

```bash
sudo apt install ros-humble-nav2-map-server
```

### 5.2 保存地图

建图完成后，保存地图：

```bash
# 保存到当前目录，文件名为 my_map
ros2 run nav2_map_server map_saver_cli -f my_map

# 指定保存路径
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

生成两个文件：
- `my_map.pgm` - 地图图像（灰度图）
- `my_map.yaml` - 地图元数据

### 5.3 地图文件格式

**my_map.yaml 内容：**

```yaml
image: my_map.pgm          # 地图图像文件
mode: trinary              # 地图模式
resolution: 0.05           # 分辨率（米/像素）
origin: [-10.0, -10.0, 0]  # 地图原点 [x, y, yaw]
negate: 0                  # 是否反转颜色
occupied_thresh: 0.65      # 占用阈值
free_thresh: 0.25          # 空闲阈值
```

**地图图像像素含义：**

| 像素值 | 含义 |
|--------|------|
| 白色 (254) | 空闲区域 |
| 黑色 (0) | 障碍物 |
| 灰色 (205) | 未知区域 |

### 5.4 加载地图

```bash
# 启动 map_server 加载地图
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml -p use_sim_time:=true

# 需要激活生命周期节点
ros2 run nav2_util lifecycle_bringup map_server
```

或者使用 launch 文件：

```bash
ros2 launch nav2_bringup map_server.launch.py map:=/path/to/my_map.yaml use_sim_time:=true
```

---

## 第六部分：Demo 5-3 完整流程

### 步骤 1：启动仿真环境

```bash
# 终端 1
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 步骤 2：启动 SLAM（Cartographer）

```bash
# 终端 2
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 步骤 3：启动 RViz2 可视化

```bash
# 终端 3
rviz2
```

配置 RViz2：
- Fixed Frame: `map`
- 添加 Map（话题 `/map`）
- 添加 LaserScan（话题 `/scan`）
- 添加 TF

### 步骤 4：键盘控制建图

```bash
# 终端 4
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

使用 WASD 或方向键控制机器人遍历整个环境。

### 步骤 5：保存地图

建图完成后（地图覆盖整个环境）：

```bash
# 终端 5
cd ~/maps  # 或你想保存的目录
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/turtlebot3_world
```

### 步骤 6：验证地图

```bash
# 查看生成的文件
ls ~/maps/
# turtlebot3_world.pgm  turtlebot3_world.yaml

# 查看地图图像
eog ~/maps/turtlebot3_world.pgm
```


---

## 第七部分：自主探索建图（进阶）

### 7.1 自主探索概念

手动遥控建图效率低，可以使用自主探索算法：
- **Frontier Exploration**：探索未知区域边界
- **explore_lite**：ROS 1 常用，ROS 2 有移植版

### 7.2 简单的自主建图脚本思路

```python
# 伪代码：简单的随机探索
while not map_complete:
    if obstacle_ahead:
        rotate_random_angle()
    else:
        move_forward()
    
    if stuck_for_too_long:
        rotate_180_degrees()
```

### 7.3 使用 Nav2 进行探索

Nav2 提供了更完善的导航框架，可以结合 SLAM 实现：
1. 启动 SLAM
2. 启动 Nav2
3. 在 RViz2 中点击目标点
4. 机器人自动导航并建图

---

## 第八部分：常见问题

### Q1: 地图有很多噪点

**原因**：激光雷达数据噪声或里程计漂移

**解决**：
- 降低机器人移动速度
- 调整 SLAM 参数
- 多次经过同一区域让算法优化

### Q2: 地图不闭合（回环失败）

**原因**：回环检测失败

**解决**：
- 确保回到起点时有足够的特征
- 降低移动速度
- 使用支持回环检测的 SLAM（如 Cartographer）

### Q3: TF 报错 "Could not find transform"

**原因**：坐标系变换链断裂

**解决**：
```bash
# 检查 TF 树
ros2 run tf2_tools view_frames

# 确保所有节点都启动
ros2 node list
```

### Q4: 保存地图失败

**原因**：map_server 没有收到地图数据

**解决**：
```bash
# 确认 /map 话题有数据
ros2 topic echo /map --once

# 确认使用了 use_sim_time
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p use_sim_time:=true
```

---

## 第九部分：调试命令汇总

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 实时查看坐标变换
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# 查看地图话题
ros2 topic info /map
ros2 topic hz /map

# 查看 SLAM 节点
ros2 node list | grep -E "(slam|cartographer)"

# 查看节点参数
ros2 param list /cartographer_node
```

---

## 总结

本课程学习了：

- ✅ SLAM 原理：同时定位与建图
- ✅ SLAM Toolbox 和 Cartographer 的使用
- ✅ RTAB-Map 视觉 SLAM 概述
- ✅ TF2 坐标体系：map → odom → base_link
- ✅ 使用 map_server 保存和加载地图
- ✅ 完整的建图流程实践

---

## 下一步

- Lesson 5-4：Nav2 导航框架与路径规划
- 使用保存的地图进行自主导航
- 学习 AMCL 定位算法

---

## 参考资料

- [SLAM Toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [Cartographer ROS 2](https://google-cartographer-ros.readthedocs.io/)
- [RTAB-Map Wiki](http://wiki.ros.org/rtabmap_ros)
- [Nav2 Map Server](https://navigation.ros.org/configuration/packages/configuring-map-server.html)
- [TF2 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
