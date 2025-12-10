# Lesson 1: ROS2 小乌龟入门教程

## 课程概述

本课程将带你从零开始学习ROS2的基础概念，通过经典的小乌龟(turtlesim)程序来理解ROS2的核心机制：节点(Node)、话题(Topic)和消息(Message)。

## 学习目标

完成本课程后，你将能够：

- ✅ 成功运行ROS2小乌龟仿真程序
- ✅ 掌握键盘控制小乌龟的方法
- ✅ 理解ROS2话题通信机制
- ✅ 学会使用命令行工具发布和订阅消息
- ✅ 掌握基本的ROS2调试技巧

## 前置要求

- 已安装ROS2 Humble
- 熟悉Linux基本命令
- 了解终端操作

## 课程资源

- 📹 配套视频教程：`lesson1.mp4`
- 📝 实践练习和代码示例


## 环境配置

### Conda用户的ROS2环境隔离

我的默认工作环境是Conda，因此我需要对两个环境隔离。
所以我没有设置默认自启动ros2，而是需要手动启动环境

**设置ROS2环境别名：**

```bash
echo "alias ros2env='conda deactivate 2>/dev/null; source /opt/ros/humble/setup.bash && export PS1=\"(ros2) \$PS1\"'" >> ~/.bashrc
source ~/.bashrc
```

**使用方法：**
- 每次需要使用ROS2时，在终端输入 `ros2env`
- 你会看到提示符变为 `(ros2)` 表示已进入ROS2环境
- 这样可以确保ROS2和Conda环境不会相互干扰

### 验证安装

```bash
ros2env
ros2 --version
```

应该显示ROS2 Humble的版本信息。


## 实践步骤

### Step 1: 启动小乌龟仿真程序

**🖥️ 终端A - 启动仿真节点：**

```bash
ros2env
ros2 run turtlesim turtlesim_node
```

**预期结果：**
- 出现一个蓝色背景的窗口
- 窗口中央有一只小乌龟
- 终端显示节点启动信息

**🎮 终端B - 启动键盘控制节点：**

```bash
ros2env
ros2 run turtlesim turtle_teleop_key
```

**操作说明：**
- 确保终端B处于活跃状态（点击终端窗口）
- 使用方向键控制小乌龟移动
- 按 `Ctrl+C` 可以停止程序

**✅ 检查点 1：** 你能够用键盘方向键控制小乌龟在窗口中自由移动。

**💡 小贴士：**
- 如果乌龟不动，检查终端B是否处于焦点状态
- 乌龟移动时会留下轨迹，这是正常现象

### Step 2: 深入理解话题通信机制

话题(Topic)是ROS2中最重要的通信方式，让我们来探索小乌龟程序中的话题。

**🔍 查看系统中的所有话题：**

```bash
ros2env
ros2 topic list
```

你应该能看到：

```
/turtle1/cmd_vel
/turtle1/pose
...

```

看看 `/turtle1/cmd_vel` 的消息类型：

```
ros2 topic info /turtle1/cmd_vel

```

打印实时消息（乌龟位置）：

```
ros2 topic echo /turtle1/pose

```

✅ **检查点 2**：你能解释

- `/turtle1/cmd_vel` 是“控制速度的 Topic”
- `/turtle1/pose` 是“乌龟状态 Topic”

### Step 3: 手动发布消息 - 掌握发布者模式

现在我们来学习如何直接通过命令行发布消息，这将帮助你深入理解ROS2的发布-订阅机制。

**🚀 让乌龟直线前进：**

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

**🔄 让乌龟原地旋转：**

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 1.0}}"
```

**⬅️ 让乌龟后退：**

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: -1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

**🎯 组合运动 - 边走边转：**

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0}, angular: {z: 0.5}}"
```

**⏹️ 停止乌龟：**

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**✅ 检查点 3：** 你能够使用 `ros2 topic pub` 命令控制乌龟进行各种运动模式。

**📚 消息结构解析：**
- `linear.x`: 前后移动速度（正值前进，负值后退）
- `linear.y`: 左右移动速度（小乌龟不支持侧向移动）
- `angular.z`: 旋转速度（正值逆时针，负值顺时针）

## 课程总结

通过本课程，你已经掌握了：

1. **环境配置** - 设置ROS2工作环境
2. **节点运行** - 启动和管理ROS2节点
3. **话题通信** - 理解发布-订阅模式
4. **消息发布** - 使用命令行工具控制机器人

## 下一步学习

- 学习编写自己的发布者和订阅者节点
- 探索ROS2的服务(Service)和动作(Action)
- 了解参数系统和启动文件

## 常见问题

**Q: 乌龟不响应键盘控制怎么办？**
A: 确保键盘控制终端处于活跃状态，点击终端窗口使其获得焦点。

**Q: 如何清除乌龟的移动轨迹？**
A: 在仿真窗口中按 `Ctrl+R` 或重启 turtlesim_node。

**Q: 可以同时运行多个乌龟吗？**
A: 可以！使用服务调用可以生成更多乌龟，这将在后续课程中介绍。
