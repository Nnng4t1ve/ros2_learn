# ROS2 学习项目

这是一个系统性的ROS2学习项目，记录了从基础到进阶的完整学习过程。通过实践和理论相结合的方式，帮助初学者掌握ROS2的核心概念和开发技能。

## 📚 项目简介

本项目采用循序渐进的教学方式，每个课程都包含：
- 详细的理论讲解
- 完整的代码示例
- 实践练习和实验
- 故障排除指南
- 扩展学习建议

## 🎯 学习路径

### Lesson 1: ROS2基础入门
- **主题**: 小乌龟程序入门
- **内容**: ROS2基本概念、节点运行、话题通信
- **技能**: 掌握ROS2基础操作和turtlesim使用
- **文档**: [lesson1/lesson1.md](lesson1/lesson1.md)

### Lesson 2: 发布者节点与海龟控制
- **主题**: 创建ROS2发布者节点
- **内容**: 发布者模式、Twist消息、定时器使用
- **技能**: 编写控制海龟运动的节点程序
- **文档**: [lesson2/lesson2.md](lesson2/lesson2.md)
- **代码**: [turtle_circle.py](lesson2/src/turtle_controller/turtle_controller/turtle_circle.py)

### Lesson 3: 订阅者节点与海龟状态监控
- **主题**: 创建ROS2订阅者节点
- **内容**: 订阅者模式、Pose消息、回调函数、频率控制
- **技能**: 监控海龟实时状态，掌握消息接收处理
- **文档**: [lesson3/lesson3.md](lesson3/lesson3.md)
- **代码**: [turtle_pose_sub.py](lesson3/src/turtle_controller/turtle_controller/turtle_pose_sub.py)


## 🛠️ 环境要求

### 系统要求
- **操作系统**: Ubuntu 22.04 LTS (推荐)
- **ROS2版本**: Humble Hawksbill

### 依赖安装
```bash
# 安装ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装开发工具
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-pip

# 初始化rosdep
sudo rosdep init
rosdep update
```

## 🚀 快速开始

### 1. 克隆项目
```bash
git clone https://github.com/SFMistral/ros2_learn.git
cd ros2_learn
```

### 2. 设置环境
```bash
# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 添加到bashrc（可选）
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. 运行示例

#### Lesson 1 - 基础操作
```bash
# 启动turtlesim
ros2 run turtlesim turtlesim_node

# 键盘控制（新终端）
ros2 run turtlesim turtle_teleop_key
```

#### Lesson 2 - 海龟圆形运动
```bash
# 进入lesson2目录
cd lesson2

# 编译项目
colcon build
source install/setup.bash

# 启动turtlesim（终端1）
ros2 run turtlesim turtlesim_node

# 运行圆形控制节点（终端2）
ros2 run turtle_controller turtle_circle
```

#### Lesson 3 - 海龟状态监控
```bash
# 进入lesson3目录
cd lesson3

# 编译项目
colcon build
source install/setup.bash

# 启动turtlesim（终端1）
ros2 run turtlesim turtlesim_node

# 运行圆形控制节点（终端2）
ros2 run turtle_controller turtle_circle

# 运行状态监控节点（终端3）
ros2 run turtle_controller turtle_pose_sub
```

## 📖 学习建议

### 学习顺序
1. 按照课程编号顺序学习
2. 完成每课的实践练习
3. 尝试扩展实验
4. 解决故障排除中的问题

### 实践技巧
- 多使用ROS2命令行工具进行调试
- 学会查看话题、节点和服务信息
- 使用rqt工具可视化系统状态
- 阅读官方文档加深理解

## 🔧 常用命令

```bash
# 节点管理
ros2 node list                    # 查看活动节点
ros2 node info <node_name>        # 查看节点信息

# 话题管理
ros2 topic list                   # 查看话题列表
ros2 topic echo <topic_name>      # 监听话题消息
ros2 topic info <topic_name>      # 查看话题信息

# 包管理
ros2 pkg list                     # 查看已安装包
ros2 pkg executables <pkg_name>   # 查看包的可执行文件

# 编译和运行
colcon build                      # 编译工作空间
source install/setup.bash        # 设置环境变量
```

## 📁 项目结构

```
ros2_learn/
├── README.md                     # 项目说明文档
├── .gitignore                    # Git忽略配置
├── lesson1/
│   └── lesson1.md               # 第一课教程
├── lesson2/
│   ├── lesson2.md               # 第二课教程
│   ├── src/
│   │   └── turtle_controller/   # ROS2包（发布者）
│   ├── build/                   # 编译输出（忽略）
│   ├── install/                 # 安装目录（忽略）
│   └── log/                     # 日志目录（忽略）
├── lesson3/
│   ├── lesson3.md               # 第三课教程
│   ├── src/
│   │   └── turtle_controller/   # ROS2包（订阅者）
│   ├── build/                   # 编译输出（忽略）
│   ├── install/                 # 安装目录（忽略）
│   └── log/                     # 日志目录（忽略）
└── ...                          # 更多课程
```