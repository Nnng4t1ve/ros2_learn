# Lesson 3: ROS2 订阅者节点与海龟状态监控

## 学习目标

在本课程中，你将学习：
- 如何创建ROS2订阅者节点
- 理解Pose消息类型
- 监控turtlesim海龟的实时状态
- 掌握回调函数的使用
- 学习消息频率控制技巧

## 前置条件

- 完成Lesson 1和Lesson 2的学习
- 已安装ROS2 Humble
- 熟悉基本的Python编程
- 理解ROS2发布者-订阅者模式

## 项目概述

本课程将扩展`turtle_controller`包，添加一个订阅者节点来监控海龟的位置和状态信息。我们将学习如何接收和处理来自turtlesim的实时数据。

## 项目结构

```
lesson3/
├── src/
│   └── turtle_controller/
│       ├── turtle_controller/
│       │   ├── __init__.py
│       │   ├── turtle_circle.py      # 发布者节点（Lesson 2）
│       │   └── turtle_pose_sub.py    # 订阅者节点（本课程重点）
│       ├── resource/
│       ├── test/
│       ├── package.xml               # 包描述文件
│       ├── setup.cfg                 # 配置文件
│       └── setup.py                  # 安装脚本
├── build/                            # 编译输出目录
├── install/                          # 安装目录
└── log/                             # 日志目录
```

## 核心概念

### 1. ROS2订阅者（Subscriber）

订阅者是ROS2中用于接收消息的组件：
- 订阅者监听特定话题的消息
- 当有新消息到达时，自动调用回调函数
- 可以有多个订阅者监听同一个话题

### 2. Pose消息类型

`turtlesim/msg/Pose`是海龟位置信息的消息类型：

```python
# Pose消息结构
float32 x              # x坐标 (0-11范围)
float32 y              # y坐标 (0-11范围)
float32 theta          # 朝向角度 (弧度制)
float32 linear_velocity    # 线性速度
float32 angular_velocity   # 角速度
```

### 3. 回调函数（Callback Function）

回调函数是订阅者的核心：
- 当收到新消息时自动被调用
- 接收消息对象作为参数
- 在函数内处理接收到的数据

### 4. 频率控制

在实际应用中，消息频率可能很高，需要控制输出频率：
- 避免日志输出过于频繁
- 提高程序性能
- 保持界面清洁

## 代码详解

### turtle_pose_sub.py 分析

#### 1. 导入必要的模块

```python
import rclpy                    # ROS2 Python客户端库
from rclpy.node import Node     # 节点基类
from turtlesim.msg import Pose  # 海龟位置消息类型
import time                     # 时间模块，用于频率控制
```

#### 2. 创建订阅者节点类

```python
class TurtlePoseSub(Node):
    def __init__(self):
        super().__init__('turtle_pose_sub')  # 设置节点名称
```

#### 3. 创建订阅者

```python
self.subscription = self.create_subscription(
    Pose,                    # 消息类型
    '/turtle1/pose',         # 话题名称
    self.callback,           # 回调函数
    10                       # 队列大小
)
```

#### 4. 频率控制变量

```python
self.last_print_time = 0.0      # 上次输出时间
self.print_interval = 0.5       # 输出间隔（秒）
self.message_count = 0          # 消息计数
self.skip_count = 0             # 跳过计数
```

#### 5. 回调函数实现

```python
def callback(self, msg: Pose):
    self.message_count += 1
    current_time = time.time()
    
    # 频率控制逻辑
    if current_time - self.last_print_time >= self.print_interval:
        # 输出海龟状态信息
        self.get_logger().info(
            f'位置: x={msg.x:.2f}, y={msg.y:.2f}\n'
            f'朝向: {msg.theta:.2f}弧度\n'
            f'速度: 线速度={msg.linear_velocity:.2f}'
        )
        self.last_print_time = current_time
```

## 运行步骤

### 1. 编译包

在lesson3目录下执行：

```bash
# 编译包
colcon build

# 设置环境变量
source install/setup.bash
```

**⚠️ 重要提醒：每次修改代码后都需要重新构建（colcon build）并更新环境变量（source install/setup.bash）！**

### 2. 启动turtlesim

在新终端中：

```bash
ros2env  # 如果使用conda环境隔离
ros2 run turtlesim turtlesim_node
```

### 3. 运行发布者节点（可选）

在另一个终端中：

```bash
source install/setup.bash
ros2 run turtle_controller turtle_circle
```

### 4. 运行订阅者节点

在新终端中：

```bash
source install/setup.bash
ros2 run turtle_controller turtle_pose_sub
```

## 预期结果

运行成功后，你应该看到：

1. **turtlesim窗口**：显示海龟图形界面
2. **发布者终端**：海龟开始圆形运动（如果运行了turtle_circle）
3. **订阅者终端**：每0.5秒输出一次海龟状态信息：

```
[INFO] [turtle_pose_sub]: === 海龟状态报告 === [总消息: 156, 跳过: 45]
位置: x=5.54, y=5.54
朝向: 0.78弧度 (45.0度)
速度: 线速度=1.00, 角速度=1.00
下次报告将在0.5秒后...
```

## 实验与探索

### 实验1：调整输出频率

修改`turtle_pose_sub.py`中的输出间隔：

```python
# 更频繁的输出
self.print_interval = 0.1  # 每0.1秒输出一次

# 更少的输出
self.print_interval = 2.0  # 每2秒输出一次
```

### 实验2：添加位置分析

扩展回调函数，添加位置分析功能：

```python
def callback(self, msg: Pose):
    # 检查海龟是否接近边界
    if msg.x < 1.0 or msg.x > 10.0 or msg.y < 1.0 or msg.y > 10.0:
        self.get_logger().warn('海龟接近边界！')
    
    # 检查海龟是否在中心区域
    center_x, center_y = 5.5, 5.5
    distance = ((msg.x - center_x)**2 + (msg.y - center_y)**2)**0.5
    if distance < 1.0:
        self.get_logger().info('海龟在中心区域')
```

### 实验3：数据记录

添加数据记录功能：

```python
def __init__(self):
    # ... 其他初始化代码 ...
    self.position_history = []  # 位置历史记录

def callback(self, msg: Pose):
    # 记录位置数据
    self.position_history.append({
        'time': time.time(),
        'x': msg.x,
        'y': msg.y,
        'theta': msg.theta
    })
    
    # 保持最近100个位置记录
    if len(self.position_history) > 100:
        self.position_history.pop(0)
```

## 多节点协作演示

### 同时运行发布者和订阅者

1. **终端1 - turtlesim**：
```bash
ros2 run turtlesim turtlesim_node
```

2. **终端2 - 圆形运动控制**：
```bash
source install/setup.bash
ros2 run turtle_controller turtle_circle
```

3. **终端3 - 位置监控**：
```bash
source install/setup.bash
ros2 run turtle_controller turtle_pose_sub
```

4. **终端4 - 键盘控制（可选）**：
```bash
ros2 run turtlesim turtle_teleop_key
```

这样可以同时观察：
- 海龟的圆形运动
- 实时位置数据输出
- 手动键盘控制的效果

## 故障排除

### 常见问题

1. **订阅者收不到消息**
   - 检查话题名称是否正确：`/turtle1/pose`
   - 确认turtlesim节点正在运行
   - 使用`ros2 topic list`查看可用话题

2. **输出频率过高**
   - 调整`print_interval`参数
   - 检查频率控制逻辑

3. **编译错误**
   - 检查setup.py中的entry_points配置
   - 确保所有依赖项已安装

### 调试命令

```bash
# 查看活动节点
ros2 node list

# 查看话题信息
ros2 topic info /turtle1/pose

# 监听话题消息（原始数据）
ros2 topic echo /turtle1/pose

# 查看消息发布频率
ros2 topic hz /turtle1/pose

# 查看节点图
rqt_graph
```

## 性能优化技巧

### 1. 消息频率控制

```python
# 方法1：时间间隔控制
if current_time - self.last_print_time >= self.print_interval:
    # 处理消息

# 方法2：计数器控制
self.counter += 1
if self.counter % 10 == 0:  # 每10条消息处理一次
    # 处理消息
```

### 2. 选择性数据处理

```python
def callback(self, msg: Pose):
    # 只在位置变化较大时处理
    if abs(msg.x - self.last_x) > 0.1 or abs(msg.y - self.last_y) > 0.1:
        # 处理位置变化
        self.last_x, self.last_y = msg.x, msg.y
```

## 扩展学习

### 下一步学习方向

1. **服务调用**: 学习请求-响应通信模式
2. **动作系统**: 了解长时间运行的任务处理
3. **参数服务器**: 动态配置节点参数
4. **tf变换**: 处理坐标系变换
5. **启动文件**: 自动化多节点系统启动

### 相关资源

- [ROS2订阅者教程](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [turtlesim消息类型](https://docs.ros2.org/latest/api/turtlesim/)
- [rclpy回调函数文档](https://docs.ros2.org/latest/api/rclpy/)

## 总结

本课程学习了：
- ✅ ROS2订阅者的创建和使用
- ✅ Pose消息类型的结构和应用
- ✅ 回调函数的实现方法
- ✅ 消息频率控制技巧
- ✅ 多节点协作的基本模式
- ✅ 性能优化的实用方法

通过订阅者节点，你已经掌握了ROS2通信的另一个重要方面。结合之前学习的发布者，你现在可以构建完整的发布-订阅通信系统。

## 作业

1. 修改订阅者，当海龟到达特定位置时发出警告
2. 创建一个节点，记录海龟的运动轨迹并保存到文件
3. 实现一个简单的"跟随者"节点，让第二只海龟跟随第一只海龟的轨迹

---