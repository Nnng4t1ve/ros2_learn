# Lesson 5-1: TurtleBot3 话题控制与路径点导航

## 学习目标

在本课程中，你将学习：
- 使用 `/cmd_vel` 话题控制 TurtleBot3 移动
- 订阅 `/odom` 里程计获取机器人位置
- 理解 TF 坐标变换 `/tf`
- 编写虚拟遥控器节点发布路径点导航指令

## 前置条件

- 完成 Lesson 4 TurtleBot3 仿真环境安装
- ROS 2 Humble 已安装
- 熟悉 ROS2 发布者/订阅者模式

## 核心话题介绍

### 1. /cmd_vel - 速度控制话题

`/cmd_vel` 是控制机器人运动的核心话题，使用 `geometry_msgs/msg/Twist` 消息类型：

```python
# Twist 消息结构
geometry_msgs/Vector3 linear   # 线性速度
  float64 x  # 前进/后退 (m/s)
  float64 y  # 左右平移 (差速机器人通常为0)
  float64 z  # 上下 (地面机器人为0)

geometry_msgs/Vector3 angular  # 角速度
  float64 x  # 横滚 (通常为0)
  float64 y  # 俯仰 (通常为0)
  float64 z  # 偏航/转向 (rad/s)
```

### 2. /odom - 里程计话题

`/odom` 提供机器人的位置和速度估计，使用 `nav_msgs/msg/Odometry` 消息类型：

```python
# Odometry 消息结构（简化）
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id          # 通常是 "odom"

string child_frame_id      # 通常是 "base_footprint"

geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position      # x, y, z 位置
    geometry_msgs/Quaternion orientation  # 四元数姿态

geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist  # 当前速度
```

### 3. /tf - 坐标变换话题

TF 系统管理多个坐标系之间的变换关系：

```
map → odom → base_footprint → base_link → [sensors]
```

- `map`: 全局固定坐标系
- `odom`: 里程计坐标系（会漂移）
- `base_footprint`: 机器人底盘投影到地面
- `base_link`: 机器人底盘中心

## 项目结构

```
lesson5/
├── src/
│   └── turtlebot3_controller/
│       ├── turtlebot3_controller/
│       │   ├── __init__.py
│       │   ├── simple_mover.py       # 简单移动控制
│       │   ├── odom_subscriber.py    # 里程计订阅
│       │   ├── waypoint_nav.py       # 路径点导航
│       │   └── virtual_teleop.py     # 虚拟遥控器
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
└── lesson5.md
```


## 第一部分：创建功能包

### 1. 创建工作空间和功能包

```bash
# 进入 lesson5 目录
cd lesson5

# 创建 src 目录
mkdir -p src
cd src

# 创建功能包
ros2 pkg create --build-type ament_python turtlebot3_controller \
  --dependencies rclpy geometry_msgs nav_msgs tf2_ros

cd ..
```

### 2. 配置 package.xml

确保 `package.xml` 包含以下依赖：

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```

## 第二部分：简单移动控制节点

### simple_mover.py - 基础速度控制

```python
#!/usr/bin/env python3
"""
简单移动控制节点 - 控制 TurtleBot3 前进和旋转
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        
        # 创建速度发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 运动参数
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.5  # rad/s
        
        # 定时器 - 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 运动状态
        self.state = 'forward'  # forward, turn, stop
        self.state_start_time = self.get_clock().now()
        
        self.get_logger().info('SimpleMover 节点已启动')
        self.get_logger().info('机器人将前进2秒，然后旋转2秒，循环执行')

    def timer_callback(self):
        msg = Twist()
        current_time = self.get_clock().now()
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
        
        if self.state == 'forward':
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0
            if elapsed > 2.0:  # 前进2秒
                self.state = 'turn'
                self.state_start_time = current_time
                self.get_logger().info('切换到旋转状态')
                
        elif self.state == 'turn':
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed
            if elapsed > 2.0:  # 旋转2秒
                self.state = 'forward'
                self.state_start_time = current_time
                self.get_logger().info('切换到前进状态')
        
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 停止机器人
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.get_logger().info('节点已停止，机器人已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```


## 第三部分：里程计订阅节点

### odom_subscriber.py - 监控机器人位置

```python
#!/usr/bin/env python3
"""
里程计订阅节点 - 监控 TurtleBot3 位置和速度
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        
        # 创建里程计订阅者
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 输出频率控制
        self.last_print_time = 0.0
        self.print_interval = 1.0  # 每秒输出一次
        
        self.get_logger().info('OdomSubscriber 节点已启动')
        self.get_logger().info('正在监听 /odom 话题...')

    def quaternion_to_yaw(self, q):
        """四元数转欧拉角（只取 yaw）"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if current_time - self.last_print_time >= self.print_interval:
            # 提取位置
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            # 提取姿态（转换为角度）
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            yaw_deg = math.degrees(yaw)
            
            # 提取速度
            linear_vel = msg.twist.twist.linear.x
            angular_vel = msg.twist.twist.angular.z
            
            self.get_logger().info(
                f'\n=== 里程计数据 ===\n'
                f'位置: x={x:.3f}m, y={y:.3f}m\n'
                f'朝向: {yaw_deg:.1f}° ({yaw:.3f} rad)\n'
                f'速度: 线速度={linear_vel:.3f}m/s, 角速度={angular_vel:.3f}rad/s'
            )
            
            self.last_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 第四部分：路径点导航节点

### waypoint_nav.py - 自动导航到指定路径点

```python
#!/usr/bin/env python3
"""
路径点导航节点 - 控制 TurtleBot3 依次到达指定路径点
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # 发布者和订阅者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # 定义路径点列表 [(x, y), ...]
        self.waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0),
        ]
        self.current_waypoint_idx = 0
        
        # 当前位置和姿态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # 控制参数
        self.distance_tolerance = 0.1  # 到达判定距离 (m)
        self.angle_tolerance = 0.1     # 角度容差 (rad)
        self.linear_speed = 0.15       # 线速度 (m/s)
        self.angular_speed = 0.5       # 角速度 (rad/s)
        
        # 控制定时器 - 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('WaypointNavigator 节点已启动')
        self.get_logger().info(f'路径点: {self.waypoints}')

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def get_distance_to_goal(self, goal_x, goal_y):
        return math.sqrt((goal_x - self.current_x)**2 + 
                        (goal_y - self.current_y)**2)

    def get_angle_to_goal(self, goal_x, goal_y):
        return math.atan2(goal_y - self.current_y, 
                         goal_x - self.current_x)

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            # 所有路径点已完成
            self.stop_robot()
            self.get_logger().info('所有路径点导航完成！')
            self.timer.cancel()
            return
        
        # 获取当前目标点
        goal_x, goal_y = self.waypoints[self.current_waypoint_idx]
        
        # 计算距离和角度
        distance = self.get_distance_to_goal(goal_x, goal_y)
        angle_to_goal = self.get_angle_to_goal(goal_x, goal_y)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)
        
        msg = Twist()
        
        # 判断是否到达目标点
        if distance < self.distance_tolerance:
            self.get_logger().info(
                f'到达路径点 {self.current_waypoint_idx + 1}: '
                f'({goal_x}, {goal_y})'
            )
            self.current_waypoint_idx += 1
            return
        
        # 先转向，再前进
        if abs(angle_diff) > self.angle_tolerance:
            # 需要转向
            msg.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        else:
            # 朝向正确，前进
            msg.linear.x = self.linear_speed
            # 轻微角度修正
            msg.angular.z = 0.3 * angle_diff
        
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().info('导航已取消')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```


## 第五部分：虚拟遥控器节点

### virtual_teleop.py - 交互式路径点发布

```python
#!/usr/bin/env python3
"""
虚拟遥控器节点 - 通过命令行输入路径点进行导航
支持功能：
1. 输入目标坐标进行导航
2. 预设路径点快捷键
3. 实时显示机器人状态
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import threading
import sys
import select


class VirtualTeleop(Node):
    def __init__(self):
        super().__init__('virtual_teleop')
        
        # 发布者和订阅者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # 当前状态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # 目标点
        self.goal_x = None
        self.goal_y = None
        self.navigating = False
        
        # 控制参数
        self.distance_tolerance = 0.1
        self.angle_tolerance = 0.1
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        
        # 控制定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 启动输入线程
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.print_help()

    def print_help(self):
        help_text = """
╔════════════════════════════════════════════════╗
║         TurtleBot3 虚拟遥控器                  ║
╠════════════════════════════════════════════════╣
║  命令:                                         ║
║    g x y  - 导航到坐标 (x, y)                  ║
║    1      - 预设点1: (1.0, 0.0)                ║
║    2      - 预设点2: (1.0, 1.0)                ║
║    3      - 预设点3: (0.0, 1.0)                ║
║    4      - 预设点4: (0.0, 0.0) 原点           ║
║    s      - 停止机器人                         ║
║    p      - 显示当前位置                       ║
║    h      - 显示帮助                           ║
║    q      - 退出                               ║
╚════════════════════════════════════════════════╝
"""
        print(help_text)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def set_goal(self, x, y):
        self.goal_x = x
        self.goal_y = y
        self.navigating = True
        print(f'\n>>> 开始导航到 ({x:.2f}, {y:.2f})')

    def stop_robot(self):
        self.navigating = False
        self.goal_x = None
        self.goal_y = None
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        print('\n>>> 机器人已停止')

    def print_position(self):
        yaw_deg = math.degrees(self.current_yaw)
        print(f'\n>>> 当前位置: x={self.current_x:.3f}, y={self.current_y:.3f}, '
              f'yaw={yaw_deg:.1f}°')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if not self.navigating or self.goal_x is None:
            return
        
        # 计算距离和角度
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)
        
        msg = Twist()
        
        # 到达目标
        if distance < self.distance_tolerance:
            print(f'\n>>> 已到达目标点 ({self.goal_x:.2f}, {self.goal_y:.2f})')
            self.stop_robot()
            return
        
        # 控制逻辑
        if abs(angle_diff) > self.angle_tolerance:
            msg.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        else:
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.3 * angle_diff
        
        self.cmd_vel_pub.publish(msg)

    def input_loop(self):
        """处理用户输入"""
        preset_points = {
            '1': (1.0, 0.0),
            '2': (1.0, 1.0),
            '3': (0.0, 1.0),
            '4': (0.0, 0.0),
        }
        
        while rclpy.ok():
            try:
                cmd = input('\n输入命令: ').strip().lower()
                
                if cmd == 'q':
                    self.stop_robot()
                    print('退出程序...')
                    rclpy.shutdown()
                    break
                elif cmd == 'h':
                    self.print_help()
                elif cmd == 's':
                    self.stop_robot()
                elif cmd == 'p':
                    self.print_position()
                elif cmd in preset_points:
                    x, y = preset_points[cmd]
                    self.set_goal(x, y)
                elif cmd.startswith('g '):
                    try:
                        parts = cmd.split()
                        x = float(parts[1])
                        y = float(parts[2])
                        self.set_goal(x, y)
                    except (IndexError, ValueError):
                        print('格式错误！使用: g x y (例如: g 1.5 2.0)')
                else:
                    print('未知命令，输入 h 查看帮助')
                    
            except EOFError:
                break


def main(args=None):
    rclpy.init(args=args)
    node = VirtualTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```


## 第六部分：配置文件

### setup.py

```python
from setuptools import find_packages, setup

package_name = 'turtlebot3_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TurtleBot3 controller with waypoint navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_mover = turtlebot3_controller.simple_mover:main',
            'odom_subscriber = turtlebot3_controller.odom_subscriber:main',
            'waypoint_nav = turtlebot3_controller.waypoint_nav:main',
            'virtual_teleop = turtlebot3_controller.virtual_teleop:main',
        ],
    },
)
```

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>turtlebot3_controller</name>
  <version>0.0.1</version>
  <description>TurtleBot3 controller with waypoint navigation</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## 运行步骤

### 1. 编译功能包

```bash
cd lesson5
colcon build
source install/setup.bash
```

### 2. 启动仿真环境

终端1 - 启动 Gazebo 空白世界：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 3. 运行节点

#### 方式1：简单移动测试

```bash
# 终端2
source install/setup.bash
ros2 run turtlebot3_controller simple_mover
```

#### 方式2：里程计监控

```bash
# 终端2
source install/setup.bash
ros2 run turtlebot3_controller odom_subscriber
```

#### 方式3：路径点自动导航

```bash
# 终端2
source install/setup.bash
ros2 run turtlebot3_controller waypoint_nav
```

#### 方式4：虚拟遥控器（推荐）

```bash
# 终端2
source install/setup.bash
ros2 run turtlebot3_controller virtual_teleop
```

## 调试命令

```bash
# 查看话题列表
ros2 topic list

# 查看 /cmd_vel 消息
ros2 topic echo /cmd_vel

# 查看 /odom 消息
ros2 topic echo /odom

# 查看 TF 树
ros2 run tf2_tools view_frames

# 实时查看 TF
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 常见问题

### Q1: 机器人不动

- 检查 Gazebo 是否正常运行
- 确认 `/cmd_vel` 话题是否正确发布
- 使用 `ros2 topic echo /cmd_vel` 验证

### Q2: 里程计数据不更新

- 确认 `/odom` 话题存在
- 检查仿真时间是否正常流动

### Q3: 导航精度不高

- 调整 `distance_tolerance` 和 `angle_tolerance`
- 降低速度以提高精度

## 总结

本课程学习了：
- ✅ `/cmd_vel` 话题控制机器人移动
- ✅ `/odom` 里程计数据订阅和解析
- ✅ 四元数到欧拉角的转换
- ✅ 路径点导航算法实现
- ✅ 虚拟遥控器交互式控制

下一课将学习 SLAM 建图和自主导航。
