# Lesson 5-4: Nav2 导航框架与自主导航

## 学习目标

- 理解 Nav2 架构：BT（行为树）、Planner、Controller、Recovery
- 掌握 AMCL 定位原理与配置
- 理解 Costmap 参数配置
- 使用 Action 发送导航目标
- 实践：机器人在已建地图内导航到任意目标点并避障

## 前置条件

- 完成 Lesson 5-3，已保存地图文件
- 已安装 Nav2 导航包
- TurtleBot3 仿真环境正常

---

## 第一部分：Nav2 架构概述

### 1.1 什么是 Nav2

**Nav2**（Navigation2）是 ROS 2 的官方导航框架，提供：
- 路径规划（Planning）
- 路径跟踪（Control）
- 定位（Localization）
- 恢复行为（Recovery）
- 行为树协调（Behavior Tree）

### 1.2 Nav2 核心组件

```
┌─────────────────────────────────────────────────────────────┐
│                      BT Navigator                           │
│                    （行为树导航器）                           │
│         协调 Planner、Controller、Recovery                   │
└─────────────────────────────────────────────────────────────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌───────────────┐  ┌─────────────────┐  ┌─────────────────┐
│   Planner     │  │   Controller    │  │    Recovery     │
│  （路径规划）  │  │  （路径跟踪）    │  │  （恢复行为）    │
│  NavFn/Smac   │  │  DWB/TEB/MPPI   │  │  Spin/BackUp    │
└───────────────┘  └─────────────────┘  └─────────────────┘
        │                    │
        ▼                    ▼
┌─────────────────────────────────────────────────────────────┐
│                      Costmap 2D                             │
│              Global Costmap    Local Costmap                │
└─────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────┐
│                        AMCL                                 │
│                    （自适应蒙特卡洛定位）                      │
└─────────────────────────────────────────────────────────────┘
```

### 1.3 组件功能说明

| 组件 | 功能 | 常用插件 |
|------|------|----------|
| BT Navigator | 行为树协调导航流程 | NavigateToPose |
| Planner Server | 全局路径规划 | NavFn, Smac, Theta* |
| Controller Server | 局部路径跟踪 | DWB, TEB, MPPI |
| Recovery Server | 恢复行为 | Spin, BackUp, Wait |
| Costmap 2D | 代价地图 | Static, Inflation, Obstacle |
| AMCL | 粒子滤波定位 | - |

---

## 第二部分：行为树（Behavior Tree）

### 2.1 什么是行为树

行为树是一种用于控制决策流程的树状结构：
- **比状态机更灵活**：易于扩展和修改
- **模块化**：每个节点独立，可复用
- **可视化**：结构清晰，便于调试

### 2.2 行为树节点类型

| 节点类型 | 符号 | 说明 |
|----------|------|------|
| Sequence | → | 顺序执行，全部成功才成功 |
| Fallback | ? | 选择执行，一个成功就成功 |
| Action | ○ | 执行具体动作 |
| Condition | ◇ | 检查条件 |
| Decorator | ◆ | 修饰子节点行为 |

### 2.3 Nav2 默认导航行为树

```
Root
└── PipelineSequence
    ├── RateController (1Hz)
    │   └── ComputePathToPose (Planner)
    └── FollowPath (Controller)
        └── RecoveryNode
            ├── FollowPath
            └── Recovery Fallback
                ├── ClearCostmap
                ├── Spin
                └── BackUp
```

### 2.4 行为树文件位置

```bash
# Nav2 默认行为树
/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/

# 常用行为树
navigate_to_pose_w_replanning_and_recovery.xml
navigate_through_poses_w_replanning_and_recovery.xml
```


---

## 第三部分：Planner（路径规划器）

### 3.1 Planner 作用

Planner 负责计算从当前位置到目标点的全局路径。

### 3.2 常用 Planner 插件

| 插件 | 算法 | 特点 |
|------|------|------|
| NavFn | Dijkstra/A* | 经典算法，稳定可靠 |
| Smac Planner 2D | A* 变体 | 支持更多启发式 |
| Smac Hybrid-A* | Hybrid A* | 考虑车辆运动学 |
| Smac Lattice | State Lattice | 适合非完整约束 |
| Theta* | Theta* | 任意角度路径 |

### 3.3 Planner 配置示例

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5                    # 目标容差
      use_astar: false                  # 使用 Dijkstra
      allow_unknown: true               # 允许穿越未知区域
```

---

## 第四部分：Controller（路径跟踪器）

### 4.1 Controller 作用

Controller 负责跟踪全局路径，生成速度指令，实现局部避障。

### 4.2 常用 Controller 插件

| 插件 | 全称 | 特点 |
|------|------|------|
| DWB | Dynamic Window Approach | 经典方法，参数多 |
| TEB | Timed Elastic Band | 时间最优，适合动态环境 |
| MPPI | Model Predictive Path Integral | 采样优化，效果好 |
| RPP | Regulated Pure Pursuit | 简单高效，适合直线 |

### 4.3 DWB Controller 配置示例

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.26                   # 最大线速度
      max_vel_theta: 1.0                # 最大角速度
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      acc_lim_x: 2.5                    # 线加速度限制
      acc_lim_theta: 3.2                # 角加速度限制
      decel_lim_x: -2.5
      decel_lim_theta: -3.2
      
      # 轨迹评分
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

---

## 第五部分：Recovery（恢复行为）

### 5.1 Recovery 作用

当机器人卡住或导航失败时，执行恢复动作。

### 5.2 常用 Recovery 插件

| 插件 | 动作 | 使用场景 |
|------|------|----------|
| Spin | 原地旋转 | 清除周围障碍物感知 |
| BackUp | 后退 | 离开死角 |
| Wait | 等待 | 等待动态障碍物移开 |
| ClearCostmap | 清除代价地图 | 重置错误的障碍物信息 |

### 5.3 Recovery 配置示例

```yaml
recoveries_server:
  ros__parameters:
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
```


---

## 第六部分：AMCL 定位

### 6.1 什么是 AMCL

**AMCL**（Adaptive Monte Carlo Localization）= 自适应蒙特卡洛定位

- 基于粒子滤波的定位算法
- 在已知地图中估计机器人位置
- 使用激光雷达数据与地图匹配

### 6.2 AMCL 工作原理

```
1. 初始化：在地图上撒布大量粒子（位置假设）
2. 预测：根据里程计移动所有粒子
3. 更新：用激光数据评估每个粒子的权重
4. 重采样：保留高权重粒子，删除低权重粒子
5. 收敛：粒子逐渐聚集到真实位置
```

### 6.3 AMCL 关键参数

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    
    # 粒子数量
    min_particles: 500              # 最小粒子数
    max_particles: 2000             # 最大粒子数
    
    # 运动模型（差速驱动）
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    alpha1: 0.2                     # 旋转噪声（旋转）
    alpha2: 0.2                     # 旋转噪声（平移）
    alpha3: 0.2                     # 平移噪声（平移）
    alpha4: 0.2                     # 平移噪声（旋转）
    alpha5: 0.2                     # 平移噪声（平移）
    
    # 激光模型
    laser_model_type: "likelihood_field"
    laser_max_range: 12.0           # 激光最大范围
    laser_min_range: 0.1            # 激光最小范围
    max_beams: 60                   # 使用的激光束数量
    z_hit: 0.5                      # 命中权重
    z_rand: 0.5                     # 随机权重
    sigma_hit: 0.2                  # 命中标准差
    
    # 更新阈值
    update_min_d: 0.25              # 最小平移距离触发更新
    update_min_a: 0.2               # 最小旋转角度触发更新
    
    # 坐标系
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    global_frame_id: "map"
    
    # 初始位姿
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0
```

### 6.4 AMCL 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/amcl_pose` | PoseWithCovarianceStamped | 估计位姿 |
| `/particlecloud` | PoseArray | 粒子云（可视化） |
| `/initialpose` | PoseWithCovarianceStamped | 设置初始位姿 |

### 6.5 在 RViz2 中设置初始位姿

1. 点击工具栏 "2D Pose Estimate"
2. 在地图上点击机器人实际位置
3. 拖动箭头指示朝向
4. 粒子会收敛到该位置

---

## 第七部分：Costmap 代价地图

### 7.1 什么是 Costmap

Costmap 将环境表示为代价值网格：
- **0**：自由空间
- **1-252**：代价递增
- **253**：内切障碍物
- **254**：致命障碍物
- **255**：未知

### 7.2 Global vs Local Costmap

| 类型 | 范围 | 用途 | 更新频率 |
|------|------|------|----------|
| Global Costmap | 整个地图 | 全局路径规划 | 低 |
| Local Costmap | 机器人周围 | 局部避障 | 高 |

### 7.3 Costmap 层（Layers）

```
┌─────────────────────────────────────┐
│         Master Costmap              │  ← 最终合成
├─────────────────────────────────────┤
│       Inflation Layer               │  ← 障碍物膨胀
├─────────────────────────────────────┤
│       Obstacle Layer                │  ← 实时传感器障碍物
├─────────────────────────────────────┤
│       Static Layer                  │  ← 静态地图
└─────────────────────────────────────┘
```

### 7.4 Global Costmap 配置

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0           # 更新频率
      publish_frequency: 1.0          # 发布频率
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22              # 机器人半径
      resolution: 0.05                # 分辨率
      track_unknown_space: true
      
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0      # 代价衰减因子
        inflation_radius: 0.55        # 膨胀半径
```

### 7.5 Local Costmap 配置

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true            # 滚动窗口
      width: 3                        # 宽度 (m)
      height: 3                       # 高度 (m)
      resolution: 0.05
      robot_radius: 0.22
      
      plugins: ["obstacle_layer", "inflation_layer"]
      
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```


---

## 第八部分：使用 Action 发送导航目标

### 8.1 Nav2 Action 接口

Nav2 使用 ROS 2 Action 接收导航目标：

| Action | 类型 | 说明 |
|--------|------|------|
| `/navigate_to_pose` | NavigateToPose | 导航到单个目标点 |
| `/navigate_through_poses` | NavigateThroughPoses | 导航经过多个路径点 |
| `/follow_waypoints` | FollowWaypoints | 跟随路径点序列 |

### 8.2 NavigateToPose Action 结构

```bash
# 查看 Action 定义
ros2 interface show nav2_msgs/action/NavigateToPose
```

```yaml
# Goal（目标）
geometry_msgs/PoseStamped pose
  std_msgs/Header header
  geometry_msgs/Pose pose
string behavior_tree              # 可选：指定行为树

---
# Result（结果）
std_msgs/Empty result

---
# Feedback（反馈）
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
```

### 8.3 命令行发送导航目标

```bash
# 发送导航目标到 (1.0, 1.0)，朝向 0 度
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 8.4 Python 导航客户端

```python
#!/usr/bin/env python3
"""
Nav2 导航客户端 - 发送导航目标
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        
        # 创建 Action 客户端
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.get_logger().info('Nav2 客户端已启动')

    def send_goal(self, x, y, yaw=0.0):
        """发送导航目标"""
        # 等待 Action 服务器
        self.get_logger().info('等待 Nav2 服务器...')
        self._action_client.wait_for_server()
        
        # 创建目标消息
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 设置位置
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # 设置朝向（yaw 转四元数）
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.get_logger().info(f'发送导航目标: ({x}, {y}), yaw={math.degrees(yaw):.1f}°')
        
        # 发送目标
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """目标响应回调"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return
        
        self.get_logger().info('目标已接受，开始导航...')
        
        # 获取结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose.position
        distance = feedback.distance_remaining
        
        self.get_logger().info(
            f'当前位置: ({current_pose.x:.2f}, {current_pose.y:.2f}), '
            f'剩余距离: {distance:.2f}m'
        )

    def result_callback(self, future):
        """结果回调"""
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('导航成功完成！')
        else:
            self.get_logger().warn(f'导航结束，状态码: {result.status}')


def main(args=None):
    rclpy.init(args=args)
    
    client = Nav2Client()
    
    # 发送导航目标
    client.send_goal(x=2.0, y=1.0, yaw=0.0)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 8.5 使用 RViz2 发送导航目标

1. 点击工具栏 "2D Goal Pose"
2. 在地图上点击目标位置
3. 拖动箭头指示目标朝向
4. 机器人开始自主导航


---

## 第九部分：Demo 5-4 完整导航流程

### 步骤 1：启动仿真环境

```bash
# 终端 1
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 步骤 2：启动 Nav2 导航（使用已保存的地图）

```bash
# 终端 2
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# 使用 Lesson 5-3 保存的地图
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/maps/turtlebot3_world.yaml
```

### 步骤 3：启动 RViz2

```bash
# 终端 3
rviz2
```

RViz2 配置：
1. Fixed Frame: `map`
2. 添加 Map（话题 `/map`）
3. 添加 Map（话题 `/global_costmap/costmap`）
4. 添加 Map（话题 `/local_costmap/costmap`）
5. 添加 Path（话题 `/plan`）
6. 添加 LaserScan（话题 `/scan`）
7. 添加 PoseArray（话题 `/particlecloud`）- 查看 AMCL 粒子
8. 添加 RobotModel

### 步骤 4：设置初始位姿

在 RViz2 中：
1. 点击 "2D Pose Estimate"
2. 在地图上点击机器人当前位置
3. 拖动设置朝向
4. 观察粒子云收敛

### 步骤 5：发送导航目标

方式 1 - RViz2：
1. 点击 "2D Goal Pose"
2. 在地图上点击目标位置
3. 拖动设置目标朝向

方式 2 - 命令行：
```bash
# 终端 4
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 步骤 6：观察导航过程

- 全局路径（绿色线）
- 局部代价地图（机器人周围）
- 机器人跟踪路径并避障
- 到达目标后停止

---

## 第十部分：导航参数调优

### 10.1 速度参数

```yaml
# 根据机器人能力调整
max_vel_x: 0.26        # TurtleBot3 Burger 最大速度
max_vel_theta: 1.82    # 最大角速度
```

### 10.2 Costmap 膨胀参数

```yaml
inflation_layer:
  inflation_radius: 0.55    # 膨胀半径 > 机器人半径
  cost_scaling_factor: 3.0  # 越大，代价衰减越快
```

- `inflation_radius` 太小：可能碰撞
- `inflation_radius` 太大：通道变窄，无法通过

### 10.3 Planner 参数

```yaml
GridBased:
  tolerance: 0.5            # 目标容差
  use_astar: true           # A* 比 Dijkstra 快
  allow_unknown: false      # 不允许穿越未知区域
```

### 10.4 Controller 参数

```yaml
FollowPath:
  xy_goal_tolerance: 0.25   # 位置容差
  yaw_goal_tolerance: 0.25  # 角度容差
```

---

## 第十一部分：常见问题

### Q1: 机器人不动

**检查项**：
```bash
# 检查 Nav2 节点是否启动
ros2 node list | grep nav2

# 检查 Action 服务器
ros2 action list

# 检查 TF
ros2 run tf2_ros tf2_echo map base_link
```

### Q2: 定位不准确

**解决**：
- 重新设置初始位姿
- 增加 AMCL 粒子数
- 检查激光雷达数据

### Q3: 路径规划失败

**原因**：目标点在障碍物内或不可达

**解决**：
- 检查 Costmap 是否正确
- 增加 `tolerance` 参数
- 清除 Costmap：
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

### Q4: 机器人卡住

**解决**：
- 等待 Recovery 行为执行
- 手动清除 Costmap
- 检查是否有动态障碍物

---

## 第十二部分：调试命令汇总

```bash
# 查看 Nav2 节点
ros2 node list | grep -E "(nav2|amcl|planner|controller)"

# 查看 Action 状态
ros2 action list
ros2 action info /navigate_to_pose

# 查看 Costmap
ros2 topic echo /global_costmap/costmap_raw --once
ros2 topic echo /local_costmap/costmap --once

# 查看 AMCL 位姿
ros2 topic echo /amcl_pose

# 查看规划路径
ros2 topic echo /plan

# 清除 Costmap
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap

# 查看 TF
ros2 run tf2_tools view_frames
```

---

## 总结

本课程学习了：

- ✅ Nav2 架构：BT Navigator、Planner、Controller、Recovery
- ✅ 行为树协调导航流程
- ✅ AMCL 粒子滤波定位原理与配置
- ✅ Costmap 代价地图：Global/Local、层结构、膨胀参数
- ✅ 使用 Action 发送导航目标
- ✅ 完整的自主导航流程实践

---

## 参考资料

- [Nav2 官方文档](https://navigation.ros.org/)
- [Nav2 配置指南](https://navigation.ros.org/configuration/index.html)
- [AMCL 参数说明](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [Costmap 2D 配置](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)
- [TurtleBot3 Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/)
