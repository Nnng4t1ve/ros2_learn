#!/usr/bin/env python3
"""
ROS2 海龟跟随者节点 - 让第二只海龟跟随第一只海龟

这个程序演示了如何创建一个跟随者节点，让第二只海龟跟随第一只海龟的轨迹。
主要功能：
1. 订阅第一只海龟的位置信息
2. 生成第二只海龟
3. 控制第二只海龟跟随第一只海龟

学习要点：
1. 多海龟系统的管理
2. 服务调用（生成新海龟）
3. 位置跟踪算法
4. 速度控制策略
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import math
import time


class TurtleFollower(Node):
    """
    海龟跟随者类
    
    这个类实现了一个跟随者节点，让第二只海龟跟随第一只海龟的轨迹。
    """
    
    def __init__(self):
        """初始化跟随者节点"""
        super().__init__('turtle_follower')
        
        # 跟随参数
        self.follow_distance = 2.0      # 期望的跟随距离
        self.max_linear_speed = 2.0     # 最大线速度
        self.max_angular_speed = 2.0    # 最大角速度
        self.speed_gain = 1.0           # 速度增益
        self.angular_gain = 4.0         # 角速度增益
        
        # 海龟状态
        self.turtle1_pose = None        # 第一只海龟的位置
        self.turtle2_pose = None        # 第二只海龟的位置
        self.turtle2_name = 'turtle2'   # 第二只海龟的名称
        
        # 创建订阅者 - 订阅第一只海龟的位置
        self.turtle1_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1_pose_callback,
            10
        )
        
        # 创建订阅者 - 订阅第二只海龟的位置
        self.turtle2_sub = self.create_subscription(
            Pose,
            f'/{self.turtle2_name}/pose',
            self.turtle2_pose_callback,
            10
        )
        
        # 创建发布者 - 控制第二只海龟的速度
        self.turtle2_cmd_pub = self.create_publisher(
            Twist,
            f'/{self.turtle2_name}/cmd_vel',
            10
        )
        
        # 创建服务客户端 - 用于生成新海龟
        self.spawn_client = self.create_client(Spawn, '/spawn')
        
        # 创建定时器 - 控制跟随逻辑的执行频率
        self.control_timer = self.create_timer(0.1, self.control_callback)  # 10Hz
        
        # 等待服务可用并生成第二只海龟
        self.spawn_turtle2()
        
        self.get_logger().info('海龟跟随者节点已启动')
        self.get_logger().info(f'第二只海龟将跟随第一只海龟，保持{self.follow_distance}米距离')

    def spawn_turtle2(self):
        """生成第二只海龟"""
        # 等待spawn服务可用
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待spawn服务...')
        
        # 创建spawn请求
        request = Spawn.Request()
        request.x = 2.0  # 初始x位置
        request.y = 2.0  # 初始y位置
        request.theta = 0.0  # 初始朝向
        request.name = self.turtle2_name  # 海龟名称
        
        # 发送异步请求
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_response_callback)

    def spawn_response_callback(self, future):
        """处理spawn服务的响应"""
        try:
            response = future.result()
            self.get_logger().info(f'成功生成第二只海龟: {response.name}')
        except Exception as e:
            self.get_logger().error(f'生成第二只海龟失败: {e}')

    def turtle1_pose_callback(self, msg: Pose):
        """第一只海龟位置回调函数"""
        self.turtle1_pose = msg

    def turtle2_pose_callback(self, msg: Pose):
        """第二只海龟位置回调函数"""
        self.turtle2_pose = msg

    def control_callback(self):
        """控制回调函数 - 实现跟随逻辑"""
        # 检查是否已经获得两只海龟的位置信息
        if self.turtle1_pose is None or self.turtle2_pose is None:
            return
        
        # 计算两只海龟之间的距离和角度
        dx = self.turtle1_pose.x - self.turtle2_pose.x
        dy = self.turtle1_pose.y - self.turtle2_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 计算目标角度（从turtle2指向turtle1）
        target_angle = math.atan2(dy, dx)
        
        # 计算角度差（考虑角度的周期性）
        angle_diff = target_angle - self.turtle2_pose.theta
        
        # 将角度差规范化到[-π, π]范围
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 创建速度命令
        cmd = Twist()
        
        # 距离控制：如果距离大于期望距离，则前进
        distance_error = distance - self.follow_distance
        if distance_error > 0.1:  # 死区，避免震荡
            # 线速度与距离误差成正比
            cmd.linear.x = min(self.speed_gain * distance_error, self.max_linear_speed)
        else:
            cmd.linear.x = 0.0
        
        # 角度控制：调整朝向以面向目标
        if abs(angle_diff) > 0.1:  # 角度死区
            cmd.angular.z = min(max(self.angular_gain * angle_diff, -self.max_angular_speed), 
                               self.max_angular_speed)
        else:
            cmd.angular.z = 0.0
        
        # 发布速度命令
        self.turtle2_cmd_pub.publish(cmd)
        
        # 定期输出状态信息
        if hasattr(self, 'last_log_time'):
            if time.time() - self.last_log_time > 2.0:  # 每2秒输出一次
                self.log_status(distance, angle_diff)
                self.last_log_time = time.time()
        else:
            self.last_log_time = time.time()

    def log_status(self, distance, angle_diff):
        """输出状态信息"""
        self.get_logger().info(
            f'跟随状态 - 距离: {distance:.2f}m (目标: {self.follow_distance}m), '
            f'角度差: {angle_diff*180/math.pi:.1f}°'
        )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = TurtleFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n跟随者节点被用户中断，正在关闭...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()