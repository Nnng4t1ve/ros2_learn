#!/usr/bin/env python3
"""
ROS2 海龟画圆控制节点
功能：控制turtlesim中的海龟以圆形轨迹运动
作者：ROS2学习项目
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleCircle(Node):
    """
    海龟圆形运动控制类
    继承自ROS2的Node基类，实现海龟的圆形运动控制
    """
    
    def __init__(self):
        """
        初始化函数
        创建发布者和定时器，设置节点名称为'turtle_circle'
        """
        super().__init__('turtle_circle')  # 调用父类构造函数，设置节点名称
        
        # 创建Twist消息发布者，用于发送速度命令到海龟
        # 话题名：/turtle1/cmd_vel，队列大小：10
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 创建定时器，每0.1秒（10Hz频率）调用一次回调函数
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 输出节点启动信息
        self.get_logger().info('海龟圆形运动节点已启动')

    def timer_callback(self):
        """
        定时器回调函数
        每次被调用时发送速度命令，使海龟做圆形运动
        """
        # 创建Twist消息对象，用于存储速度命令
        msg = Twist()
        
        # 设置线性速度（前进速度）为1.0 m/s
        msg.linear.x = 1.0
        
        # 设置角速度（旋转速度）为1.0 rad/s
        # 同时有线性和角速度会产生圆形轨迹
        msg.angular.z = 1.0
        
        # 发布速度命令消息
        self.publisher_.publish(msg)


def main(args=None):
    """
    主函数
    初始化ROS2，创建节点并运行
    """
    # 初始化ROS2 Python客户端库
    rclpy.init(args=args)
    
    # 创建海龟圆形运动节点实例
    node = TurtleCircle()
    
    # 保持节点运行，等待回调函数被调用
    rclpy.spin(node)
    
    # 清理节点资源
    node.destroy_node()
    
    # 关闭ROS2 Python客户端库
    rclpy.shutdown()


if __name__ == '__main__':
    """
    程序入口点
    当脚本被直接执行时调用main函数
    """
    main()
