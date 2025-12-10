#!/usr/bin/env python3
"""
ROS2 海龟位置订阅器 - 教学示例

这个程序演示了如何在ROS2中创建一个订阅器(Subscriber)来接收海龟的位置信息。
通过订阅turtlesim节点发布的位置数据，我们可以实时监控海龟在窗口中的位置和朝向。

学习要点：
1. 如何创建ROS2订阅器
2. 如何处理接收到的消息
3. 回调函数的使用
4. 消息类型的理解
"""

# 导入必要的ROS2库
import rclpy                    # ROS2 Python客户端库的核心模块
from rclpy.node import Node     # ROS2节点基类，所有节点都需要继承这个类
from turtlesim.msg import Pose  # 导入海龟位置消息类型，包含x, y, theta等位置信息
import time                     # 导入时间模块，用于控制输出频率


class TurtlePoseSub(Node):
    """
    海龟位置订阅器类
    
    这个类继承自ROS2的Node基类，实现了一个专门用于订阅海龟位置信息的节点。
    主要功能是接收并显示海龟的实时位置数据。
    """
    
    def __init__(self):
        """
        初始化海龟位置订阅器
        
        在这个构造函数中，我们完成以下工作：
        1. 调用父类构造函数，设置节点名称
        2. 创建订阅器，指定要订阅的话题和消息类型
        3. 设置回调函数来处理接收到的消息
        4. 初始化频率控制变量
        """
        # 调用父类构造函数，创建名为'turtle_pose_sub'的ROS2节点
        super().__init__('turtle_pose_sub')
        
        # 频率控制变量 - 用于降低输出频率，避免性能问题
        self.last_print_time = 0.0      # 上次输出的时间戳
        self.print_interval = 0.5       # 输出间隔时间（秒），设置为3秒降低频率
        self.message_count = 0          # 接收到的消息总数，用于统计
        self.skip_count = 0             # 跳过的消息数量统计
        
        # 创建订阅器 - 这是订阅器的核心部分
        self.subscription = self.create_subscription(
            Pose,                    # 消息类型：Pose包含x, y坐标和theta角度
            '/turtle1/pose',         # 话题名称：海龟发布位置信息的话题
            self.callback,           # 回调函数：当收到消息时调用的函数
            10                       # 队列大小：最多缓存10条未处理的消息
        )
        
        # 输出启动信息，确认订阅器已经开始工作
        self.get_logger().info(f'海龟位置订阅器已启动，输出频率设置为每{self.print_interval}秒一次')

    def callback(self, msg: Pose):
        """
        消息回调函数 - 低频率输出版本
        
        这个函数会在每次收到新的位置消息时被自动调用。
        为了避免输出过于频繁影响性能和可读性，我们设置了较长的时间间隔控制。
        
        性能优化策略：
        - 设置3秒输出间隔，大幅降低日志频率
        - 统计跳过的消息数量，了解实际接收频率
        - 只在需要时输出，但保持数据处理的完整性
        
        参数:
            msg (Pose): 包含海龟位置信息的消息对象
                       - msg.x: 海龟的x坐标 (0-11范围)
                       - msg.y: 海龟的y坐标 (0-11范围)  
                       - msg.theta: 海龟的朝向角度 (弧度制)
                       - msg.linear_velocity: 线速度
                       - msg.angular_velocity: 角速度
        """
        # 增加消息计数器
        self.message_count += 1
        
        # 获取当前时间
        current_time = time.time()
        
        # 频率控制：只有当距离上次输出超过指定间隔时才输出
        if current_time - self.last_print_time >= self.print_interval:
            # 格式化输出海龟的位置信息，包含统计信息
            self.get_logger().info(
                f'=== 海龟状态报告 === [总消息: {self.message_count}, 跳过: {self.skip_count}]\n'
                f'位置: x={msg.x:.2f}, y={msg.y:.2f}\n'
                f'朝向: {msg.theta:.2f}弧度 ({msg.theta * 180 / 3.14159:.1f}度)\n'
                f'速度: 线速度={msg.linear_velocity:.2f}, 角速度={msg.angular_velocity:.2f}\n'
                f'下次报告将在{self.print_interval}秒后...'
            )
            
            # 更新上次输出时间，重置跳过计数
            self.last_print_time = current_time
            self.skip_count = 0
        else:
            # 统计跳过的消息数量
            self.skip_count += 1
        
        # 注意：即使不输出日志，我们仍然接收并处理每一条消息
        # 这样可以保证数据的实时性，只是大幅降低了显示频率


def main(args=None):
    """
    主函数 - 程序的入口点
    
    这个函数负责：
    1. 初始化ROS2系统
    2. 创建并启动订阅器节点
    3. 保持节点运行以持续接收消息
    4. 程序结束时进行清理工作
    
    参数:
        args: 命令行参数，通常为None
    """
    # 初始化ROS2 Python客户端库
    # 这是使用ROS2功能的必要步骤
    rclpy.init(args=args)
    
    # 创建海龟位置订阅器节点实例
    node = TurtlePoseSub()
    
    try:
        # 启动节点的事件循环
        # spin()函数会让节点持续运行，等待并处理接收到的消息
        # 这个函数会一直阻塞，直到节点被关闭（如按Ctrl+C）
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获键盘中断信号（Ctrl+C），优雅地关闭程序
        print('\n程序被用户中断，正在关闭...')
    finally:
        # 清理工作：销毁节点并关闭ROS2系统
        node.destroy_node()  # 销毁节点，释放相关资源
        rclpy.shutdown()     # 关闭ROS2系统


# Python程序入口点
# 当直接运行这个脚本时（而不是作为模块导入），会执行main()函数
if __name__ == '__main__':
    main()
