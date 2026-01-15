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
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 运动参数
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.5  # rad/s
        
        # 定时器 - 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 运动状态
        self.state = 'forward'
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
            if elapsed > 2.0:
                self.state = 'turn'
                self.state_start_time = current_time
                self.get_logger().info('切换到旋转状态')
                
        elif self.state == 'turn':
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed
            if elapsed > 2.0:
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
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.get_logger().info('节点已停止，机器人已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
