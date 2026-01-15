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
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # 输出频率控制
        self.last_print_time = 0.0
        self.print_interval = 1.0
        
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
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            yaw_deg = math.degrees(yaw)
            
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
