#!/usr/bin/env python3
"""
虚拟遥控器节点 - 通过命令行输入路径点进行导航
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import threading


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
        
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)
        
        msg = Twist()
        
        if distance < self.distance_tolerance:
            print(f'\n>>> 已到达目标点 ({self.goal_x:.2f}, {self.goal_y:.2f})')
            self.stop_robot()
            return
        
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
