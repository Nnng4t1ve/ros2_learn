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
        self.distance_tolerance = 0.1
        self.angle_tolerance = 0.1
        self.linear_speed = 0.15
        self.angular_speed = 0.5
        
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
            self.stop_robot()
            self.get_logger().info('所有路径点导航完成！')
            self.timer.cancel()
            return
        
        goal_x, goal_y = self.waypoints[self.current_waypoint_idx]
        
        distance = self.get_distance_to_goal(goal_x, goal_y)
        angle_to_goal = self.get_angle_to_goal(goal_x, goal_y)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)
        
        msg = Twist()
        
        if distance < self.distance_tolerance:
            self.get_logger().info(
                f'到达路径点 {self.current_waypoint_idx + 1}: ({goal_x}, {goal_y})'
            )
            self.current_waypoint_idx += 1
            return
        
        if abs(angle_diff) > self.angle_tolerance:
            msg.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        else:
            msg.linear.x = self.linear_speed
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
