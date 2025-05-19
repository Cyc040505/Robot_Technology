import re
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TurtleFollower(Node):
    def __init__(self, turtle_captured_publisher):
        super().__init__('turtle_follower')
        self.main_turtle_pose = None
        self.other_turtle_poses = {}
        self.captured_turtles = {}
        self.subscribed_turtles = set()
        self.turtle_captured_publisher = turtle_captured_publisher

        self.captured_order = []  # 记录捕获顺序的队列

        # 动态发现新乌龟的定时器 (1秒)
        self.turtle_discovery_timer = self.create_timer(1.0, self.discover_new_turtles)

        # 被抓捕乌龟的控制定时器 (0.1秒)
        self.follow_control_timer = self.create_timer(0.1, self.control_captured_turtles)

    def discover_new_turtles(self):
        """发现并订阅新生成的乌龟"""
        topic_names_and_types = self.get_topic_names_and_types()
        turtle_pose_pattern = re.compile(r'/(turtle_\d+|turtle1)/pose')
        
        for topic_name, _ in topic_names_and_types:
            match = turtle_pose_pattern.match(topic_name)
            if match:
                turtle_name = match.group(1)
                if turtle_name not in self.subscribed_turtles:
                    self.subscribe_to_turtle(turtle_name)

    def subscribe_to_turtle(self, turtle_name):
        """订阅乌龟的位置"""
        if turtle_name not in self.subscribed_turtles:
            self.create_subscription(
                Pose,
                f'/{turtle_name}/pose',
                lambda msg, name=turtle_name: self.other_turtle_pose_callback(msg, name),
                10)
            self.subscribed_turtles.add(turtle_name)
            self.other_turtle_poses[turtle_name] = None
            self.get_logger().info(f'Subscribed to {turtle_name} pose')

    def other_turtle_pose_callback(self, msg, turtle_name):
        """更新其他乌龟位置"""
        self.other_turtle_poses[turtle_name] = msg

    def check_collisions(self):
        """检查碰撞并捕获乌龟"""
        if self.main_turtle_pose is None:
            return
            
        for turtle_name, pose in list(self.other_turtle_poses.items()):
            if pose is None or turtle_name == 'turtle1' or turtle_name in self.captured_turtles:
                continue
            # 计算距离平方
            dx = self.main_turtle_pose.x - pose.x
            dy = self.main_turtle_pose.y - pose.y
            distance_squared = dx*dx + dy*dy
            # 捕获阈值 (0.5的平方)
            if distance_squared < 0.5:
                self.capture_turtle(turtle_name)

    def capture_turtle(self, turtle_name):
        """捕获乌龟并初始化速度发布器"""
        if turtle_name not in self.captured_turtles:
            self.captured_order.append(turtle_name)  # 记录捕获顺序
            # 为被抓捕的乌龟创建速度发布器
            cmd_vel_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
            self.captured_turtles[turtle_name] = {
                'publisher': cmd_vel_pub,
                'last_cmd': Twist()
            }
            # 发布捕获消息
            msg = String()
            msg.data = turtle_name
            self.turtle_captured_publisher.publish(msg)
            self.get_logger().info(f'Controlling {turtle_name} to follow turtle1')

    def control_captured_turtles(self):
        """控制所有被抓捕的乌龟跟随前一只乌龟"""
        if not self.main_turtle_pose or not self.captured_order:
            return

        for i, turtle_name in enumerate(self.captured_order):
            # 确定跟随目标：总是跟随队列中前一个元素
            if i == 0:
                # 第一个跟随主龟，但主龟不属于队列，需单独处理
                leader_pose = self.main_turtle_pose
            else:
                leader_name = self.captured_order[i-1]
                leader_pose = self.other_turtle_poses.get(leader_name)
            
            # 统一参数设定
            follow_distance = 1.0  # 固定跟随距离
            max_speed = 3.0        # 最大线速度提升至3.0
            angular_gain = 2.5     # 角速度增益提升
            
            if not leader_pose or turtle_name not in self.other_turtle_poses:
                continue

            current_pose = self.other_turtle_poses[turtle_name]
            dx = leader_pose.x - current_pose.x
            dy = leader_pose.y - current_pose.y
            target_distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            
            # 角度差计算与规范化
            angle_diff = target_angle - current_pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # 动态速度控制
            cmd = Twist()
            if target_distance > follow_distance:
                # 线速度：距离越远速度越快，但不超过上限
                cmd.linear.x = min(1.2 * target_distance, max_speed)  # 增益提升到1.2
                # 角速度：偏差越大响应越快
                cmd.angular.z = angular_gain * angle_diff
                # 角速度限幅防止抖动
                cmd.angular.z = max(min(cmd.angular.z, 4.0), -4.0)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            
            # 发布指令
            self.captured_turtles[turtle_name]['publisher'].publish(cmd)

    def update_main_turtle_pose(self, msg):
        """更新主龟位置"""
        self.main_turtle_pose = msg
        self.check_collisions()  # 确保调用碰撞检测
        self.get_logger().info(f"Main turtle updated: ({msg.x:.2f}, {msg.y:.2f})")