import re
import math
import time
import rclpy
import threading

from pynput import keyboard
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TurtleMaster(Node):
    def __init__(self):
        super().__init__('turtle_master')
        # 初始化速度发布器（主龟）
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 初始化速度参数
        self.linear_speed = 2.0
        self.angular_speed = 1.0
        self.current_twist = Twist()
        
        # 主龟位置
        self.main_turtle_pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.main_turtle_pose_callback, 10)
        
        # 其他乌龟位置
        self.other_turtle_poses = {}
        self.captured_turtles = {}  # 被抓捕的乌龟及其速度发布器
        self.subscribed_turtles = set()  # 已订阅的乌龟集合
        
        # 碰撞检测定时器 (0.05秒)
        self.collision_check_timer = self.create_timer(0.05, self.check_collisions)
        
        # 动态发现新乌龟的定时器 (1秒)
        self.turtle_discovery_timer = self.create_timer(1.0, self.discover_new_turtles)
        
        # 被抓捕乌龟的控制定时器 (0.1秒)
        self.follow_control_timer = self.create_timer(0.1, self.control_captured_turtles)
        
        # 启动键盘监听线程
        self.listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.listener.start()
        self.get_logger().info("Manual control enabled (↑:前进 ↓:后退 ←:左转 →:右转)")

        # 初始订阅
        self.subscribe_to_turtle('turtle1')
        
        self.turtle_captured_publisher = self.create_publisher(String, '/turtle_captured', 10)

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

    def main_turtle_pose_callback(self, msg):
        """更新主龟位置"""
        self.main_turtle_pose = msg

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
            if distance_squared < 0.25:
                self.get_logger().info(f'Captured turtle {turtle_name} at distance {math.sqrt(distance_squared):.2f}')
                self.capture_turtle(turtle_name)

    def capture_turtle(self, turtle_name):
        """捕获乌龟并初始化速度发布器"""
        if turtle_name not in self.captured_turtles:
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
        """控制所有被抓捕的乌龟跟随主龟"""
        if self.main_turtle_pose is None:
            return
            
        for turtle_name, turtle_data in self.captured_turtles.items():
            pose = self.other_turtle_poses.get(turtle_name)
            if pose is None:
                continue
                
            # 计算与主龟的距离和角度差
            dx = self.main_turtle_pose.x - pose.x
            dy = self.main_turtle_pose.y - pose.y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # 生成速度指令
            cmd = Twist()
            if distance > 1.0:
                cmd.linear.x = min(2.0, distance * 0.5)
                cmd.angular.z = angle_diff * 2.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            
            # 发布速度指令
            turtle_data['publisher'].publish(cmd)
            turtle_data['last_cmd'] = cmd

    def on_key_press(self, key):
        """处理按键按下事件"""
        try:
            if key == keyboard.Key.up:
                self.current_twist.linear.x = self.linear_speed
            elif key == keyboard.Key.down:
                self.current_twist.linear.x = -self.linear_speed
            elif key == keyboard.Key.left:
                self.current_twist.angular.z = self.angular_speed
            elif key == keyboard.Key.right:
                self.current_twist.angular.z = -self.angular_speed
            self.publisher_.publish(self.current_twist)
        except AttributeError:
            pass

    def on_key_release(self, key):
        """处理按键释放事件"""
        if key in [keyboard.Key.up, keyboard.Key.down]:
            self.current_twist.linear.x = 0.0
        elif key in [keyboard.Key.left, keyboard.Key.right]:
            self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

def main(args=None):
    rclpy.init(args=args)
    master = TurtleMaster()
    
    try:
        spin_thread = threading.Thread(target=rclpy.spin, args=(master,))
        spin_thread.start()
        spin_thread.join()
    except KeyboardInterrupt:
        master.get_logger().info("Manual control disabled")
    finally:
        master.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
