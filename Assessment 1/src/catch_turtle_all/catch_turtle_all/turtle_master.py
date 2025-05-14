import math
import time
import rclpy
import threading
from pynput import keyboard
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from .turtle_follower import TurtleFollower

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
        
        # 启动键盘监听线程
        self.listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.listener.start()
        self.get_logger().info("Manual control enabled (↑:前进 ↓:后退 ←:左转 →:右转)")

        # 初始化 TurtleFollower
        self.turtle_captured_publisher = self.create_publisher(String, '/turtle_captured', 10)
        self.turtle_follower = TurtleFollower(
            self.turtle_captured_publisher
        )

    def main_turtle_pose_callback(self, msg):
        """更新主龟位置"""
        self.main_turtle_pose = msg
        self.turtle_follower.update_main_turtle_pose(msg)

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
    follower = master.turtle_follower

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(master)
    executor.add_node(follower)  # 确保添加了follower节点
    
    try:
        executor.spin()  # 仅使用executor处理事件循环
    except KeyboardInterrupt:
        master.get_logger().info("Manual control disabled")
    finally:
        master.destroy_node()
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()