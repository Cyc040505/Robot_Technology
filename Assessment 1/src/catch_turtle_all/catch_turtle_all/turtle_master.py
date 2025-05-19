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
        self.get_logger().info("Manual control enabled (↑:Move forward ↓:Move backward ←:Turn Left →:Turn right)")

        # 初始化 TurtleFollower
        self.turtle_captured_publisher = self.create_publisher(String, '/turtle_captured', 10)
        self.turtle_follower = TurtleFollower(
            self.turtle_captured_publisher
        )

        # 存储所有海龟位置
        self.turtle_poses = {}  
        self.current_target = None
        # 订阅主龟位置
        self.create_subscription(Pose, '/turtle1/pose', self.main_turtle_pose_callback, 10)
        # 订阅其他海龟位置
        self.create_subscription(String, '/turtle_captured', self.turtle_spawn_callback, 10)
        # 路径规划定时器
        self.create_timer(0.1, self.update_target)
        
        # 初始化跟随系统
        self.turtle_captured_publisher = self.create_publisher(String, '/turtle_captured', 10)
        self.turtle_follower = TurtleFollower(self.turtle_captured_publisher)


    def main_turtle_pose_callback(self, msg):
        """更新主龟位置"""
        self.main_turtle_pose = msg
        self.turtle_follower.update_main_turtle_pose(msg)

    def turtle_spawn_callback(self, msg):
        """监听新生成的海龟"""
        turtle_name = msg.data
        self.create_subscription(
            Pose,
            f'/{turtle_name}/pose',
            lambda msg, name=turtle_name: self.turtle_pose_callback(msg, name),
            10)
        
    def turtle_pose_callback(self, msg, turtle_name):
        """更新其他海龟位置"""
        self.turtle_poses[turtle_name] = msg

    def turtle_pose_callback(self, msg, turtle_name):
        """更新其他海龟位置"""
        self.turtle_poses[turtle_name] = msg

    def update_target(self):
        """选择最近的未捕获海龟作为目标"""
        if not self.turtle_poses or not self.main_turtle_pose:
            return
        
        # 过滤未被捕获的海龟
        available_turtles = {
            name: pose for name, pose in self.turtle_poses.items()
            if name not in self.turtle_follower.captured_turtles
        }
        if not available_turtles:
            return
        
        # 计算欧氏距离
        closest_turtle = min(
            available_turtles.items(),
            key=lambda item: math.hypot(
                item[1].x - self.main_turtle_pose.x,
                item[1].y - self.main_turtle_pose.y
            )
        )
        self.current_target = closest_turtle[1]
        self.navigate_to_target()

    def navigate_to_target(self):
        """控制主龟移动至目标"""
        dx = self.current_target.x - self.main_turtle_pose.x
        dy = self.current_target.y - self.main_turtle_pose.y
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.main_turtle_pose.theta
        
        # 规范化角度差到[-π, π]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        cmd = Twist()
        if abs(angle_diff) > 0.1:  # 角度偏差较大时先转向
            cmd.angular.z = 0.8 if angle_diff > 0 else -0.8
        else:                      # 角度对齐后前进
            cmd.linear.x = 1.5 * math.hypot(dx, dy)
            cmd.linear.x = min(cmd.linear.x, 2.0)  # 限速
        
        self.publisher_.publish(cmd)

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