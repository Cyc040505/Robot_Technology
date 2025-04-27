import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from pynput import keyboard
import threading
import math
import time
import re

class TurtleMaster(Node):
    def __init__(self):
        super().__init__('turtle_master')
        # 初始化速度发布器
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
        self.kill_attempts = {}  # 记录每个乌龟的kill尝试次数
        self.subscribed_turtles = set()  # 已订阅的乌龟集合
        
        # 初始化kill服务客户端
        self.kill_client = self.create_client(Kill, '/kill')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for kill service...')
        
        # 更频繁的碰撞检测定时器 (0.05秒)
        self.collision_check_timer = self.create_timer(0.05, self.check_collisions)
        
        # 动态发现新乌龟的定时器 (1秒)
        self.turtle_discovery_timer = self.create_timer(1.0, self.discover_new_turtles)
        
        # 启动键盘监听线程
        self.listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.listener.start()
        self.get_logger().info("Manual control enabled (↑:前进 ↓:后退 ←:左转 →:右转)")

        # 初始订阅
        self.subscribe_to_turtle('turtle1')

    def discover_new_turtles(self):
        """发现并订阅新生成的乌龟"""
        # 获取所有活跃的话题
        topic_names_and_types = self.get_topic_names_and_types()
        
        # 匹配乌龟pose话题的正则表达式
        turtle_pose_pattern = re.compile(r'/(turtle_\d+|turtle1)/pose')
        
        for topic_name, _ in topic_names_and_types:
            match = turtle_pose_pattern.match(topic_name)
            if match:
                turtle_name = match.group(1)
                if turtle_name not in self.subscribed_turtles:
                    self.subscribe_to_turtle(turtle_name)

    def main_turtle_pose_callback(self, msg):
        """更新主龟位置"""
        self.main_turtle_pose = msg

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
            self.kill_attempts[turtle_name] = 0
            self.get_logger().info(f'Subscribed to {turtle_name} pose')

    def other_turtle_pose_callback(self, msg, turtle_name):
        """更新其他乌龟位置"""
        self.other_turtle_poses[turtle_name] = msg

    def check_collisions(self):
        """检查碰撞"""
        if self.main_turtle_pose is None:
            return
            
        current_time = time.time()
        
        for turtle_name, pose in list(self.other_turtle_poses.items()):
            if pose is None or turtle_name == 'turtle1':
                continue
                
            # 计算距离 - 使用更精确的方法
            dx = self.main_turtle_pose.x - pose.x
            dy = self.main_turtle_pose.y - pose.y
            distance_squared = dx*dx + dy*dy
            
            # 碰撞阈值 (0.5的平方，避免开平方计算)
            if distance_squared < 0.25:  # 0.5^2 = 0.25
                self.get_logger().info(f'Collision detected with {turtle_name} at distance {math.sqrt(distance_squared):.2f}')
                self.kill_turtle(turtle_name)

    def kill_turtle(self, turtle_name):
        """删除乌龟"""
        # 检查是否已经尝试过太多次
        if self.kill_attempts.get(turtle_name, 0) >= 3:
            self.get_logger().warning(f'Too many failed attempts to kill {turtle_name}, giving up')
            self.cleanup_turtle(turtle_name)
            return
            
        req = Kill.Request()
        req.name = turtle_name
        future = self.kill_client.call_async(req)
        future.add_done_callback(lambda f: self.kill_result(f, turtle_name))
        
        # 增加尝试计数
        self.kill_attempts[turtle_name] = self.kill_attempts.get(turtle_name, 0) + 1

    def kill_result(self, future, turtle_name):
        """处理删除结果"""
        try:
            response = future.result()
            self.get_logger().info(f'Successfully killed {turtle_name}')
            self.cleanup_turtle(turtle_name)
        except Exception as e:
            self.get_logger().error(f'Failed to kill {turtle_name}: {str(e)}')
            # 如果失败，稍后重试
            if turtle_name in self.other_turtle_poses and self.other_turtle_poses[turtle_name] is not None:
                self.get_logger().info(f'Will retry killing {turtle_name} later')
                # 0.5秒后重试
                threading.Timer(0.5, lambda: self.kill_turtle(turtle_name)).start()

    def cleanup_turtle(self, turtle_name):
        """清理乌龟相关数据"""
        if turtle_name in self.other_turtle_poses:
            del self.other_turtle_poses[turtle_name]
        if turtle_name in self.kill_attempts:
            del self.kill_attempts[turtle_name]
        if turtle_name in self.subscribed_turtles:
            self.subscribed_turtles.remove(turtle_name)

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
        # 使用多线程运行ROS2节点
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
