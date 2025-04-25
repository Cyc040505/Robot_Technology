import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading

class TurtleMaster(Node):
    def __init__(self):
        super().__init__('turtle_master')
        # 初始化速度发布器
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 初始化速度参数
        self.linear_speed = 2.0
        self.angular_speed = 1.0
        self.current_twist = Twist()
        
        # 启动键盘监听线程
        self.listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.listener.start()
        self.get_logger().info("Manual control enabled (↑:前进 ↓:后退 ←:左转 →:右转)")

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