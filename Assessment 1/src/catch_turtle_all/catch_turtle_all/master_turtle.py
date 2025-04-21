import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
import math
from functools import partial

class MasterTurtle(Node):
    def __init__(self):
        super().__init__('master_turtle')
        self.pose = None
        self.target_turtle = None
        self.target_pose = None
        
        # Create publisher for master turtle velocity
        self.velocity_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
            
        # Subscribe to master turtle pose
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.update_pose, 10)
            
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.move_to_target)
        
    def update_pose(self, data):
        self.pose = data
        
    def set_target(self, turtle_name):
        self.target_turtle = turtle_name
        # Subscribe to target turtle pose
        self.target_pose_subscriber = self.create_subscription(
            Pose, f'/{turtle_name}/pose', self.update_target_pose, 10)
            
    def update_target_pose(self, data):
        self.target_pose = data
        
    def move_to_target(self):
        if self.pose is None or self.target_pose is None:
            return
            
        # Calculate distance and angle to target
        dx = self.target_pose.x - self.pose.x
        dy = self.target_pose.y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # If close enough, consider it caught
        if distance < 0.5:
            self.get_logger().info(f'Caught {self.target_turtle}!')
            self.target_turtle = None
            self.target_pose = None
            return
            
        # Calculate desired angle
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.pose.theta
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2*math.pi
        while angle_diff < -math.pi:
            angle_diff += 2*math.pi
            
        # Create and publish twist message
        twist = Twist()
        
        # Rotate first if angle difference is large
        if abs(angle_diff) > 0.1:
            twist.angular.z = 0.5 if angle_diff > 0 else -0.5
        else:
            # Move forward if pointing in the right direction
            twist.linear.x = min(2.0, distance)
            
        self.velocity_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    master_turtle = MasterTurtle()
    rclpy.spin(master_turtle)
    master_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()