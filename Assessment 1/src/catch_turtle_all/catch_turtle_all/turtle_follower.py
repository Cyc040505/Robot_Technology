import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleFollower(Node):
    def __init__(self, turtle_name, leader_name):
        super().__init__(f'{turtle_name}_follower')
        self.turtle_name = turtle_name
        self.leader_name = leader_name
        self.pose = None
        self.leader_pose = None
        
        # Create publisher for this turtle's velocity
        self.velocity_publisher = self.create_publisher(
            Twist, f'/{turtle_name}/cmd_vel', 10)
            
        # Subscribe to this turtle's pose
        self.pose_subscriber = self.create_subscription(
            Pose, f'/{turtle_name}/pose', self.update_pose, 10)
            
        # Subscribe to leader turtle's pose
        self.leader_pose_subscriber = self.create_subscription(
            Pose, f'/{leader_name}/pose', self.update_leader_pose, 10)
            
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.follow_leader)
        
    def update_pose(self, data):
        self.pose = data
        
    def update_leader_pose(self, data):
        self.leader_pose = data
        
    def follow_leader(self):
        if self.pose is None or self.leader_pose is None:
            return
            
        # Calculate distance and angle to leader
        dx = self.leader_pose.x - self.pose.x
        dy = self.leader_pose.y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # If too close, stop moving
        if distance < 1.0:
            twist = Twist()
            self.velocity_publisher.publish(twist)
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
            twist.linear.x = min(2.0, distance - 1.0)  # Maintain 1.0 distance
            
        self.velocity_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    # This would be called from turtle_controller, not directly
    rclpy.shutdown()

if __name__ == '__main__':
    main()