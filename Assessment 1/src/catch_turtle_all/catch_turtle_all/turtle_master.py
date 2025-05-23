import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from .turtle_follower import TurtleFollower

class TurtleMaster(Node):
    def __init__(self):
        super().__init__('turtle_master')
        # Initialize the velocity publisher (for the main turtle)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Initialize velocity parameters
        self.linear_speed = 2.0
        self.angular_speed = 1.0
        self.current_twist = Twist()
        
        # Subscribe to the main turtle's position
        self.main_turtle_pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.main_turtle_pose_callback, 10)
        
        # Initialize TurtleFollower
        self.turtle_captured_publisher = self.create_publisher(String, '/turtle_captured', 10)
        self.turtle_follower = TurtleFollower(
            self.turtle_captured_publisher
        )

        # Store the positions of all turtles
        self.turtle_poses = {}  
        self.current_target = None
        # Subscribe to the positions of other turtles
        self.create_subscription(String, '/turtle_captured', self.turtle_spawn_callback, 10)

    def main_turtle_pose_callback(self, msg):
        """Update the position of the main turtle"""
        self.main_turtle_pose = msg
        self.turtle_follower.update_main_turtle_pose(msg)

    def turtle_spawn_callback(self, msg):
        """Listen for newly spawned turtles"""
        turtle_name = msg.data
        self.create_subscription(
            Pose,
            f'/{turtle_name}/pose',
            lambda msg, name=turtle_name: self.turtle_pose_callback(msg, name),
            10)
        
    def turtle_pose_callback(self, msg, turtle_name):
        """Update the positions of other turtles"""
        self.turtle_poses[turtle_name] = msg
        self.update_target() 

    def update_target(self):
        """Select the nearest uncaught turtle as the target"""
        if not self.turtle_poses or not self.main_turtle_pose:
            return
        
        # Filter out uncaught turtles
        available_turtles = {
            name: pose for name, pose in self.turtle_poses.items()
            if name not in self.turtle_follower.captured_turtles
        }
        if not available_turtles:
            return
        
        # Calculate Euclidean distance
        closest_turtle = min(
            available_turtles.items(),
            key=lambda item: math.hypot(
                item[1].x - self.main_turtle_pose.x,
                item[1].y - self.main_turtle_pose.y
            )
        )
        target_pose = closest_turtle[1]
        self.navigate_to_target(target_pose)

    def navigate_to_target(self, target_pose):
        """Control the main turtle to move to the target position"""
        dx = target_pose.x - self.main_turtle_pose.x
        dy = target_pose.y - self.main_turtle_pose.y
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.main_turtle_pose.theta
        
        # Normalize the angle difference to [-π, π]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        cmd = Twist()
        # Prioritize turning if the angle difference is large
        if abs(angle_diff) > 0.1:
            cmd.angular.z = 1.5 if angle_diff > 0 else -1.5
        else:
            # Move forward after aligning the direction, with speed proportional to the distance
            distance = math.hypot(dx, dy)
            cmd.linear.x = min(2.0, distance * 0.8)
        
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    master = TurtleMaster()
    follower = master.turtle_follower

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(master)
    executor.add_node(follower)  # Ensure the follower node is added
    
    try:
        executor.spin()  # Use the executor to handle the event loop
    except KeyboardInterrupt:
        master.get_logger().info("Manual control disabled")
    finally:
        master.destroy_node()
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
