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

        self.captured_order = []  # Queue to record the order of capture

        # Timer to dynamically discover new turtles (1 second)
        self.turtle_discovery_timer = self.create_timer(1.0, self.discover_new_turtles)

        # Control timer for captured turtles (0.1 second)
        self.follow_control_timer = self.create_timer(0.1, self.control_captured_turtles)

    def discover_new_turtles(self):
        """Discover and subscribe to new turtles"""
        topic_names_and_types = self.get_topic_names_and_types()
        turtle_pose_pattern = re.compile(r'/(turtle_\d+|turtle1)/pose')
        
        for topic_name, _ in topic_names_and_types:
            match = turtle_pose_pattern.match(topic_name)
            if match:
                turtle_name = match.group(1)
                if turtle_name not in self.subscribed_turtles:
                    self.subscribe_to_turtle(turtle_name)

    def subscribe_to_turtle(self, turtle_name):
        """Subscribe to the position of a turtle"""
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
        """Update the position of other turtles"""
        self.other_turtle_poses[turtle_name] = msg

    def check_collisions(self):
        """Check for collisions and capture turtles"""
        if self.main_turtle_pose is None:
            return
            
        for turtle_name, pose in list(self.other_turtle_poses.items()):
            if pose is None or turtle_name == 'turtle1' or turtle_name in self.captured_turtles:
                continue
            # Calculate squared distance
            dx = self.main_turtle_pose.x - pose.x
            dy = self.main_turtle_pose.y - pose.y
            distance_squared = dx*dx + dy*dy
            # Capture threshold (square of 0.5)
            if distance_squared < 0.4:
                self.capture_turtle(turtle_name)

    def capture_turtle(self, turtle_name):
        """Capture a turtle and initialize its velocity publisher"""
        if turtle_name not in self.captured_turtles:
            self.captured_order.append(turtle_name)  # Record the order of capture
            # Create a velocity publisher for the captured turtle
            cmd_vel_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
            self.captured_turtles[turtle_name] = {
                'publisher': cmd_vel_pub,
                'last_cmd': Twist()
            }
            # Publish capture message
            msg = String()
            msg.data = turtle_name
            self.turtle_captured_publisher.publish(msg)
            self.get_logger().info(f'Controlling {turtle_name} to follow turtle1')

    def control_captured_turtles(self):
        """Control all captured turtles to follow the previous one"""
        if not self.main_turtle_pose or not self.captured_order:
            return

        for i, turtle_name in enumerate(self.captured_order):
            # Determine the target to follow: always the previous element in the queue
            if i == 0:
                # The first one follows the main turtle, but the main turtle is not in the queue, so handle separately
                leader_pose = self.main_turtle_pose
            else:
                leader_name = self.captured_order[i-1]
                leader_pose = self.other_turtle_poses.get(leader_name)
            
            # Unified parameter settings
            follow_distance = 0.5  # Fixed follow distance
            max_speed = 3.0        # Increased maximum linear speed to 3.0
            angular_gain = 1.5   # Increased angular gain
            
            if not leader_pose or turtle_name not in self.other_turtle_poses:
                continue

            current_pose = self.other_turtle_poses[turtle_name]
            dx = leader_pose.x - current_pose.x
            dy = leader_pose.y - current_pose.y
            target_distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            
            # Angle difference calculation and normalization
            angle_diff = target_angle - current_pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # Dynamic speed control
            cmd = Twist()
            if target_distance > follow_distance:
                # Linear speed: the further the distance, the faster the speed, but not exceeding the upper limit
                cmd.linear.x = min(1.2 * target_distance, max_speed)  # Increased gain to 1.2
                # Angular speed: the larger the deviation, the faster the response
                cmd.angular.z = angular_gain * angle_diff
                # Limit angular speed to prevent jitter
                cmd.angular.z = max(min(cmd.angular.z, 4.0), -4.0)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            
            # Publish command
            self.captured_turtles[turtle_name]['publisher'].publish(cmd)

    def update_main_turtle_pose(self, msg):
        """Update the position of the main turtle"""
        self.main_turtle_pose = msg
        self.check_collisions()  # Ensure collision detection is called
        self.get_logger().info(f"Main turtle updated: ({msg.x:.2f}, {msg.y:.2f})")