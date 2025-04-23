import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
import random
import math
from collections import defaultdict


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Waiting for Generation Server...')

        # Dynamic turtle position tracking
        self.turtle_poses = defaultdict(dict)
        self.active_subscriptions = set()

        # The initial subscription to the main turtle
        self.subscribe_to_turtle('turtle1')

        # Intelligently generate parameters
        self.min_safe_distance = 1.5  # Unit safe distance
        self.max_attempts = 100  # Maximum attempts
        self.spawn_interval = 3.0  # Basic generation interval

        # Adaptive timer
        self.create_timer(self.spawn_interval, self.adaptive_spawning)

    """Subscribe to the new turtle's location"""
    def subscribe_to_turtle(self, turtle_name):
        if turtle_name not in self.active_subscriptions:
            self.create_subscription(
                Pose,
                f'/{turtle_name}/pose',
                lambda msg, name=turtle_name: self.pose_callback(msg, name),
                10)
            self.active_subscriptions.add(turtle_name)
            self.get_logger().info(f'Have subscribed to the location of {turtle_name}')

    """Update the database of turtle locations"""
    def pose_callback(self, msg, turtle_name):
        self.turtle_poses[turtle_name] = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.theta
        }

    """Intelligent safe location generation algorithm"""
    def is_position_safe(self, x, y):
        for name, pos in self.turtle_poses.items():
            dist = math.sqrt((x - pos['x']) ** 2 + (y - pos['y']) ** 2)
            if dist < self.min_safe_distance:
                return False
        return True

    def calculate_safe_position(self):
        # Attempt to generate randomly
        for _ in range(self.max_attempts // 2):
            x = random.uniform(0.5, 10.5)
            y = random.uniform(0.5, 10.5)
            if self.is_position_safe(x, y):
                return x, y

        # Try spiral search (random generation failed)
        center_x, center_y = 5.5, 5.5
        for radius in [0.5, 1.0, 1.5, 2.0, 2.5]:
            for angle in range(0, 360, 30):
                rad = math.radians(angle)
                x = center_x + radius * math.cos(rad)
                y = center_y + radius * math.sin(rad)
                if 0 < x < 11 and 0 < y < 11 and self.is_position_safe(x, y):
                    return x, y

        return None, None  # No safe location

    """Adaptive generation logic"""
    def adaptive_spawning(self):
        x, y = self.calculate_safe_position()

        if x is not None:
            # generate new turtle
            turtle_name = f'turtle_{len(self.turtle_poses) + 1}'
            req = Spawn.Request()
            req.x = x
            req.y = y
            req.theta = random.uniform(0, 6.28)
            req.name = turtle_name

            future = self.spawn_client.call_async(req)
            future.add_done_callback(
                lambda f, name=turtle_name: self.spawn_result(f, name))

            self.get_logger().info(
                f'Generate {turtle_name} on ({x:.2f}, {y:.2f})')
        else:
            self.get_logger().warning(
                'No safe location, generation frequency reduced')
            # Dynamically adjust the generation interval
            self.spawn_interval = min(10.0, self.spawn_interval * 1.5)

    """Process the generated results"""
    def spawn_result(self, future, turtle_name):
        try:
            response = future.result()
            self.subscribe_to_turtle(turtle_name)
            self.get_logger().info(f'Successfully generate: {response.name}')
            # Reset the generation interval
            self.spawn_interval = 3.0
        except Exception as e:
            self.get_logger().error(f'Generation failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    spawner = TurtleSpawner()
    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        spawner.get_logger().info('Close Generator...')
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()