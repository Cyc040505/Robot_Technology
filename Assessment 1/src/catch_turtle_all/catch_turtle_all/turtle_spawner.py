import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Pose
import random


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.client = self.create_client(Spawn, '/spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        # Generate a turtle every 3 seconds
        self.timer = self.create_timer(3.0, self.spawn_new_turtle)
        self.turtle_count = 1  # amount of turtle

    def spawn_new_turtle(self):
        request = Spawn.Request()
        request.x = random.uniform(0.0, 11.0)  # random x-coordinate
        request.y = random.uniform(0.0, 11.0)  # random y-coordinate
        request.theta = random.uniform(0.0, 6.28)  # random direction
        request.name = f'turtle_{self.turtle_count}'  # turtle name

        self.client.call_async(request).add_done_callback(self.spawn_callback)
        self.turtle_count += 1
        self.get_logger().info(f'Spawning turtle at ({request.x:.2f}, {request.y:.2f})')

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Spawned turtle: {response.name}')
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle: {e}')


def main(args=None):
    rclpy.init(args=args)
    spawner = TurtleSpawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()