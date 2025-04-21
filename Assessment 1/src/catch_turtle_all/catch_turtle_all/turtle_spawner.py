import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
import random
import math
from functools import partial

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.timer = self.create_timer(3.0, self.spawn_new_turtle)
        self.turtle_count = 0
        self.alive_turtles = []
        
    def spawn_new_turtle(self):
        # Generate random position and angle
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 2*math.pi)
        
        # Create turtle name
        self.turtle_count += 1
        turtle_name = f'turtle{self.turtle_count}'
        
        # Call spawn service
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')
            
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name
        
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.spawn_callback, turtle_name=turtle_name))
    
    def spawn_callback(self, future, turtle_name):
        try:
            response = future.result()
            if response.name == turtle_name:
                self.alive_turtles.append(turtle_name)
                self.get_logger().info(f'Spawned {turtle_name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def kill_turtle(self, turtle_name):
        if turtle_name in self.alive_turtles:
            client = self.create_client(Kill, 'kill')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('kill service not available, waiting again...')
            
            request = Kill.Request()
            request.name = turtle_name
            
            future = client.call_async(request)
            future.add_done_callback(
                partial(self.kill_callback, turtle_name=turtle_name))
    
    def kill_callback(self, future, turtle_name):
        try:
            future.result()
            self.alive_turtles.remove(turtle_name)
            self.get_logger().info(f'Killed {turtle_name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    turtle_spawner = TurtleSpawner()
    rclpy.spin(turtle_spawner)
    turtle_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()