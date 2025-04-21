import rclpy
from rclpy.node import Node
from catch_turtle_all.turtle_spawner import TurtleSpawner
from catch_turtle_all.master_turtle import MasterTurtle
from catch_turtle_all.turtle_follower import TurtleFollower
import threading

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.spawner = TurtleSpawner()
        self.master = MasterTurtle()
        self.followers = {}
        
        # Thread for spawner
        self.spawner_thread = threading.Thread(
            target=rclpy.spin, args=(self.spawner,))
        self.spawner_thread.start()
        
        # Thread for master
        self.master_thread = threading.Thread(
            target=rclpy.spin, args=(self.master,))
        self.master_thread.start()
        
        # Timer to check for new turtles
        self.timer = self.create_timer(1.0, self.update_target)
        
    def update_target(self):
        # If master has no target, assign the nearest turtle
        if self.master.target_turtle is None and len(self.spawner.alive_turtles) > 0:
            # Find nearest turtle (simple implementation - just pick first)
            if self.spawner.alive_turtles:
                target = self.spawner.alive_turtles[0]
                self.master.set_target(target)
                
        # Check if master caught a turtle
        if self.master.target_turtle is not None and self.master.target_pose is None:
            caught_turtle = self.master.target_turtle
            self.spawner.kill_turtle(caught_turtle)
            
            # Create follower for caught turtle
            leader = 'turtle1'  # Master turtle
            follower_node = TurtleFollower(caught_turtle, leader)
            self.followers[caught_turtle] = follower_node
            
            # Start follower in a new thread
            follower_thread = threading.Thread(
                target=rclpy.spin, args=(follower_node,))
            follower_thread.start()

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleController()
    rclpy.spin(controller)
    
    # Cleanup
    controller.spawner.destroy_node()
    controller.master.destroy_node()
    for follower in controller.followers.values():
        follower.destroy_node()
    rclpy.shutdown()
    
    # Wait for threads to finish
    controller.spawner_thread.join()
    controller.master_thread.join()

if __name__ == '__main__':
    main()