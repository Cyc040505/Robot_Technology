from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='catch_turtle_all',
            executable='turtle_spawner',
            name='turtle_spawner'
        ),
        Node(
            package='catch_turtle_all',
            executable='master_turtle',
            name='master_turtle'
        ),
        Node(
            package='catch_turtle_all',
            executable='turtle_controller',
            name='turtle_controller'
        ),
    ])