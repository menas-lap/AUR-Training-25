from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )
    game_node = Node(
        package='turtle_pkg',
        executable='turtle_chase',
        name='turtle_chase'
    )
    return LaunchDescription([turtlesim_node, game_node])
