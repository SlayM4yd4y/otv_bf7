from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='otv_bf7',
            executable='iranyitas',
            output='screen',
        ),
        Node(
            package='otv_bf7',
            executable='draw_node',
            output='screen',
        ),
    ])