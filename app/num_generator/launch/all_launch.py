from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='num_generator',
            executable='generator',
            name='generator',
        ),
        Node(
            package='num_generator',
            executable='multiplier',
            name='multiplier',
        ),
        Node(
            package='num_generator',
            executable='feedback',
            name='feedback',
        ),
        Node(
            package='num_generator',
            executable='initialiser',
            name='initialiser',
        )
    ])
