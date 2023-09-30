from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basic_ros2',
            namespace='Publisher',
            executable='pub',
            name='sim'
        ),
        Node(
            package='basic_ros2',
            namespace='Subscriber',
            executable='sub',
            name='sim'
        ),
    ])