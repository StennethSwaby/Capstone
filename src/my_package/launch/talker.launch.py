from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',   # or 'demo_nodes_py' if you want Python version
            executable='talker',
            name='talker',
            output='screen'
        )
    ])

