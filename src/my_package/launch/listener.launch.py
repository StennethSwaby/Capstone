from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',  # or 'demo_nodes_py' if using Python version
            executable='listener',
            name='listener',
            output='screen'
        )
    ])
