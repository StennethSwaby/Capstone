import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('controlled_env')
    default_world = os.path.join(pkg_share, 'worlds', 'controlled_maze.world')

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Absolute path to world file'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            LaunchConfiguration('world'),
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    return LaunchDescription([world_arg, gazebo])
