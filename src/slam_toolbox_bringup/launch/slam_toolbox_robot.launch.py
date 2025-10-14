from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Use Gazebo simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Default params file inside this bringup package
    default_params = PathJoinSubstitution([
        FindPackageShare('slam_toolbox_bringup'),
        'config',
        'mapper_params_online_async.yaml'
    ])
    params_file = LaunchConfiguration('params_file', default=default_params)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to a SLAM Toolbox parameters YAML'
        ),

        # Use *either* sync_slam_toolbox_node or async_slam_toolbox_node.
        # Async is typical for online SLAM in sim.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
        ),
    ])
