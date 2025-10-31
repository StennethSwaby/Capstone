from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---- Launch args (defaults tuned for a real TurtleBot3) ----
    use_sim_time = LaunchConfiguration('use_sim_time')     # false by default below
    base_frame   = LaunchConfiguration('base_frame')       # TB3 uses base_footprint
    scan_topic   = LaunchConfiguration('scan_topic')       # usually /scan
    start_rviz   = LaunchConfiguration('start_rviz')       # toggle RViz

    # Parameters YAML in this package (use your actual filename)
    default_params = PathJoinSubstitution([
        FindPackageShare('slam_toolbox_bringup'),
        'config',
        'mapper_parameters_online_async.yaml'
    ])
    params_file = LaunchConfiguration('params_file')

    # RViz config in this package
    default_rviz = PathJoinSubstitution([
        FindPackageShare('slam_toolbox_bringup'),
        'config',
        'view_robot.rviz'
    ])
    rviz_config = LaunchConfiguration('rviz_config')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        # Put YAML first, then inline overrides so the overrides win.
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'base_frame': base_frame,
                'scan_topic': scan_topic,
                'odom_frame': 'odom',
                'map_frame':  'map',
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (/clock). Set true only in Gazebo.'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot base frame (TB3 typically base_footprint).'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Laser scan topic.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to SLAM Toolbox parameters YAML.'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Full path to an RViz2 config file.'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 along with SLAM.'
        ),

        slam_node,
        rviz_node,
    ])
