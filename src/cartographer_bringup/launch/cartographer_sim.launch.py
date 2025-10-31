from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = 'cartographer_bringup'
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([FindPackageShare(pkg), 'worlds', 'obstacles.world'])
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    configuration_basename_arg = DeclareLaunchArgument('configuration_basename', default_value='turtlebot3_lds_2d.lua')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare(pkg), 'rviz', 'cartographer.rviz'])
    )

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_basename = LaunchConfiguration('configuration_basename')
    rviz_config = LaunchConfiguration('rviz_config')

    # IMPORTANT: add a literal space between xacro and file path
    my_robot_xacro_path = PathJoinSubstitution([FindPackageShare(pkg), 'description', 'my_robot.xacro'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', my_robot_xacro_path])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        output='screen'
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world}.items()
    )

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(filter(None, [
            os.getenv('GZ_SIM_RESOURCE_PATH', ''),
            os.path.join(get_package_share_directory(pkg), 'worlds'),
            os.path.join(get_package_share_directory(pkg), 'description'),
        ]))
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=['-topic', 'robot_description', '-name', 'my_robot', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Adjust /world/<name>/... if your world name isn’t "default"
    lidar_and_cmdvel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        remappings=[
            ('/scan',    '/world/default/model/my_robot/link/laser_frame/sensor/laser/scan'),
            ('/cmd_vel', '/model/my_robot/cmd_vel'),
        ],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', PathJoinSubstitution([FindPackageShare(pkg), 'config']),
            '-configuration_basename', configuration_basename
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        world_arg, use_sim_time_arg, configuration_basename_arg, rviz_config_arg,
        set_resource_path,
        gz_launch,
        robot_state_publisher,
        spawn_entity,
        lidar_and_cmdvel_bridge,
        cartographer_node,
        occupancy_grid_node,
        rviz2,
    ])
