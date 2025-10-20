#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg = 'cartographer_bringup'

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_gazebo = LaunchConfiguration('start_gazebo')
    start_rviz   = LaunchConfiguration('start_rviz')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_start_gazebo = DeclareLaunchArgument('start_gazebo', default_value='false')
    declare_start_rviz   = DeclareLaunchArgument('start_rviz', default_value='true')

    share = get_package_share_directory(pkg)

    # If you want to spawn your robot here, point to your xacro
    # (or comment this block out if the robot is already running elsewhere)
    # Example using your slam_toolbox_bringup’s my_robot.xacro:
    try:
        st_share = get_package_share_directory('slam_toolbox_bringup')
        xacro_path = os.path.join(st_share, 'description', 'my_robot.xacro')
        robot_description_xml = xacro.process_file(xacro_path).toxml()
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_xml, 'use_sim_time': use_sim_time}],
            output='screen'
        )
        use_rsp = True
    except Exception:
        rsp = None
        use_rsp = False

    # Gazebo (optional)
    world_path = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'worlds', 'empty.world'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        condition=IfCondition(start_gazebo),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                condition=IfCondition(start_gazebo),
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'carto_bot'],
                output='screen'
            )
        ]
    )

    # Cartographer nodes
    lua_file = os.path.join(share, 'config', 'cartographer_2d.lua')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', os.path.join(share, 'config'),
                   '-configuration_basename', 'cartographer_2d.lua'],
        remappings=[('/scan', '/scan')]  # change if your scan topic differs
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'resolution': 0.05, 'publish_period_sec': 1.0}]
    )

    # RViz (optional)
    rviz_config = os.path.join(share, 'config', 'rviz_cartographer.rviz')
    rviz = Node(
        condition=IfCondition(start_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )

    nodes = [
        declare_use_sim_time, declare_start_gazebo, declare_start_rviz,
        gazebo, spawn_entity, cartographer_node, occupancy_grid_node, rviz
    ]
    if use_rsp and rsp is not None:
        nodes.insert(3, rsp)

    return LaunchDescription(nodes)

