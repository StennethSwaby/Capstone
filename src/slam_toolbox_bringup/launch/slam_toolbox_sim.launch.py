#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg = 'slam_toolbox_bringup'

    # ---- Launch args ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_gazebo = LaunchConfiguration('start_gazebo')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_start_gazebo = DeclareLaunchArgument('start_gazebo', default_value='true')  # set false to skip Gazebo

    # ---- Paths in your package ----
    share = get_package_share_directory(pkg)

    # Your xacro lives as "description/my_robot.xacro"
    xacro_path = os.path.join(share, 'description', 'my_robot.xacro')
    robot_description_xml = xacro.process_file(xacro_path).toxml()

    # Your RViz configs live in config/: drive_robot.rviz, view_robot.rviz
    rviz_config = os.path.join(share, 'config', 'view_robot.rviz')

    # Your SLAM params file (you already have this)
    slam_params = os.path.join(share, 'config', 'mapper_parameters_online_async.yaml')

    # ---- Robot State Publisher (from xacro -> /robot_description) ----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_xml,
                     'use_sim_time': use_sim_time}],
        output='screen'
    )

    # (Optional) Gazebo Classic + spawn
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        condition=IfCondition(start_gazebo)
    )

    spawn_entity = TimerAction(  # small delay so /robot_description is ready
        period=2.0,
        actions=[
            Node(
                condition=IfCondition(start_gazebo),
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'capstone_bot'],
                output='screen'
            )
        ]
    )

    # ---- SLAM Toolbox (online/sync) ----
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ---- RViz2 ----
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_start_gazebo,
        rsp,
        gazebo_launch,
        spawn_entity,
        slam,
        rviz2
    ])
