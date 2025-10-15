#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg = 'slam_toolbox_bringup'

    # ---- Launch args ----
    use_sim_time     = LaunchConfiguration('use_sim_time')
    start_gazebo     = LaunchConfiguration('start_gazebo')
    start_rviz       = LaunchConfiguration('start_rviz')
    world_file       = LaunchConfiguration('world')        # filename in worlds/
    extra_modelpath  = LaunchConfiguration('model_path')   # optional extra model paths
    gui              = LaunchConfiguration('gui')          # start gzclient?
    force_sw_gl      = LaunchConfiguration('force_software_gl')

    declare_use_sim_time  = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_start_gazebo  = DeclareLaunchArgument('start_gazebo', default_value='true')
    declare_start_rviz    = DeclareLaunchArgument('start_rviz', default_value='true')
    declare_world         = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='obstacles.world'),  # <-- your world by default
        description='SDF world under share/<pkg>/worlds/'
    )
    declare_model_path    = DeclareLaunchArgument(
        'model_path',
        default_value=TextSubstitution(text=''),
        description='Extra GAZEBO_MODEL_PATH entries (colon-separated).'
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Start Gazebo GUI (gzclient). Set true to show window.'
    )
    declare_force_sw_gl = DeclareLaunchArgument(
        'force_software_gl', default_value='false',
        description='Force software OpenGL (LIBGL_ALWAYS_SOFTWARE=1).'
    )

    # ---- Paths in your package ----
    share = get_package_share_directory(pkg)

    # xacro -> /robot_description
    xacro_path = os.path.join(share, 'description', 'my_robot.xacro')
    robot_description_xml = xacro.process_file(xacro_path).toxml()

    # RViz & SLAM params
    rviz_config  = os.path.join(share, 'config', 'view_robot.rviz')
    slam_params  = os.path.join(share, 'config', 'mapper_parameters_online_async.yaml')

    # ---- Env: ensure Gazebo can find models you ship (plus user-specified paths) ----
    # Final GAZEBO_MODEL_PATH = <user extra> : <pkg>/description
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[extra_modelpath, TextSubstitution(text=':'), PathJoinSubstitution([FindPackageShare(pkg), 'description'])]
    )

    # Prefer XCB over Wayland (helps GUI stability)
    set_qt_xcb = SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb')

    # Conditionally force software GL (helps on some laptops)
    force_software_gl_env = GroupAction(
        condition=IfCondition(force_sw_gl),
        actions=[SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')]
    )

    # ---- Robot State Publisher ----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_xml,
                     'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ---- Gazebo world include (Classic) ----
    world_path = PathJoinSubstitution([FindPackageShare(pkg), 'worlds', world_file])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        condition=IfCondition(start_gazebo),
        launch_arguments={
            'world': world_path,
            'gui': gui,
            'verbose': 'true'
        }.items()
    )

    # Spawn entity after /robot_description is ready (slightly longer delay)
    spawn_entity = TimerAction(
        period=3.0,
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

    # ---- SLAM Toolbox ----
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ---- RViz2 (toggleable) ----
    rviz2 = Node(
        condition=IfCondition(start_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_start_gazebo,
        declare_start_rviz,
        declare_world,
        declare_model_path,
        declare_gui,
        declare_force_sw_gl,
        set_qt_xcb,
        force_software_gl_env,
        set_gazebo_model_path,
        rsp,
        gazebo_launch,
        spawn_entity,
        slam,
        rviz2
    ])
