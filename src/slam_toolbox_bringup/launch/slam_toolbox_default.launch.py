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
    # This package (where this launch file lives)
    pkg = 'slam_toolbox_bringup'

    # ---- Launch args ----
    use_sim_time    = LaunchConfiguration('use_sim_time')
    start_gazebo    = LaunchConfiguration('start_gazebo')
    start_rviz      = LaunchConfiguration('start_rviz')
    gui             = LaunchConfiguration('gui')                # Gazebo GUI?
    force_sw_gl     = LaunchConfiguration('force_software_gl')  # LIBGL_ALWAYS_SOFTWARE=1
    tb3_model       = LaunchConfiguration('tb3_model')          # burger | waffle | waffle_pi
    extra_modelpath = LaunchConfiguration('model_path')         # extra GAZEBO_MODEL_PATH entries

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_start_gazebo = DeclareLaunchArgument('start_gazebo', default_value='true')
    declare_start_rviz   = DeclareLaunchArgument('start_rviz', default_value='true')
    declare_gui          = DeclareLaunchArgument('gui', default_value='false',
                              description='Start Gazebo GUI (gzclient) if true.')
    declare_force_sw_gl  = DeclareLaunchArgument('force_software_gl', default_value='false',
                              description='Force software OpenGL for stability.')
    declare_tb3_model    = DeclareLaunchArgument('tb3_model', default_value='burger',
                              description='TurtleBot3 model: burger | waffle | waffle_pi')
    declare_model_path   = DeclareLaunchArgument('model_path', default_value='',
                              description='Extra colon-separated paths for GAZEBO_MODEL_PATH')

    # ---- Package shares ----
    tb3_desc_share  = get_package_share_directory('turtlebot3_description')
    tb3_gz_share    = get_package_share_directory('turtlebot3_gazebo')
    this_share      = get_package_share_directory(pkg)

    # ---- Robot description (xacro) from TurtleBot3 ----
    # e.g. turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro
    tb3_xacro = PathJoinSubstitution([
        FindPackageShare('turtlebot3_description'),
        'urdf',
        ['turtlebot3_', tb3_model, '.urdf.xacro']
    ])

    # Process the xacro into XML now (evaluates LaunchConfiguration at runtime)
    # To pass LaunchConfiguration into xacro, evaluate path at runtime:
    def _xacro_to_xml(context):
        # Resolve xacro path string
        xacro_path = os.path.join(
            tb3_desc_share, 'urdf', f"turtlebot3_{context.perform_substitution(tb3_model)}.urdf.xacro"
        )
        # TURTLEBOT3_MODEL is used by many TB3 tools; export it for consistency
        os.environ['TURTLEBOT3_MODEL'] = context.perform_substitution(tb3_model)
        return xacro.process_file(xacro_path).toxml()

    # Create a parameter substitution for robot_description that evaluates at launch time
    class RobotDescription:
        def perform(self, context):
            return _xacro_to_xml(context)

    # ---- RViz & SLAM params (use your existing files) ----
    rviz_config = os.path.join(this_share, 'config', 'view_robot.rviz')
    slam_params = os.path.join(this_share, 'config', 'mapper_parameters_online_async.yaml')

    # ---- Gazebo model path (TB3 models + your package + user extras) ----
    # Final: <user extras> : <tb3_gazebo>/models : <this_pkg>/description
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            extra_modelpath,
            TextSubstitution(text=':'),
            PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'models']),
            TextSubstitution(text=':'),
            PathJoinSubstitution([FindPackageShare(pkg), 'description']),
        ]
    )

    # Prefer XCB over Wayland (GUI stability)
    set_qt_xcb = SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb')

    # Optionally force software GL (useful on some laptops)
    force_software_gl_env = GroupAction(
        condition=IfCondition(force_sw_gl),
        actions=[SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')]
    )

    # ---- Robot State Publisher ----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': RobotDescription(),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # ---- Gazebo Classic world include: turtlebot3_world.world ----
    world_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'), 'worlds', 'turtlebot3_world.world'
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        condition=IfCondition(start_gazebo),
        launch_arguments={
            'world': world_path,
            'gui':   gui,
            'verbose': 'true'
        }.items()
    )

    # ---- Spawn the TB3 after /robot_description is ready ----
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                condition=IfCondition(start_gazebo),
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'turtlebot3'],
                output='screen'
            )
        ]
    )

    # ---- SLAM Toolbox (sync) ----
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {
            'use_sim_time': use_sim_time,
            # Make sure these match TB3 + your topics
            'scan_topic': '/scan',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
        }],
        output='screen'
    )

    # ---- RViz2 (toggle) ----
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
        declare_gui,
        declare_force_sw_gl,
        declare_tb3_model,
        declare_model_path,
        set_qt_xcb,
        force_software_gl_env,
        set_gazebo_model_path,
        rsp,
        gazebo_launch,
        spawn_entity,
        slam,
        rviz2
    ])
