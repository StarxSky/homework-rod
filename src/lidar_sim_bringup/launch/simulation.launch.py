"""Bring up Gazebo, the lidar robot, the bridges, detection and RViz."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('lidar_robot_description')
    bringup_share = get_package_share_directory('lidar_sim_bringup')
    detector_share = get_package_share_directory('obstacle_detector')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(
        description_share, 'urdf', 'lidar_robot.urdf.xacro')
    world_file = os.path.join(
        bringup_share, 'worlds', 'obstacle_arena.sdf')
    bridge_config = os.path.join(
        bringup_share, 'config', 'gz_bridge.yaml')
    rviz_config = os.path.join(
        bringup_share, 'rviz', 'obstacle_detection.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    gz_args = LaunchConfiguration('gz_args')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'lidar_robot',
                    '-topic', 'robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.10',
                ],
            ),
        ],
    )

    # Clock, cmd_vel and odometry bridges.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    # Dedicated LaserScan bridge so the frame id matches the URDF tree.
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        parameters=[{
            'override_frame_id': 'laser_link',
            'use_sim_time': use_sim_time,
        }],
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detector_share, 'launch', 'detector.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo simulation clock.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz.'),
        DeclareLaunchArgument(
            'gz_args', default_value='-r -v 3 ' + world_file,
            description='Arguments passed to Gazebo Sim.'),

        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        scan_bridge,
        detector,
        rviz,
    ])
