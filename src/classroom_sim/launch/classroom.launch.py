"""Bring up the Blender-built classroom, the small car and obstacle detection."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prepend_path(variable, path):
    current = os.environ.get(variable, '')
    return path + (':' + current if current else '')


def generate_launch_description():
    package_share = get_package_share_directory('classroom_sim')
    detector_share = get_package_share_directory('obstacle_detector')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    models_dir = os.path.join(package_share, 'models')
    xacro_file = os.path.join(package_share, 'urdf', 'classroom_car.urdf.xacro')
    world_file = os.path.join(package_share, 'worlds', 'classroom.sdf')
    bridge_config = os.path.join(package_share, 'config', 'gz_bridge.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'classroom.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gz_gui = LaunchConfiguration('use_gz_gui')
    gz_args = LaunchConfiguration('gz_args')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    # Let Gazebo resolve model://classroom and its meshes.  Both names are
    # set because Humble ships Fortress (IGN_*) while newer releases use GZ_*.
    resource_env = [
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            _prepend_path('IGN_GAZEBO_RESOURCE_PATH', models_dir)),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            _prepend_path('GZ_SIM_RESOURCE_PATH', models_dir)),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # The combined `ign gazebo world.sdf` launcher can deadlock under Xvfb, so
    # the server always runs headless (-s) and the GUI is started separately.
    gz_gui = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['ign', 'gazebo', '-g', '-v', '3'],
                output='screen',
                condition=IfCondition(use_gz_gui),
            ),
        ],
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

    # Publish default joint states so the continuous wheel joints get TF and
    # RViz's RobotModel display can render every link.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_car = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'classroom_car',
                    '-topic', 'robot_description',
                    '-x', '2.9', '-y', '2.55', '-z', '0.09',
                    '-Y', '3.14159',
                ],
            ),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

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
            'use_gz_gui', default_value='true',
            description='Open the Gazebo GUI (separate from the server).'),
        DeclareLaunchArgument(
            'gz_args', default_value='-r -s -v 3 ' + world_file,
            description='Arguments passed to Gazebo Sim (server only by '
                        'default; the GUI is launched separately).'),

        *resource_env,
        gz_sim,
        gz_gui,
        robot_state_publisher,
        joint_state_publisher,
        spawn_car,
        bridge,
        scan_bridge,
        detector,
        rviz,
    ])
