# jackal_world.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    front_laser = LaunchConfiguration('front_laser')
    joystick = LaunchConfiguration('joystick')

    config = LaunchConfiguration('config')

    # Paths
    jackal_gazebo_dir = get_package_share_directory('jackal_gazebo')
    gazebo_ros_dir = get_package_share_directory('ros_gz_sim')

    # World file path
    world_name = LaunchConfiguration('world_name')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'world_name',
            default_value=[jackal_gazebo_dir, '/worlds/jackal_race.world']
        ),
        DeclareLaunchArgument('front_laser', default_value='false'),
        DeclareLaunchArgument(
            'default_config',
            default_value='front_laser',
            condition=IfCondition(front_laser)
        ),
        DeclareLaunchArgument(
            'default_config',
            default_value='base',
            condition=UnlessCondition(front_laser)
        ),
        DeclareLaunchArgument('config', default_value=LaunchConfiguration('default_config')),
        DeclareLaunchArgument('joystick', default_value='true'),

        # Include Gazebo empty_world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                gazebo_ros_dir, '/launch/gz_sim.launch.py'
            ]),
            launch_arguments={
                'debug': '0',
                'gui': gui,
                'use_sim_time': use_sim_time,
                'headless': headless,
                'world': world_name,
            }.items()
        ),

        # Include spawn_jackal.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                jackal_gazebo_dir, '/launch/jackal_spawn.launch.py'
            ]),
            launch_arguments={
                'x': '0',
                'y': '0',
                'z': '1.0',
                'yaw': '0',
                'config': config,
                'joystick': joystick,
            }.items()
        ),
    ])
