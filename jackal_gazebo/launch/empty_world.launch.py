# jackal_world.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    world_name = LaunchConfiguration('world_name')
    front_laser = LaunchConfiguration('front_laser')
    default_config = LaunchConfiguration('default_config')
    config = LaunchConfiguration('config')
    joystick = LaunchConfiguration('joystick')

    # Find package directories
    gazebo_ros_share = FindPackageShare('gazebo_ros')
    jackal_gazebo_share = FindPackageShare('jackal_gazebo')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'world_name',
            default_value=[gazebo_ros_share, '/worlds/empty_world.world']
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
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('joystick', default_value='true'),

        # Include the Gazebo world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [gazebo_ros_share, '/launch/gazebo.launch.py']
            ),
            launch_arguments={
                'debug': '0',
                'gui': gui,
                'use_sim_time': use_sim_time,
                'headless': headless,
                'world': world_name
            }.items()
        ),

        # Include the Jackal spawn launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [jackal_gazebo_share, '/launch/spawn_jackal.launch.py']
            ),
            launch_arguments={
                'x': '0',
                'y': '0',
                'z': '1.0',
                'yaw': '0',
                'config': config,
                'joystick': joystick
            }.items()
        ),
    ])
