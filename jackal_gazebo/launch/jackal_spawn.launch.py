# jackal_spawn.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_prefix, get_package_share_directory



def generate_launch_description():
    # Launch configuration variables
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    joystick = LaunchConfiguration('joystick')
    config = LaunchConfiguration('config')

    # Find package directories
    jackal_description_share = get_package_share_directory('jackal_description')
    jackal_control_share = get_package_share_directory('jackal_control')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='1'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('joystick', default_value='true'),
        DeclareLaunchArgument('config', default_value='base'),

        # Include the Jackal description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(jackal_description_share, 'launch/description.launch.py')
            ),
            launch_arguments={'config': config}.items()
        ),

        # Include the control launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(jackal_control_share, 'launch/control.launch.py')
            ),
        ),

        # Include the teleop launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(jackal_control_share, 'launch/teleop.launch.py')
            ),
            launch_arguments={'joystick': joystick}.items()
        ),

        # Spawn Jackal using the 'spawn_model' node
        Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-urdf',
                '-model', 'jackal',
                '-param', 'robot_description',
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', '0',
                '-P', '0',
                '-Y', yaw
            ]
        ),
    ])
