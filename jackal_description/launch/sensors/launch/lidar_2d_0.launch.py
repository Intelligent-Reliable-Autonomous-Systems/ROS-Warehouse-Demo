"""
Lidar2D launch file for Jackal Robot

Written by Will Solow, 2025
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os 

def generate_launch_description():

    pkg_jackal_description = get_package_share_directory("jackal_description")


    # Nodes
    node_lidar_2d_0_gz_bridge = Node(
        name='lidar_2d_0_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='j100_0000/sensors/',
        output='screen',
        parameters=[
                {'use_sim_time': True,
                 'config_file': os.path.join(pkg_jackal_description, "launch/sensors/config/lidar_2d_0.yaml"),
                },
            ],
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(node_lidar_2d_0_gz_bridge)
    return ld
