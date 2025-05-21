from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages

    # Declare launch files
    launch_file_gps_1 = '/home/will-solow/clearpath/sensors/launch/gps_1.launch.py'

    # Include launch files
    launch_gps_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_1]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_gps_1)
    return ld
