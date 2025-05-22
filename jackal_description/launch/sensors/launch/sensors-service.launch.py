from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os 

def generate_launch_description():

    # Include Packages
    pkg_jackal_description = get_package_share_directory("jackal_description")

    # Declare launch files
    launch_file_gps_1 = '/home/will-solow/clearpath/sensors/launch/gps_1.launch.py'
    launch_file_gps_1 = os.path.join(pkg_jackal_description, "launch/sensors/launch/gps_1.launch.py")

    # Include launch files
    launch_gps_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_1]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    #ld.add_action(launch_gps_1)
    return ld
