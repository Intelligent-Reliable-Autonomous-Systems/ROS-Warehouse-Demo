from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os

def generate_launch_description():
    pkg_jackal_description = get_package_share_directory("iras_jackal_description")
    # Include Packages

    # Declare launch files
    launch_file_lidar2d_0 = os.path.join(pkg_jackal_description, 'launch/sensors/launch/lidar2d_0.launch.py')
    launch_file_lidar2d_1 = os.path.join(pkg_jackal_description, 'launch/sensors/launch/lidar2d_1.launch.py')
    launch_file_lidar3d_0 = os.path.join(pkg_jackal_description, 'launch/sensors/launch/lidar3d_0.launch.py')
    launch_file_camera_0 = os.path.join(pkg_jackal_description, 'launch/sensors/launch/camera_0.launch.py')
    launch_file_imu_1 = os.path.join(pkg_jackal_description, 'launch/sensors/launch/imu_1.launch.py')
    launch_file_ins_0 = os.path.join(pkg_jackal_description, 'launch/sensors/launch/ins_0.launch.py')

    # Include launch files
    launch_lidar2d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_0]),
    )

    launch_lidar2d_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar2d_1]),
    )

    launch_lidar3d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar3d_0]),
    )

    launch_camera_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_0]),
    )

    launch_imu_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_1]),
    )

    launch_ins_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_ins_0]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_lidar2d_0)
    ld.add_action(launch_lidar2d_1)
    ld.add_action(launch_lidar3d_0)
    ld.add_action(launch_camera_0)
    ld.add_action(launch_imu_1)
    ld.add_action(launch_ins_0)
    return ld
