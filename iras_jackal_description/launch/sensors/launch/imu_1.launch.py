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

    launch_arg_prefix = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='')

    prefix = LaunchConfiguration('prefix')

    # Nodes
    node_imu_1_gz_bridge = Node(
        name='imu_1_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='j100_0000/',
        output='screen',
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                    'config_file': os.path.join(pkg_jackal_description, 'launch/sensors/config/imu_1.yaml')
                    ,
                }
                ,
            ]
        ,
    )

    node_imu_1_static_tf = Node(
        name='imu_1_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        namespace='j100_0000',
        output='screen',
        arguments=
            [
                '--frame-id'
                ,
                'imu_1_link'
                ,
                '--child-frame-id'
                ,
                'j100_0000/robot/base_link/imu_1'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
                (
                    '/tf_static'
                    ,
                    'tf_static'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_prefix)
    ld.add_action(node_imu_1_gz_bridge)
    ld.add_action(node_imu_1_static_tf)
    return ld
