import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    sim_gazebo = LaunchConfiguration("sim_gazebo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_arm = LaunchConfiguration("spawn_arm")
    spawn_jackal = LaunchConfiguration("spawn_jackal")

    pkg_warehouseworld = get_package_share_directory('warehouse_world')
    pkg_warehouse_setup = get_package_share_directory('warehouse_setup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    
    warehouse_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_warehouseworld, 'models'])
    )

    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]

    # Set ignition resource path to include all sourced ros packages
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_clearpath_gz, 'worlds') + ':',
            os.path.join(pkg_clearpath_gz, 'meshes') + ':',
            ':' + ':'.join(packages_paths)])
    
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            
            "gz_args": f" -r -v 4 warehouse_world/worlds/warehouse.sdf",
            "on_exit_shutdown": "True"
        }.items(),
        condition=IfCondition(sim_gazebo),
    )

    # Kinova Arm Launch Description
    kinova_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_warehouse_setup, 'launch', 'spawn_kinova.launch.py'])),
        launch_arguments={}.items(),
        condition=IfCondition(spawn_arm),
    )

    # Jackal Launch Description
    jackal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [pkg_warehouse_setup, 'launch', 'spawn_jackal.launch.py'])),
        launch_arguments={
            "world": "warehouse",
        }.items(),
        condition=IfCondition(spawn_jackal)
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    table_camera_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='table_camera_bridge',
            output='screen',
            arguments=[
                '/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/world/warehouse/model/table_camera/link/camera_link/sensor/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            remappings=[
                ('/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/depth_image', '/table_camera/depth/image_raw'),
                ('/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/camera_info', '/table_camera/camera_info'),
                ('/world/warehouse/model/table_camera/link/camera_link/sensor/rgb_camera/image', '/table_camera/image/image_raw'),
                ('/world/warehouse/model/table_camera/link/camera_link/sensor/rgbd_camera/depth_image/points', '/table_camera/depth/points'),
            ]
        )
    
    world_camera_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='world_camera_bridge',
            output='screen',
            arguments=[
                '/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/world/warehouse/model/world_camera/link/camera_link/sensor/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            remappings=[
                ('/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/depth_image', '/world_camera/depth/image_raw'),
                ('/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/camera_info', '/world_camera/camera_info'),
                ('/world/warehouse/model/world_camera/link/camera_link/sensor/rgb_camera/image', '/world_camera/image/image_raw'),
                ('/world/warehouse/model/world_camera/link/camera_link/sensor/rgbd_camera/depth_image/points', '/world_camera/depth/points'),
            ]
        )

    nodes_to_launch = [
        gz_sim_resource_path,
        warehouse_resource_path,
        gz_launch_description,
        kinova_arm_launch,
        jackal_launch,
        clock_bridge,
        table_camera_bridge,
        world_camera_bridge,
    ]

    return nodes_to_launch

def generate_launch_description():
   

    declared_arguments = []

    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Use Gazebo for simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "worlds_file",
            default_value="warehouse.sdf",
            description="World to launch",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_arm",
            default_value="true",
            description="Spawn the Kinova Arm",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_jackal",
            default_value="false",
            description="Spawn the Jackal",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

