import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
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
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    sim_gazebo = LaunchConfiguration("sim_gazebo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_arm = LaunchConfiguration("spawn_arm")

    pkg_warehouseworld = get_package_share_directory('warehouse_world')
    pkg_warehouse_setup = get_package_share_directory('warehouse_setup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_warehouseworld, 'models'])
    )
    
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            
            "gz_args": f" -r -v 3 warehouse_world/worlds/warehouse2.sdf --physics-engine gz-physics-bullet-featherstone-plugin",
            "on_exit_shutdown": "True"
        }.items(),
        condition=IfCondition(sim_gazebo),
    )

    # Kinova Arm Launch Description
    kinova_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_warehouse_setup, 'launch', 'kortex_sim_control.launch.py'])),
        launch_arguments={}.items(),
        condition=IfCondition(spawn_arm),
    )

    nodes_to_launch = [
        gz_resource_path,
        gz_launch_description,
        kinova_arm_launch
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
            default_value="warehouse2.sdf",
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

