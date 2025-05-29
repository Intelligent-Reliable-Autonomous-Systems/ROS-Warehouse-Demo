# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
]

def launch_setup(context, *args, **kwargs):
    pkg_jackal_description = get_package_share_directory('iras_jackal_description')

    world = LaunchConfiguration('world')

    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(os.path.join(
        pkg_jackal_description, 'launch/robot.yaml'))

    namespace = clearpath_config.system.namespace
    if namespace in ('', '/'):
        robot_name = 'robot'
    else:
        robot_name = namespace + '/robot'
    

    # Paths
    launch_file_platform_service = PathJoinSubstitution([
        pkg_jackal_description, 'launch/platform/launch', 'platform-service.launch.py'])
    launch_file_sensors_service = PathJoinSubstitution([
        pkg_jackal_description, 'launch/sensors/launch', 'sensors-service.launch.py'])

    group_action_spawn_robot = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_platform_service]),
            launch_arguments=[
              ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_sensors_service]),
            launch_arguments=[
              ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])]
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=['-name', robot_name,
                       '-x', "0.0",
                       '-y', "0.0",
                       '-z', "0.1",
                       '-Y', "0.0",
                       '-topic', 'robot_description'],
            output='screen'
        ),


    ])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("iras_jackal_description"), "launch", "robot.urdf.xacro"]
            ),

        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=f"{namespace}/full_description",
        parameters=[robot_description],

    )

    tf_relay_node = Node(
        name='j100_0000_relay_node',
        executable='tf_relay',
        package='warehouse_sim',
        namespace='',
        output='screen',
        parameters=[{'namespace': 'j100_0000'}],
    )

    actions = [group_action_spawn_robot,
               tf_relay_node,
               robot_state_publisher, #NOTE this causes the controller manager to fail silently :(
               ]

    return actions


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
