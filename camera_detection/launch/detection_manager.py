#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_detection',
            executable='detection_manager',
            name='detection_manager',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # Set to False for real robot
            }]
        ),
    ]) 