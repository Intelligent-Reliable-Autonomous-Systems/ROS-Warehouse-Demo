from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            namespace='table_camera',
            parameters=[{
                'image_transport': 'raw',
                'family': '36h11',
                'size': 0.1,
                'max_hamming': 0,
                'detector': {
                    'threads': 1,
                    'decimate': 2.0,
                    'blur': 0.0,
                    'refine': True,
                    'sharpening': 0.25,
                    'debug': False
                }
            }],
            remappings=[
                ('image_rect', '/table_camera/image_raw'),
                ('camera_info', '/table_camera/camera_info'),
            ],
        ),

        Node(
            package='camera_detection',
            executable='camera_tf',
            name='camera_tf',
            namespace='table_camera',
        )
    ]) 