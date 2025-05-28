from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    video_device = LaunchConfiguration('video_device', default='/dev/video0')
    camera_name = LaunchConfiguration('camera_name', default='anker_c200')
    image_width = LaunchConfiguration('image_width', default='640')
    image_height = LaunchConfiguration('image_height', default='480')
    pixel_format = LaunchConfiguration('pixel_format', default='yuyv')
    frame_id = LaunchConfiguration('frame_id', default='camera_optical_frame')

    return LaunchDescription([
        # Declare all the launch arguments
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Video device path'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='anker_c200',
            description='Camera name'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Image width'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Image height'
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='yuyv',
            description='Pixel format'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_optical_frame',
            description='Camera frame id'
        ),

        # Launch the USB camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': video_device,
                'camera_name': camera_name,
                'image_width': image_width,
                'image_height': image_height,
                'pixel_format': pixel_format,
                'frame_id': frame_id,
                'camera_info_url': 'package://manipulation_camera/config/camera_info.yaml'
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
        ),

        # Launch image_proc node for image processing
        Node(
            package='image_proc',
            executable='rectify_node',
            name='image_proc',
            namespace=camera_name,
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
        ),
    ]) 