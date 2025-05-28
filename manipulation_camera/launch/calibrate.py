from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Camera Launch Arguments (from camera.py)
    video_device = LaunchConfiguration('video_device', default='/dev/video0')
    camera_name = LaunchConfiguration('camera_name', default='anker_c200')
    image_width = LaunchConfiguration('image_width', default='640')
    image_height = LaunchConfiguration('image_height', default='480')
    pixel_format = LaunchConfiguration('pixel_format', default='yuyv')
    frame_id = LaunchConfiguration('frame_id', default='camera_optical_frame')
    camera_info_url = LaunchConfiguration('camera_info_url', default='package://manipulation_camera/config/camera_info.yaml')

    # Calibration specific launch arguments
    checkerboard_size = LaunchConfiguration('checkerboard_size', default='8x6')
    square_size = LaunchConfiguration('square_size', default='0.025') # in meters

    return LaunchDescription([
        # Declare camera launch arguments
        DeclareLaunchArgument('video_device', default_value='/dev/video0', description='Video device path'),
        DeclareLaunchArgument('camera_name', default_value='anker_c200', description='Camera name for namespace and camera_info file'),
        DeclareLaunchArgument('image_width', default_value='640', description='Image width'),
        DeclareLaunchArgument('image_height', default_value='480', description='Image height'),
        DeclareLaunchArgument('pixel_format', default_value='yuyv', description='Pixel format (yuyv, mjpeg2rgb, etc.)'),
        DeclareLaunchArgument('frame_id', default_value='camera_optical_frame', description='Camera frame_id'),
        DeclareLaunchArgument('camera_info_url', default_value='package://manipulation_camera/config/camera_info.yaml', description='URL to camera calibration file'),

        # Declare calibration specific arguments
        DeclareLaunchArgument('checkerboard_size', default_value='8x6', description='Number of inner corners on the checkerboard (e.g., 8x6 for a 9x7 board)'),
        DeclareLaunchArgument('square_size', default_value='0.025', description='Size of a square on the checkerboard in meters'),

        # Launch the USB camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera',
            parameters=[{
                'video_device': video_device,
                'camera_name': camera_name,
                'image_width': image_width,
                'image_height': image_height,
                'pixel_format': pixel_format,
                'frame_id': frame_id,
                'camera_info_url': camera_info_url
            }],
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('~/set_camera_info', 'set_camera_info'),
            ],
        ),

        # Launch the camera calibrator node
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='cameracalibrator',
            parameters=[{
                'size': checkerboard_size,
                'square': square_size,
            }],
            arguments=[
                '--size', checkerboard_size,
                '--square', square_size,
            ],
            remappings=[
                ('image',  '/camera/image_raw'),
            ],
            output='screen'
        )
    ])
