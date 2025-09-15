import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    video_device = LaunchConfiguration('video_device')
    camera_frame_id = LaunchConfiguration('camera_frame_id')
    fps = LaunchConfiguration('fps')

    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
            
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Video device path for USB camera'),

        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_link_optical',
            description='Camera frame ID'),

        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='Frames per second'),

        # USB Camera Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'video_device': video_device,
                'image_size': [320, 240], # Fixed: Use integer array directly
                'camera_frame_id': camera_frame_id,
                'fps': 10,
                'pixel_format': 'YUYV',  # Common format for USB cameras
                'camera_info_url': 'file://' + os.path.join(get_package_share_directory('lunohod-1'), 'config', 'camera_info.yaml')
                }],
            remappings=[
                ('image_raw', 'camera/image_raw'),
                ('camera_info', 'camera/camera_info'),
            ]
        )
    ])