import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition

def generate_launch_description():

    channel_type = LaunchConfiguration('channel_type', default='serial')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    auto_reconnect = LaunchConfiguration('auto_reconnect', default='true')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'channel_type',
            default_value='serial'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='460800'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser_frame'
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Standard'
        ),
        DeclareLaunchArgument(
            'auto_reconnect',
            default_value='true'
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': lidar_port, 
                'serial_baudrate': serial_baudrate, 
                'frame_id': frame_id,
                'inverted': inverted, 
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
                'auto_reconnect': auto_reconnect,
                # Add timeout and retry parameters
                'timeout': 5.0,
                'retry_delay': 2.0
            }],
            output='screen',
            respawn=True,  # Automatically restart if node dies
            respawn_delay=3.0  # Wait 3 seconds before restart
        ),
        
        # Optional: Add a monitoring node
        Node(
            package='topic_tools',
            executable='relay',
            name='scan_monitor',
            arguments=['/scan', '/scan_backup'],
            condition=IfCondition(auto_reconnect),
            respawn=True
        )
    ])