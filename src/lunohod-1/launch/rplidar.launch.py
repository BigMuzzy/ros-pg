import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    return LaunchDescription([

Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate,
                           'scan_mode': scan_mode
                         }],
            output='screen')
    ])
