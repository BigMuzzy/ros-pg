#!/usr/bin/env python3

"""
RPLidar C1 Launch File for Lunohod-2

Configures and launches the Slamtec RPLidar C1 2D laser scanner.
Publishes laser scan data to /scan topic for SLAM and navigation.

RPLidar C1 Specifications:
- Range: 0.15m - 12m
- Scan Rate: ~10 Hz
- Angular Resolution: ~0.9°
- Interface: USB (CP2102 serial)
- Baudrate: 460800
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare launch arguments
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f86253bee863ef11a2a1e2a9c169b110-if00-port0',
        description='Serial port for RPLidar C1'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='Serial baudrate for RPLidar C1 (typically 460800)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='TF frame for laser scanner (must match URDF)'
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Scan mode: Standard or Sensitivity'
    )

    # Get launch configuration values
    lidar_port = LaunchConfiguration('lidar_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    scan_mode = LaunchConfiguration('scan_mode')

    # RPLidar Node (from sllidar_ros2 package)
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': scan_mode,
            'auto_reconnect': True,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        lidar_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        scan_mode_arg,
        rplidar_node
    ])
