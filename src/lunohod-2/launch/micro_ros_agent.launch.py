#!/usr/bin/env python3

"""
micro-ROS Agent Launch File for Lunohod-2

Establishes serial communication between ROS2 and micro-ROS firmware on ESP32.
The agent bridges micro-ROS topics from the chassis to standard ROS2 topics.

Expected topics from chassis:
- /ugv/odom (nav_msgs/Odometry) - Wheel odometry at 50 Hz
- /ugv/imu (sensor_msgs/Imu) - IMU data at 50 Hz
- /ugv/encoder (sensor_msgs/JointState) - Encoder ticks at 50 Hz

Expected topics to chassis:
- /ugv/cmd_vel (geometry_msgs/Twist) - Velocity commands
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare launch arguments
    microros_device_arg = DeclareLaunchArgument(
        'microros_device',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_02640c7312daef11ad31593dc8728757-if00-port0',
        description='Serial device path for micro-ROS connection'
    )

    microros_baud_arg = DeclareLaunchArgument(
        'microros_baud',
        default_value='2000000',
        description='Baud rate for serial communication (default: 2000000)'
    )

    # Get launch configuration values
    microros_device = LaunchConfiguration('microros_device')
    microros_baud = LaunchConfiguration('microros_baud')

    # micro-ROS Agent Node
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', microros_device, '-b', microros_baud],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        microros_device_arg,
        microros_baud_arg,
        micro_ros_agent_node
    ])
