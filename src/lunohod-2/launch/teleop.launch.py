#!/usr/bin/env python3

"""
Lunohod-2 Joystick Teleoperation Launch File

Provides joystick control using teleop_twist_joy with Xbox controller.

Note: For keyboard control, run directly in terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ugv/cmd_vel

Usage:
    ros2 launch lunohod-2 teleop.launch.py
    ros2 launch lunohod-2 teleop.launch.py joy_device:=/dev/input/js1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Get package directory
    pkg_lunohod2 = get_package_share_directory('lunohod-2')

    # Launch arguments
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/ugv/cmd_vel',
        description='Velocity command topic'
    )

    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    # Get launch configurations
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    joy_device = LaunchConfiguration('joy_device')

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': joy_device,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # Joystick teleop node
    teleop_config = os.path.join(pkg_lunohod2, 'config', 'teleop.yaml')
    joystick_teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_joystick',
        parameters=[teleop_config],
        remappings=[
            ('cmd_vel', cmd_vel_topic),
        ],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        cmd_vel_topic_arg,
        joy_device_arg,

        # Nodes
        joy_node,
        joystick_teleop_node,
    ])
