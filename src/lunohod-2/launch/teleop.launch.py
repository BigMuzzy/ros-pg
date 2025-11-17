#!/usr/bin/env python3

"""
Lunohod-2 Teleoperation Launch File

Provides two teleoperation modes:
1. Keyboard (default) - Uses teleop_twist_keyboard
2. Joystick - Uses teleop_twist_joy with Xbox controller

Usage:
    # Keyboard mode (default)
    ros2 launch lunohod-2 teleop.launch.py

    # Joystick mode
    ros2 launch lunohod-2 teleop.launch.py teleop_mode:=joystick

    # Custom velocity limits
    ros2 launch lunohod-2 teleop.launch.py max_linear:=0.7 max_angular:=1.5
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # Get package directory
    pkg_lunohod2 = get_package_share_directory('lunohod-2')

    # Launch arguments
    teleop_mode_arg = DeclareLaunchArgument(
        'teleop_mode',
        default_value='keyboard',
        description='Teleoperation mode: keyboard or joystick'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/ugv/cmd_vel',
        description='Velocity command topic'
    )

    max_linear_arg = DeclareLaunchArgument(
        'max_linear',
        default_value='0.5',
        description='Maximum linear velocity (m/s)'
    )

    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='1.0',
        description='Maximum angular velocity (rad/s)'
    )

    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    # Get launch configurations
    teleop_mode = LaunchConfiguration('teleop_mode')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')
    joy_device = LaunchConfiguration('joy_device')

    # Keyboard teleop condition
    use_keyboard = PythonExpression([
        "'", teleop_mode, "' == 'keyboard'"
    ])

    # Joystick teleop condition
    use_joystick = PythonExpression([
        "'", teleop_mode, "' == 'joystick'"
    ])

    # Keyboard teleop node
    keyboard_teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal
        remappings=[
            ('cmd_vel', cmd_vel_topic),
        ],
        condition=IfCondition(use_keyboard)
    )

    # Joy node (for joystick)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': joy_device,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen',
        condition=IfCondition(use_joystick)
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
        output='screen',
        condition=IfCondition(use_joystick)
    )

    return LaunchDescription([
        # Arguments
        teleop_mode_arg,
        cmd_vel_topic_arg,
        max_linear_arg,
        max_angular_arg,
        joy_device_arg,

        # Nodes
        keyboard_teleop_node,
        joy_node,
        joystick_teleop_node,
    ])
