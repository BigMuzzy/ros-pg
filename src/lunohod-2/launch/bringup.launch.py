#!/usr/bin/env python3

"""
Lunohod-2 Robot Bringup Launch File

This launch file starts all essential components for the robot:
1. Robot State Publisher (URDF/TF)
2. micro-ROS Agent (chassis communication)
3. RPLidar C1 Driver (laser scanner)

Usage:
    ros2 launch lunohod-2 bringup.launch.py

Optional arguments:
    microros_device:=/dev/ttyUSB0
    lidar_port:=/dev/ttyUSB1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Get package directory
    pkg_lunohod2 = get_package_share_directory('lunohod-2')

    # Launch arguments
    microros_device_arg = DeclareLaunchArgument(
        'microros_device',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_02640c7312daef11ad31593dc8728757-if00-port0',
        description='Serial device for micro-ROS chassis'
    )

    microros_baud_arg = DeclareLaunchArgument(
        'microros_baud',
        default_value='2000000',
        description='Baud rate for micro-ROS (default: 2000000)'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f86253bee863ef11a2a1e2a9c169b110-if00-port0',
        description='Serial port for RPLidar C1'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    # Get launch configurations
    microros_device = LaunchConfiguration('microros_device')
    microros_baud = LaunchConfiguration('microros_baud')
    lidar_port = LaunchConfiguration('lidar_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Robot Description
    urdf_file = os.path.join(pkg_lunohod2, 'description', 'lunohod2.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Node 1: Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Node 2: micro-ROS Agent
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', microros_device, '-b', microros_baud],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # Node 3: RPLidar C1 (delayed start to ensure USB is ready)
    rplidar_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='sllidar_ros2',
                executable='sllidar_node',
                name='sllidar_node',
                parameters=[{
                    'channel_type': 'serial',
                    'serial_port': lidar_port,
                    'serial_baudrate': 460800,
                    'frame_id': 'laser_frame',
                    'inverted': False,
                    'angle_compensate': True,
                    'scan_mode': 'Standard',
                    'auto_reconnect': True,
                }],
                output='screen',
                respawn=True,
                respawn_delay=2.0
            )
        ]
    )

    # Node 4: RViz (optional)
    rviz_config = os.path.join(pkg_lunohod2, 'config', 'robot_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # Arguments
        microros_device_arg,
        microros_baud_arg,
        lidar_port_arg,
        use_sim_time_arg,
        use_rviz_arg,

        # Nodes
        robot_state_publisher_node,
        micro_ros_agent_node,
        rplidar_node,
        rviz_node,
    ])
