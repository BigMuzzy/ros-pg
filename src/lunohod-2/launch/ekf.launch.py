#!/usr/bin/env python3

"""
Lunohod-2 EKF (Extended Kalman Filter) Launch File

Launches robot_localization EKF node for sensor fusion.
Fuses wheel odometry and IMU data to produce filtered odometry estimate.

Publishes:
  - /odometry/filtered (nav_msgs/Odometry)
  - odom→base_link transform (if publish_tf=true)

Usage:
    ros2 launch lunohod-2 ekf.launch.py
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

    # EKF configuration file
    ekf_config = os.path.join(pkg_lunohod2, 'config', 'ekf.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
    ])
