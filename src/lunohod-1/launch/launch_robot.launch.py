import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    package_name='lunohod-1'

    arduino_device = LaunchConfiguration('arduino_device')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')


    # Robot State Publisher - Start immediately
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={
                    'arduino_device': arduino_device,
                    'use_sim_time': 'false',
                    'use_ros2_control': 'true'}.items()
    )

    # Lidar - Start after a short delay to ensure USB is ready
    rplidar = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rplidar_robust.launch.py'
                )]), launch_arguments={'lidar_port': lidar_port}.items()
            )
        ]
    )

    # Camera - Start after lidar
    camera = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','camera.launch.py'
                )])
            )
        ]
    )

    # Twist Mux - Start early
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    # Controller Manager - Start after RSP is ready
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    # Delay controller manager more to avoid conflicts
    delayed_controller_manager = TimerAction(period=6.0, actions=[controller_manager])

    # Controllers - Start after controller manager
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[TimerAction(period=3.0, actions=[diff_drive_spawner])],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[TimerAction(period=2.0, actions=[joint_broad_spawner])],
        )
    )

    return LaunchDescription([
        rsp,
        twist_mux,
        rplidar,
        camera,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])