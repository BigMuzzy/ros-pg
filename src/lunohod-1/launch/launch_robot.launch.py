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

    package_name = "lunohod-1"

    ugv_driver_port = LaunchConfiguration("ugv_driver_port", default="/dev/ttyUSB0")
    lidar_port = LaunchConfiguration("lidar_port", default="/dev/ttyUSB1")

    # Robot State Publisher - Start immediately (no ros2_control needed for micro-ROS)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={
            "ugv_driver_port": ugv_driver_port,
            "use_sim_time": "false",
            "use_ros2_control": "false",
        }.items(),
    )

    # Lidar - Start after a short delay to ensure USB is ready
    rplidar = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory(package_name),
                            "launch",
                            "rplidar_robust.launch.py",
                        )
                    ]
                ),
                launch_arguments={"lidar_port": lidar_port}.items(),
            )
        ],
    )

    # Camera - Start after lidar
    camera = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory(package_name),
                            "launch",
                            "camera.launch.py",
                        )
                    ]
                )
            )
        ],
    )

    # Twist Mux - Start early
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    # micro-ROS Agent - Start early to connect with microcontroller via serial
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", ugv_driver_port, "-b", "115200", "-v6"],
        output="screen",
    )

    # Delay micro-ROS agent to allow USB enumeration
    delayed_micro_ros_agent = TimerAction(period=2.0, actions=[micro_ros_agent])

    return LaunchDescription(
        [
            rsp,
            twist_mux,
            rplidar,
            camera,
            delayed_micro_ros_agent,
        ]
    )
