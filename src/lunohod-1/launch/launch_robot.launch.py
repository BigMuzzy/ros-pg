import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = "lunohod-1"

    # Launch configuration parameters
    microros_device = LaunchConfiguration("microros_device", default="/dev/ttyUSB0")
    microros_baud = LaunchConfiguration("microros_baud", default="115200")
    lidar_port = LaunchConfiguration("lidar_port", default="/dev/ttyUSB1")

    # Robot State Publisher - Start immediately
    # NOTE: use_ros2_control set to false for direct micro-ROS integration
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "use_ros2_control": "false",  # Disabled for micro-ROS
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

    # Twist Mux - Combines navigation and joystick commands
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/cmd_vel")],  # Direct to ESP32 micro-ROS
    )

    # micro-ROS Agent - Connects to ESP32 via serial
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', microros_device, '-b', microros_baud],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # Joint State Converter - Converts encoder data to joint states for robot_state_publisher
    joint_state_converter = Node(
        package='lunohod-1',
        executable='encoder_to_joint_states.py',
        name='encoder_to_joint_state_converter',
        parameters=[{
            'wheel_radius': 0.0325,  # WHEEL_DIAMETER / 2 = 0.065 / 2
            'counts_per_rev': 5400,  # ENCODER_CPR * GEAR_RATIO = 180 * 30
        }],
        output='screen',
    )

    # Odometry to TF Broadcaster - Publishes odom->base_link transform
    odom_to_tf = Node(
        package='lunohod-1',
        executable='odom_to_tf.py',
        name='odom_to_tf_broadcaster',
        output='screen',
    )

    return LaunchDescription(
        [
            rsp,
            twist_mux,
            micro_ros_agent,
            joint_state_converter,
            odom_to_tf,
#            rplidar,
#            camera,
        ]
    )
