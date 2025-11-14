#!/usr/bin/env python3
"""
IMU Timestamp Synchronization Node

Republishes IMU data from ESP32 with synchronized ROS timestamps.
The ESP32 publishes IMU messages with internal uptime timestamps,
which need to be converted to ROS time for proper TF integration.

Subscribes to:
    /ugv/imu (sensor_msgs/Imu): IMU data from ESP32 with uptime timestamps

Publishes:
    /imu (sensor_msgs/Imu): IMU data with synchronized ROS timestamps
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuTimestampSync(Node):
    """Synchronize IMU timestamps from ESP32 uptime to ROS time."""

    def __init__(self):
        super().__init__('imu_timestamp_sync')

        # Subscribe to ESP32 IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/ugv/imu',
            self.imu_callback,
            10
        )

        # Publish synchronized IMU
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu',
            10
        )

        self.get_logger().info('IMU timestamp synchronization started')
        self.get_logger().info('  Subscribing to: /ugv/imu')
        self.get_logger().info('  Publishing to: /imu')

    def imu_callback(self, msg):
        """
        Republish IMU message with synchronized timestamp.

        Args:
            msg (Imu): IMU message from ESP32
        """
        # Create new message with synchronized timestamp
        synced_msg = Imu()

        # Update timestamp to current ROS time
        synced_msg.header.stamp = self.get_clock().now().to_msg()
        synced_msg.header.frame_id = 'imu_link'  # Use proper IMU frame

        # Copy orientation (not available from ESP32, all zeros)
        synced_msg.orientation = msg.orientation
        synced_msg.orientation_covariance = msg.orientation_covariance

        # Copy angular velocity
        synced_msg.angular_velocity = msg.angular_velocity
        synced_msg.angular_velocity_covariance = msg.angular_velocity_covariance

        # Copy linear acceleration
        synced_msg.linear_acceleration = msg.linear_acceleration
        synced_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Publish synchronized message
        self.imu_pub.publish(synced_msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = ImuTimestampSync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
