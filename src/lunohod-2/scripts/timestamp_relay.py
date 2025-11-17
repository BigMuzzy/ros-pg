#!/usr/bin/env python3
"""
Timestamp Relay Node for micro-ROS messages

This node subscribes to micro-ROS topics that have MCU timestamps
and republishes them with current ROS2 system timestamps.

This is necessary because micro-ROS messages use MCU boot time,
while the main system uses Unix epoch time. This causes TF tree
timestamp mismatches.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class TimestampRelay(Node):
    def __init__(self):
        super().__init__('timestamp_relay')
        
        self.get_logger().info('Timestamp Relay Node started')
        
        # Odometry relay: /ugv/odom -> /ugv/odom_synced
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ugv/odom',
            self.odom_callback,
            10
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            '/ugv/odom_synced',
            10
        )
        
        # IMU relay: /ugv/imu -> /ugv/imu_synced
        self.imu_sub = self.create_subscription(
            Imu,
            '/ugv/imu',
            self.imu_callback,
            10
        )
        self.imu_pub = self.create_publisher(
            Imu,
            '/ugv/imu_synced',
            10
        )
        
        self.get_logger().info('Relaying:')
        self.get_logger().info('  /ugv/odom -> /ugv/odom_synced')
        self.get_logger().info('  /ugv/imu -> /ugv/imu_synced')
    
    def odom_callback(self, msg):
        """Re-timestamp and republish odometry"""
        # Update timestamp to current time
        msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(msg)
    
    def imu_callback(self, msg):
        """Re-timestamp and republish IMU data"""
        # Update timestamp to current time
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TimestampRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
