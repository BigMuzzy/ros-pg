#!/usr/bin/env python3
"""
Odometry to TF Broadcaster

Republishes the transform from /ugv/odom to /tf for visualization and navigation.
This is needed because the ESP32 publishes odometry messages but doesn't broadcast
the transform to the /tf topic.

Subscribes to:
    /ugv/odom (nav_msgs/Odometry): Odometry from ESP32

Publishes:
    /tf (tf2_msgs/TFMessage): Transform from odom to base_link
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    """Broadcast TF transform from odometry messages."""

    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ugv/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('Odometry to TF broadcaster started')
        self.get_logger().info('  Subscribing to: /ugv/odom')
        self.get_logger().info('  Publishing TF: odom -> base_link')

    def odom_callback(self, msg):
        """
        Convert odometry message to TF transform.

        Args:
            msg (Odometry): Odometry message from ESP32
        """
        # Create transform message
        t = TransformStamped()

        # Header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # 'odom'
        t.child_frame_id = msg.child_frame_id    # 'base_link'

        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = OdomToTF()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
