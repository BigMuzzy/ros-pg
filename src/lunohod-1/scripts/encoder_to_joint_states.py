#!/usr/bin/env python3
"""
Encoder to Joint States Converter Node

Converts encoder counts from the Waveshare General Driver ESP32 to joint states
for the robot_state_publisher. This enables proper TF tree visualization and
integration with the ROS2 navigation stack.

Subscribes to:
    /ugv/encoder (std_msgs/Int32MultiArray): [left_count, right_count]

Publishes:
    /joint_states (sensor_msgs/JointState): Wheel joint positions and velocities

Author: Generated for lunohod-1 micro-ROS migration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import math


class EncoderToJointStates(Node):
    """
    Convert encoder counts to joint states for differential drive robot.

    Handles encoder wraparound and calculates both position and velocity
    for left and right wheel joints.
    """

    def __init__(self):
        super().__init__('encoder_to_joint_state_converter')

        # Declare and get parameters
        self.declare_parameter('wheel_radius', 0.0325)  # meters
        self.declare_parameter('counts_per_rev', 5400)   # ENCODER_CPR * GEAR_RATIO
        self.declare_parameter('publish_rate', 30.0)     # Hz
        self.declare_parameter('joint_names', ['left_wheel_joint', 'right_wheel_joint'])

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.counts_per_rev = self.get_parameter('counts_per_rev').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.joint_names = self.get_parameter('joint_names').value

        # Validate parameters
        if self.wheel_radius <= 0:
            self.get_logger().error(f'Invalid wheel_radius: {self.wheel_radius}. Must be positive.')
            raise ValueError('wheel_radius must be positive')
        if self.counts_per_rev <= 0:
            self.get_logger().error(f'Invalid counts_per_rev: {self.counts_per_rev}. Must be positive.')
            raise ValueError('counts_per_rev must be positive')

        # State tracking
        self.last_encoder_counts = None
        self.current_positions = [0.0, 0.0]  # radians
        self.current_velocities = [0.0, 0.0]  # rad/s
        self.last_time = self.get_clock().now()

        # Encoder wraparound handling (assuming int32)
        self.max_encoder_value = 2**31 - 1
        self.min_encoder_value = -(2**31)

        # Create subscriber
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/ugv/encoder',
            self.encoder_callback,
            10
        )

        # Create publisher
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.get_logger().info(
            f'Encoder to Joint States converter initialized\n'
            f'  Wheel radius: {self.wheel_radius} m\n'
            f'  Counts per rev: {self.counts_per_rev}\n'
            f'  Joint names: {self.joint_names}\n'
            f'  Publish rate: {self.publish_rate} Hz'
        )

    def encoder_callback(self, msg):
        """
        Process encoder data and publish joint states.

        Args:
            msg (Int32MultiArray): Encoder counts [left, right]
        """
        if len(msg.data) != 2:
            self.get_logger().warn(
                f'Expected 2 encoder values, got {len(msg.data)}. Ignoring message.'
            )
            return

        current_time = self.get_clock().now()
        current_counts = list(msg.data)

        # Initialize on first message
        if self.last_encoder_counts is None:
            self.last_encoder_counts = current_counts
            self.last_time = current_time
            self.get_logger().info(
                f'Initialized encoder counts: left={current_counts[0]}, right={current_counts[1]}'
            )
            return

        # Calculate time delta
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            self.get_logger().warn('Non-positive time delta, skipping update')
            return

        # Process each wheel
        for i in range(2):
            # Calculate delta counts (handle wraparound)
            delta_counts = self._calculate_delta_with_wraparound(
                self.last_encoder_counts[i],
                current_counts[i]
            )

            # Convert counts to radians
            delta_radians = (delta_counts / self.counts_per_rev) * 2 * math.pi

            # Update position (accumulated)
            self.current_positions[i] += delta_radians

            # Calculate velocity (rad/s)
            self.current_velocities[i] = delta_radians / dt

        # Publish joint states
        self._publish_joint_states(current_time)

        # Update state
        self.last_encoder_counts = current_counts
        self.last_time = current_time

    def _calculate_delta_with_wraparound(self, old_count, new_count):
        """
        Calculate encoder delta handling integer wraparound.

        Args:
            old_count (int): Previous encoder count
            new_count (int): Current encoder count

        Returns:
            int: Delta counts (signed)
        """
        delta = new_count - old_count

        # Check for wraparound
        # If delta is very large positive, likely wrapped from max to min
        if delta > (self.max_encoder_value / 2):
            delta -= (self.max_encoder_value - self.min_encoder_value + 1)
        # If delta is very large negative, likely wrapped from min to max
        elif delta < (self.min_encoder_value / 2):
            delta += (self.max_encoder_value - self.min_encoder_value + 1)

        return delta

    def _publish_joint_states(self, timestamp):
        """
        Publish joint state message.

        Args:
            timestamp: ROS time for the message header
        """
        joint_state = JointState()
        joint_state.header.stamp = timestamp.to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_positions
        joint_state.velocity = self.current_velocities
        # No effort data from encoders

        self.joint_state_pub.publish(joint_state)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)

    try:
        node = EncoderToJointStates()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in encoder_to_joint_states node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
