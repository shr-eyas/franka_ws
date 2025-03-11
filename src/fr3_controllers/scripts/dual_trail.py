#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SinusoidalBasePublisher(Node):
    def __init__(self):
        super().__init__("sinusoidal_base_publisher")
        # Publisher to send the command joint state.
        self.publisher = self.create_publisher(
            JointState, "/fr3_right/joint_state/command", 10
        )
        # Subscriber to get current joint state (we use it to capture the initial state)
        self.joint_state_sub = self.create_subscription(
            JointState, "/fr3_right/joint_states", self.joint_state_callback, 10
        )
        # Timer for periodic publishing
        self.timer = self.create_timer(
            0.01, self.timer_callback
        )  # 100 Hz publishing rate
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # Capture the initial joint state once. All non-base joints will use these values.
        self.initial_joint_state = None

        # Parameters for the sinusoidal motion on the base joint:
        self.amplitude = 0.6  # Amplitude in radians
        self.frequency = 0.3  # Frequency in Hz (e.g. 0.5 Hz gives a 2-second period)

    def joint_state_callback(self, msg: JointState):
        # Capture the initial joint state once.
        if self.initial_joint_state is None:
            self.initial_joint_state = msg
            self.get_logger().info("Captured initial joint state.")

    def timer_callback(self):
        # Only publish if the initial joint state has been captured.
        if self.initial_joint_state is None:
            return

        # Calculate elapsed time in seconds.
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.start_time

        # Build a new JointState command message using the initial joint state for non-base joints.
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = list(self.initial_joint_state.name)
        cmd_msg.position = list(self.initial_joint_state.position)

        # Compute the sinusoidal offset for the base joint (assumed to be at index 0)
        sinusoidal_value = self.amplitude * math.sin(
            2 * math.pi * self.frequency * elapsed
        )
        if len(cmd_msg.position) > 0:
            cmd_msg.position[0] = sinusoidal_value

        self.publisher.publish(cmd_msg)
        self.get_logger().info(
            f"Published sinusoidal command for base: {cmd_msg.position[0]:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalBasePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
