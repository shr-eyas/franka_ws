#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SmoothTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("smooth_trajectory_publisher")
        # Subscribe to the current joint states (e.g. from the right arm)
        self.subscription = self.create_subscription(
            JointState, "/fr3_right/joint_states", self.joint_state_callback, 10
        )
        # Publisher for the desired trajectory (target joint angles)
        self.publisher = self.create_publisher(JointState, "/joint_angles", 10)

        # Define the target joint state (7 joints)
        self.target_state = np.array([0.1, 0.13, 1.33, 0.80, -2.29, -0.14, -0.86])
        self.duration = 150.0  # seconds over which to complete the trajectory
        self.trajectory_timer = None
        self.start_state = None
        self.start_time = None
        # Flag to ensure trajectory is planned only once
        self.trajectory_planned = False

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) < 7:
            self.get_logger().error("Received joint state with less than 7 positions")
            return

        # Plan the trajectory only once, using the first received valid state.
        if not self.trajectory_planned:
            self.start_state = np.array(msg.position[:7])
            self.get_logger().info(f"Start state captured: {self.start_state}")
            self.trajectory_planned = True
            self.start_time = (
                self.get_clock().now().nanoseconds * 1e-9
            )  # convert to seconds
            self.get_logger().info("Planning smooth trajectory over 5 seconds.")
            # Use a timer callback at a short interval (e.g. 50ms) to update the trajectory
            self.trajectory_timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds * 1e-9  # current time in seconds
        t = now - self.start_time

        if t >= self.duration:
            t = self.duration
            if self.trajectory_timer is not None:
                self.trajectory_timer.cancel()
                self.trajectory_timer = None
            self.get_logger().info("Trajectory complete.")

        # Cubic polynomial interpolation factor for smooth motion (s=0 at t=0, s=1 at t=T)
        # s = 3*(t/T)^2 - 2*(t/T)^3 gives zero start and end velocities.
        T = self.duration
        s = 3 * (t / T) ** 2 - 2 * (t / T) ** 3

        # Interpolate between the start and target states.
        interpolated = (1 - s) * self.start_state + s * self.target_state

        # Create and publish the JointState message.
        out_msg = JointState()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.position = interpolated.tolist()
        self.publisher.publish(out_msg)
        self.get_logger().info(
            f"Published interpolated state (t={t:.2f}s): {out_msg.position}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SmoothTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
