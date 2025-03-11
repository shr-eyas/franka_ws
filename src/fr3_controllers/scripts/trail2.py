#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class AddBaseOffsetPublisher(Node):
    def __init__(self):
        super().__init__("add_base_offset_publisher")
        # Subscriber to current joint states (adjust topic if necessary)
        self.subscriber = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        # Publisher for the command (target joint state)
        self.publisher = self.create_publisher(JointState, "/joint_state/command", 10)
        self.processed = False  # Ensure we only publish once

    def joint_state_callback(self, msg: JointState):
        if self.processed:
            return

        # Copy the received joint state
        new_msg = JointState()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.name = msg.name[:]  # copy the joint names if available
        new_msg.position = list(msg.position)  # copy the positions

        # Add 0.1 to the base joint (assuming base joint is at index 0)
        if len(new_msg.position) > 0:
            new_msg.position[0] += 0.5
            self.get_logger().info(f"Base joint command: {new_msg.position[0]:.3f} rad")
        else:
            self.get_logger().error("No joint positions received!")

        # Publish the updated joint state as a command
        self.publisher.publish(new_msg)
        self.get_logger().info("Published updated joint state to /joint_state/command")
        self.processed = True


def main(args=None):
    rclpy.init(args=args)
    node = AddBaseOffsetPublisher()
    # Spin until the message is processed, then shutdown.
    rclpy.spin_once(node, timeout_sec=5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
