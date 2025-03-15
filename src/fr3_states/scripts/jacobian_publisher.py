#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
import numpy as np

class JacobianPublisher(Node):
    def __init__(self):
        super().__init__('jacobian_publisher')
        # Publisher for the Jacobian as a flattened 2D array
        self.jacobian_pub = self.create_publisher(Float64MultiArray, 'jacobian', 10)
        
        # Subscribers for joint states and robot description
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.create_subscription(String, 'robot_description', self.robot_description_callback, 10)
        
        self.urdf_loaded = False
        self.kdl_chain = None
        self.chain_joint_names = []
        self.jnt_to_jac_solver = None

        # Allow setting the base and tip links via parameters (default values can be modified)
        self.base_link = self.declare_parameter('base_link', 'base_link').value
        self.tip_link  = self.declare_parameter('tip_link', 'ee_link').value

    def robot_description_callback(self, msg):
        if self.urdf_loaded:
            return
        self.get_logger().info("Received robot description, parsing URDF...")
        try:
            # Parse URDF and build KDL tree
            robot = URDF.from_xml_string(msg.data)
            from kdl_parser_py.urdf import treeFromUrdfModel
            (ok, tree) = treeFromUrdfModel(robot)
            if not ok:
                self.get_logger().error("Failed to construct KDL tree from URDF")
                return
            # Get chain from the specified base to tip
            self.kdl_chain = tree.getChain(self.base_link, self.tip_link)
            self.get_logger().info("KDL chain constructed from '{}' to '{}'".format(self.base_link, self.tip_link))
            # Extract joint names (skip fixed joints)
            for i in range(self.kdl_chain.getNrOfSegments()):
                segment = self.kdl_chain.getSegment(i)
                joint = segment.getJoint()
                if joint.getType() != kdl.Joint.Fixed:
                    self.chain_joint_names.append(joint.getName())
            self.get_logger().info("Chain joint names: {}".format(self.chain_joint_names))
            # Initialize the Jacobian solver
            self.jnt_to_jac_solver = kdl.ChainJntToJacSolver(self.kdl_chain)
            self.urdf_loaded = True
        except Exception as e:
            self.get_logger().error("Exception parsing URDF: {}".format(e))

    def joint_state_callback(self, msg):
        if not self.urdf_loaded:
            self.get_logger().warn("URDF not loaded yet, cannot compute Jacobian.")
            return
        
        # Extract joint positions in the order required by the KDL chain
        joint_positions = []
        for name in self.chain_joint_names:
            if name in msg.name:
                index = msg.name.index(name)
                joint_positions.append(msg.position[index])
            else:
                self.get_logger().error("Joint '{}' not found in JointState".format(name))
                return
        
        # Build a KDL JntArray
        num_joints = len(self.chain_joint_names)
        jnt_array = kdl.JntArray(num_joints)
        for i, pos in enumerate(joint_positions):
            jnt_array[i] = pos
        
        # Compute the Jacobian
        jacobian = kdl.Jacobian(num_joints)
        self.jnt_to_jac_solver.JntToJac(jnt_array, jacobian)
        
        # Convert the KDL.Jacobian (6 x n) to a NumPy array
        jacobian_np = np.zeros((jacobian.rows(), jacobian.columns()))
        for i in range(jacobian.rows()):
            for j in range(jacobian.columns()):
                jacobian_np[i, j] = jacobian[i, j]
        
        # Prepare and publish the Jacobian as a Float64MultiArray (flattened row-wise)
        jacobian_msg = Float64MultiArray()
        jacobian_msg.data = jacobian_np.flatten().tolist()
        self.jacobian_pub.publish(jacobian_msg)
        self.get_logger().info("Published Jacobian with shape {}x{}".format(jacobian_np.shape[0], jacobian_np.shape[1]))

def main(args=None):
    rclpy.init(args=args)
    node = JacobianPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
