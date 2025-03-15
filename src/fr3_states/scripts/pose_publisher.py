#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        # Publisher for the pose
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        
        # Subscribers for joint states and robot description
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.create_subscription(String, 'robot_description', self.robot_description_callback, 10)
        
        self.urdf_loaded = False
        self.kdl_chain = None
        self.chain_joint_names = []
        self.fk_solver = None

        # Parameters for base and tip link names (change as needed)
        self.base_link = self.declare_parameter('base_link', 'base_link').value
        self.tip_link  = self.declare_parameter('tip_link', 'ee_link').value

    def robot_description_callback(self, msg):
        if self.urdf_loaded:
            return
        self.get_logger().info("Received robot description, parsing URDF...")
        try:
            # Parse the URDF and build the KDL tree
            robot = URDF.from_xml_string(msg.data)
            from kdl_parser_py.urdf import treeFromUrdfModel
            (ok, tree) = treeFromUrdfModel(robot)
            if not ok:
                self.get_logger().error("Failed to construct KDL tree from URDF")
                return
            self.kdl_chain = tree.getChain(self.base_link, self.tip_link)
            self.get_logger().info("KDL chain constructed from '{}' to '{}'".format(self.base_link, self.tip_link))
            # Get the joint names from the chain (ignoring fixed joints)
            for i in range(self.kdl_chain.getNrOfSegments()):
                segment = self.kdl_chain.getSegment(i)
                joint = segment.getJoint()
                if joint.getType() != kdl.Joint.Fixed:
                    self.chain_joint_names.append(joint.getName())
            self.get_logger().info("Chain joint names: {}".format(self.chain_joint_names))
            # Initialize the forward kinematics solver
            self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)
            self.urdf_loaded = True
        except Exception as e:
            self.get_logger().error("Exception parsing URDF: {}".format(e))
    
    def joint_state_callback(self, msg):
        if not self.urdf_loaded:
            self.get_logger().warn("URDF not loaded yet, cannot compute pose.")
            return
        
        # Gather joint positions in the order expected by the chain
        joint_positions = []
        for name in self.chain_joint_names:
            if name in msg.name:
                index = msg.name.index(name)
                joint_positions.append(msg.position[index])
            else:
                self.get_logger().error("Joint '{}' not found in JointState".format(name))
                return
        
        num_joints = len(self.chain_joint_names)
        jnt_array = kdl.JntArray(num_joints)
        for i, pos in enumerate(joint_positions):
            jnt_array[i] = pos
        
        # Compute the forward kinematics to get the end-effector frame
        frame = kdl.Frame()
        ret = self.fk_solver.JntToCart(jnt_array, frame)
        if ret < 0:
            self.get_logger().error("Failed to compute FK")
            return
        
        # Extract position and orientation from the KDL frame
        pos = frame.p
        q = frame.M.GetQuaternion()  # Returns (x, y, z, w)
        
        # Fill in a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.base_link  # using the base link as the reference frame
        pose_msg.pose.position.x = pos[0]
        pose_msg.pose.position.y = pos[1]
        pose_msg.pose.position.z = pos[2]
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        self.pose_pub.publish(pose_msg)
        self.get_logger().info("Published pose: position ({:.3f}, {:.3f}, {:.3f})".format(pos[0], pos[1], pos[2]))

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
