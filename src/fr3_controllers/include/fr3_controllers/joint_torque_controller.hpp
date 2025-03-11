// #pragma once

// #include <string>
// #include <vector>
// #include <memory>

// #include <controller_interface/controller_interface.hpp>
// #include "franka_example_controllers/visibility_control.h"

// #include <rclcpp/duration.hpp>
// #include <rclcpp/time.hpp>
// #include <rclcpp_lifecycle/lifecycle_node.hpp>
// #include <rclcpp/parameter_client.hpp>

// #include <std_msgs/msg/float64_multi_array.hpp>

// #include <kdl/chain.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chaindynparam.hpp>

// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// namespace fr3_controllers {

// /**
//  * @brief A torque controller that adds external torque commands on top of gravity
//  *        and Coriolis compensation computed using KDL.
//  *
//  * The final commanded torque for each joint is:
//  *   torque = desired_torque + gravity_compensation + coriolis_compensation.
//  *
//  * The desired torque is received via a ROS topic ("torque_command").
//  * 
//  * The controller uses parameters for:
//  *   - arm_id: A prefix for joint names.
//  *   - base_link: The base link of the robot (from which the KDL chain starts).
//  *   - tip_link: The tip (or end-effector) link of the robot (the end of the KDL chain).
//  *
//  * To check the correct names:
//  *   - Inspect your URDF (or xacro) file for the <link name="..."> tags.
//  *   - Use RViz to visualize the robot.
//  *   - Query the robot_description parameter: 
//  *         ros2 param get /robot_state_publisher robot_description
//  */
// class TorqueController : public controller_interface::ControllerInterface {
//  public:
//   CallbackReturn on_init() override;
//   CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

//   [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
//   [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

//   controller_interface::return_type update(const rclcpp::Time &time,
//                                              const rclcpp::Duration &period) override;

//  private:
//   // Parameters (set via parameter server or launch file)
//   std::string arm_id_;
//   std::string base_link_;
//   std::string tip_link_;
//   std::string robot_description_;
//   const int num_joints_ = 7;

//   // KDL chain and dynamic parameter solver
//   KDL::Chain kdl_chain_;
//   std::unique_ptr<KDL::ChainDynParam> dyn_param_solver_;

//   // External desired torque (size = num_joints_)
//   std::vector<double> desired_torque_;

//   // Subscription to receive external desired torque commands.
//   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_subscriber_;

//   // Callback for torque command subscription.
//   void torqueCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
// };

// }  // namespace franka_example_controllers
