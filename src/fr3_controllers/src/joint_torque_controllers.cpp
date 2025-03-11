// #include "fr3_controllers/joint_torque_controller.hpp"

// #include <pluginlib/class_list_macros.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <urdf/model.h>
// #include <exception>
// #include <string>

// namespace fr3_controllers {

// CallbackReturn TorqueController::on_init() {
//   try {
//     // Declare parameters with default values
//     auto_declare<std::string>("arm_id", "fr3");
//     auto_declare<std::string>("base_link", "base_link");
//     auto_declare<std::string>("tip_link", "tool_link");  // Change to your actual end-effector link name
//     auto_declare<std::string>("robot_description", "");
    
//     // Initialize desired torque vector with zeros.
//     desired_torque_.resize(num_joints_, 0.0);
//   } catch (const std::exception &e) {
//     RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
//     return CallbackReturn::ERROR;
//   }

//   // Create subscription to receive desired torque commands from the "torque_command" topic.
//   torque_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
//       "torque_command", 10,
//       std::bind(&TorqueController::torqueCommandCallback, this, std::placeholders::_1));
//   return CallbackReturn::SUCCESS;
// }

// CallbackReturn TorqueController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
//   // Retrieve parameters (assumed to be set via launch or YAML file)
//   arm_id_ = get_node()->get_parameter("arm_id").as_string();
//   base_link_ = get_node()->get_parameter("base_link").as_string();
//   tip_link_ = get_node()->get_parameter("tip_link").as_string();
//   robot_description_ = get_node()->get_parameter("robot_description").as_string();

//   if (robot_description_.empty()) {
//     RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is empty.");
//     return CallbackReturn::ERROR;
//   }

//   // Optionally: Extract or verify arm name from the robot description.
//   // For example, you could use a utility function like:
//   // arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

//   // Parse the URDF to create a KDL tree.
//   urdf::Model urdf_model;
//   if (!urdf_model.initString(robot_description_)) {
//     RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF");
//     return CallbackReturn::ERROR;
//   }
//   KDL::Tree kdl_tree;
//   if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
//     RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct KDL tree");
//     return CallbackReturn::ERROR;
//   }

//   // IMPORTANT: Specify the chain from base_link_ to tip_link_
//   // Check your URDF file (or use RViz/grep) to confirm these names.
//   if (!kdl_tree.getChain(base_link_, tip_link_, kdl_chain_)) {
//     RCLCPP_ERROR(get_node()->get_logger(), "Failed to extract KDL chain from %s to %s",
//                  base_link_.c_str(), tip_link_.c_str());
//     return CallbackReturn::ERROR;
//   }

//   // Initialize the dynamic parameter solver with the KDL chain and a gravity vector.
//   KDL::Vector gravity(0, 0, -9.81);
//   dyn_param_solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity);

//   return CallbackReturn::SUCCESS;
// }

// controller_interface::InterfaceConfiguration TorqueController::command_interface_configuration() const {
//   controller_interface::InterfaceConfiguration config;
//   config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
//   // Set up effort (torque) command interfaces for each joint.
//   for (int i = 1; i <= num_joints_; ++i) {
//     config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
//   }
//   return config;
// }

// controller_interface::InterfaceConfiguration TorqueController::state_interface_configuration() const {
//   controller_interface::InterfaceConfiguration config;
//   config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
//   // Request state interfaces for joint positions and velocities.
//   for (int i = 1; i <= num_joints_; ++i) {
//     config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
//   }
//   for (int i = 1; i <= num_joints_; ++i) {
//     config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
//   }
//   return config;
// }

// controller_interface::return_type TorqueController::update(const rclcpp::Time & /*time*/,
//                                                              const rclcpp::Duration & /*period*/) {
//   // Read the current joint states from the state interfaces.
//   KDL::JntArray joint_positions(num_joints_);
//   KDL::JntArray joint_velocities(num_joints_);
//   // Assuming the first num_joints_ interfaces are positions, and the next num_joints_ are velocities.
//   for (int i = 0; i < num_joints_; ++i) {
//     joint_positions(i) = state_interfaces_[i].get_value();
//     joint_velocities(i) = state_interfaces_[i + num_joints_].get_value();
//   }

//   // Compute the gravity torques.
//   KDL::JntArray gravity_torques(num_joints_);
//   dyn_param_solver_->JntToGravity(joint_positions, gravity_torques);

//   // Compute the Coriolis torques.
//   KDL::JntArray coriolis_torques(num_joints_);
//   dyn_param_solver_->JntToCoriolis(joint_positions, joint_velocities, coriolis_torques);

//   // Combine desired torque with gravity and Coriolis compensation.
//   for (int i = 0; i < num_joints_; ++i) {
//     double torque_command = desired_torque_[i] + gravity_torques(i) + coriolis_torques(i);
//     command_interfaces_[i].set_value(torque_command);
//   }

//   return controller_interface::return_type::OK;
// }

// void TorqueController::torqueCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
//   if (msg->data.size() != static_cast<size_t>(num_joints_)) {
//     RCLCPP_ERROR(get_node()->get_logger(),
//                  "Received torque command size (%zu) does not match number of joints (%d)",
//                  msg->data.size(), num_joints_);
//     return;
//   }
//   desired_torque_ = msg->data;
// }

// }  // namespace franka_example_controllers

// // Export the controller as a plugin.
// PLUGINLIB_EXPORT_CLASS(franka_example_controllers::TorqueController,
//                        controller_interface::ControllerInterface)