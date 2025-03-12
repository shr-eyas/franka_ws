#include <fr3_controllers/joint_torque_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace fr3_controllers {

controller_interface::InterfaceConfiguration
JointTorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointTorqueController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Include position and velocity state interfaces.
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type
JointTorqueController::update(const rclcpp::Time& /*time*/,
                                                const rclcpp::Duration& /*period*/) {
  // Get the current joint positions and velocities from the state interfaces.
  KDL::JntArray q(num_joints);
  KDL::JntArray dq(num_joints);

  // Compute the gravity compensation torques.
  KDL::JntArray gravity(num_joints);
  int ret = dyn_param_solver_->JntToGravity(q, gravity);
  if (ret < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to compute gravity torques");
  }

  // Compute the Coriolis (and centrifugal) compensation torques.
  KDL::JntArray coriolis(num_joints);
  ret = dyn_param_solver_->JntToCoriolis(q, dq, coriolis);
  if (ret < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to compute Coriolis torques");
  }

  // Compose the final torque command:
  // final_torque = external_command + gravity_compensation + coriolis_compensation
  for (int i = 0; i < num_joints; ++i) {
    double final_torque = external_torque_command_(i) + gravity(i) + coriolis(i);
    command_interfaces_[i].set_value(final_torque);
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointTorqueController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::string>("base_link", "");
    auto_declare<std::string>("tip_link", "");
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception during on_init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTorqueController::on_configure(
    const rclcpp::lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  base_link_ = get_node()->get_parameter("base_link").as_string();
  tip_link_ = get_node()->get_parameter("tip_link").as_string();
  robot_description_ = get_node()->get_parameter("robot_description").as_string();

  // Build the KDL tree from the URDF.
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(robot_description_, kdl_tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct KDL tree from URDF");
    return CallbackReturn::FAILURE;
  }
  // Extract the chain from base_link to tip_link.
  if (!kdl_tree.getChain(base_link_, tip_link_, kdl_chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get KDL chain from %s to %s",
                 base_link_.c_str(), tip_link_.c_str());
    return CallbackReturn::FAILURE;
  }
  // Initialize the dynamics solver with the gravity vector (assumed as [0, 0, -9.81]).
  dyn_param_solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0, 0, -9.81));

  // Initialize the external torque command vector to zero.
  external_torque_command_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTorqueController::on_activate(
    const rclcpp::lifecycle::State& /*previous_state*/) {
  // Create a subscription for externally commanded torques.
  command_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/" + arm_id_ + "/joint_state/command",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&JointTorqueController::torqueCommandCallback, this, std::placeholders::_1));
  return CallbackReturn::SUCCESS;
}

void JointTorqueController::commandCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->effort.size() < static_cast<size_t>(num_joints)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received command with insufficient joint efforts");
    return;
  }
  for (int i = 0; i < num_joints; ++i) {
    external_torque_command_(i) = msg->effort[i];
  }
  got_new_command_ = true;
}

void JointTorqueController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace fr3_controllers

PLUGINLIB_EXPORT_CLASS(fr3_controllers::JointTorqueController,
                       controller_interface::ControllerInterface)