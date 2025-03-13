#include <fr3_controllers/joint_torque_controller.hpp>

#include <exception>
#include <string>
#include <Eigen/Eigen>

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
  return {};
}

controller_interface::return_type JointTorqueController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  torque_external << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(torque_external(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTorqueController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::JointTorqueController,
                       controller_interface::ControllerInterface)