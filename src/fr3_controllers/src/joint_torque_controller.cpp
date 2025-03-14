#include <fr3_controllers/joint_torque_controller.hpp>
#include <fr3_controllers/robot_utils.hpp>

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
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

CallbackReturn JointTorqueController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}


CallbackReturn JointTorqueController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
    franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                 arm_id_ + "/" + k_robot_state_interface_name));
                     
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTorqueController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  RCLCPP_INFO(get_node()->get_logger(), "Hola!");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointTorqueController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Vector7d coriolis = Eigen::Map<Vector7d>(coriolis_array.data());

  torque_external << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Vector7d torque_cmd = torque_external + coriolis;

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(torque_cmd(i));
  }
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::JointTorqueController,
                       controller_interface::ControllerInterface)