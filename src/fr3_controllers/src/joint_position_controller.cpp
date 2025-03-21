#include <fr3_controllers/joint_position_controller.hpp>
#include <fr3_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace fr3_controllers {

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!is_gazebo_) {
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/" +
                             k_HW_IF_INITIAL_POSITION);
    }
  } else {
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    }
  }
  return config;
}

controller_interface::return_type JointPositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    for (int i = 0; i < num_joints; ++i) {
      initial_q_.at(i) = state_interfaces_[i].get_value();
    }
    initialization_flag_ = false;
  }

  if (got_new_command_) {
    for (int i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(q_goal_external_(i));
    }
    // Optionally, if you expect one-shot commands, you can reset got_new_command_
    // got_new_command_ = false;
  } else {
    // Otherwise, use the internal trajectory (for example purposes)
    elapsed_time_ += trajectory_period;
    double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * elapsed_time_)) * 0.2;
    for (int i = 0; i < num_joints; ++i) {
      if (i == 4) {
        command_interfaces_[i].set_value(initial_q_.at(i) - delta_angle);
      } else {
        command_interfaces_[i].set_value(initial_q_.at(i) + delta_angle);
      }
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointPositionController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
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

CallbackReturn JointPositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;

  // Create subscription for external joint position commands
  command_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_state/command",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&JointPositionController::commandCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

void JointPositionController::commandCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < static_cast<size_t>(num_joints)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received command with insufficient joint positions");
    return;
  }
  for (int i = 0; i < num_joints; ++i) {
    q_goal_external_(i) = msg->position[i];
  }
  got_new_command_ = true;
}

}  // namespace fr3_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::JointPositionController,
                       controller_interface::ControllerInterface)
