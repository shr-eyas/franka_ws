#include <fr3_controllers/default_robot_behavior_utils.hpp>
#include <fr3_controllers/joint_velocity_controller.hpp>
#include <fr3_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace std::chrono_literals;

namespace fr3_controllers {

controller_interface::InterfaceConfiguration
JointVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointVelocityController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
    if (got_new_velocity_command_) {
      for (int i = 0; i < num_joints; i++) {
        command_interfaces_[i].set_value(external_velocity_(i));
      }
      got_new_velocity_command_ = false;
    } 
    else {
      for (int i = 0; i < num_joints; i++) {
        command_interfaces_[i].set_value(0.0);
      }
    }
  return controller_interface::return_type::OK;
}

CallbackReturn JointVelocityController::on_init() {
  try {
    auto_declare<bool>("use_fake_hardware", false);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  use_fake_hardware = get_node()->get_parameter("use_fake_hardware").as_bool();

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

  if (!use_fake_hardware) {
    auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");
    auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

    auto future_result = client->async_send_request(request);
    future_result.wait_for(1000ms);

    auto success = future_result.get();
    if (!success) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);

  velocity_cmd_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/" + arm_id_ + "/joint_state/velocity_command",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&JointVelocityController::velocityCommandCallback, this, std::placeholders::_1)
  );
  return CallbackReturn::SUCCESS;
}

void JointVelocityController::velocityCommandCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->velocity.size() < static_cast<size_t>(num_joints)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received velocity command with insufficient elements");
    return;
  }
  if (external_velocity_.size() != num_joints) {
    external_velocity_.resize(num_joints);
  }
  for (int i = 0; i < num_joints; i++) {
    external_velocity_(i) = msg->velocity[i];
  }
  got_new_velocity_command_ = true;
}

}  // namespace fr3_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::JointVelocityController,
                       controller_interface::ControllerInterface)