// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fr3_controllers/gripper_controller.hpp>
#include <fr3_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace fr3_controllers {

controller_interface::InterfaceConfiguration
GripperController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::InterfaceConfiguration
GripperController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::return_type GripperController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  return controller_interface::return_type::OK;
}

CallbackReturn GripperController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::string>("robot_ip", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  robot_ip_ = get_node()->get_parameter("robot_ip").as_string();
  

  // create the subscriber for teleop_motion_plan
  motion_plan_ = get_node()->create_subscription<teleop_interfaces::msg::TeleopMotionPlan>(
      "/teleop_motion_plan", 
      rclcpp::SystemDefaultsQoS(),
      std::bind(&GripperController::teleop_motion_plan_callback, this, std::placeholders::_1)
  );

  // create in instance of franka::gripper
  gripper_ = std::make_unique<franka::Gripper>(robot_ip_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  width = 0.0;
  speed = 0.5; 
  force = 1.0; // newtons
              
  prev_width = -1;
  gripper_->grasp(width, speed, force, 0.005, 0.005); // default grasp_epsilon

  return CallbackReturn::SUCCESS;
}

void GripperController::teleop_motion_plan_callback(
    const teleop_interfaces::msg::TeleopMotionPlan::SharedPtr msg) {

  
  teleop_interfaces::msg::JointGroup registered_group;

  if (arm_id_ == "fr3_left") {
    for (auto& group: msg->joint_group_plans) {
      if (group.group_name == "left_gripper") {
        registered_group = group;
        break;
      }
    }
  }else {
    for (auto& group: msg->joint_group_plans) {
      if (group.group_name == "right_gripper") {
        registered_group = group;
        break;
      }
    }
  }

  width = registered_group.positions[0];
  if (prev_width != width) {
    if (width == 0.0) {
      gripper_->stop();
    }else if (width > 0.0) {
      gripper_->move(0.08, speed);
    }else {
      gripper_->move(0.0, speed);
    }
  }
  prev_width = width;
  got_teleop_motion = true;
}


}  // namespace fr3_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::GripperController,
                       controller_interface::ControllerInterface)