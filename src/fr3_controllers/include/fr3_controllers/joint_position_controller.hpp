#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 

#include "franka_semantic_components/franka_robot_state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers {

class JointPositionController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  bool is_gazebo_{false};
  std::string robot_description_;
  const int num_joints = 7;
  std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};
  const double trajectory_period{0.001};
  double elapsed_time_ = 0.0;
  const std::string k_HW_IF_INITIAL_POSITION = "initial_joint_position";

  bool initialization_flag_{true};
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  Vector7d q_goal_external_;  
  bool got_new_command_{false}; 
  void commandCallback(const sensor_msgs::msg::JointState::SharedPtr msg); 
  
  rclcpp::Time start_time_;
};

}  // namespace fr3_controllers
