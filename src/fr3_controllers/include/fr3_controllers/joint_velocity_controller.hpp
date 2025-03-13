#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 
#include <Eigen/Dense>  

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers {

class JointVelocityController : public controller_interface::ControllerInterface {

 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  std::string robot_description_;
  bool use_fake_hardware{false};
  const int num_joints = 7;
  rclcpp::Duration elapsed_time_ = rclcpp::Duration(0, 0);

  Eigen::VectorXd external_velocity_; 
  bool got_new_velocity_command_{false}; 
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr velocity_cmd_sub_;
  void velocityCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace fr3_controllers
