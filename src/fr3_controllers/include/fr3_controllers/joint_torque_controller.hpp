#pragma once

#include <string>
#include <memory>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// KDL headers
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers {

class JointTorqueController : public controller_interface::ControllerInterface {
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
  const int num_joints = 7;
  Vector7d external_torque_command_;
  bool got_new_command_{false};
  Vector7d q_;
  Vector7d dq_;

  // KDL-related members
  std::string base_link_;
  std::string tip_link_;
  std::string robot_description_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_solver_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  void commandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace fr3_controllers
