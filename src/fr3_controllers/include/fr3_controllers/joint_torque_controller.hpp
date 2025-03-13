#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <Eigen/Eigen>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers {

class JointTorqueController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  std::string arm_id_;
  const int num_joints = 7;
  Vector7d torque_external; 
};
}  // namespace fr3_controllers
