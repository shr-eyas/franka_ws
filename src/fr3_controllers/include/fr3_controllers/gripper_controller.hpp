#pragma once

#include <string>
#include <memory>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <teleop_interfaces/msg/joint_group.hpp>
#include <teleop_interfaces/msg/teleop_motion_plan.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;


namespace fr3_controllers {

class GripperController : public controller_interface::ControllerInterface {
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
  std::unique_ptr<franka::Gripper> gripper_;
  std::string arm_id_;
  std::string robot_ip_;
  std::string robot_description_;
  rclcpp::Subscription<teleop_interfaces::msg::TeleopMotionPlan>::SharedPtr motion_plan_;

  void teleop_motion_plan_callback(const teleop_interfaces::msg::TeleopMotionPlan::SharedPtr msg);
  bool got_teleop_motion;
  float width;
  float prev_width;
  float speed;
  float force;
};

}  