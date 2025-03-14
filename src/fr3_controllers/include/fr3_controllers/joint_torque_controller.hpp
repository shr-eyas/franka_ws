#pragma once

#include <string>
#include <Eigen/Eigen>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <controller_interface/controller_interface.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers {

class JointTorqueController : public controller_interface::ControllerInterface {

public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    std::string arm_id_;
    const int num_joints = 7;
    std::string robot_description_;
    Vector7d torque_external; 
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"}; 
};

}  // namespace fr3_controllers
