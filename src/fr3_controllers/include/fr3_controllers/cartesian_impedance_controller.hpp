#pragma once

#include <string>
#include <memory>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers {

class CartesianImpedanceController : public controller_interface::ControllerInterface {

 public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
    bool got_new_pose_{false};

    // Constant: number of joints (for a 7-DOF robot).
    const int num_joints = 7;

    // Controller parameters.
    std::string arm_id_;
    const double delta_tau_max_{1.0};
    double translational_stiffness_{300.0};
    double rotational_stiffness_{50.0};
    double nullspace_stiffness_{20.0};
    double filter_param_{0.005};
    double nullspace_stiffness_target_{20.0};

    // Cartesian impedance control matrices.
    Matrix6d cartesian_stiffness_;
    Matrix6d cartesian_damping_;

    // Desired Cartesian pose and its target.
    Eigen::Vector3d position_d_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Quaterniond orientation_d_target_;

    // Joint state vectors and nullspace target.
    Vector7d q_;
    Vector7d dq_;
    Vector7d q_d_nullspace_;
    Vector7d tau_previous_;

    // Franka robot model for kinematics and dynamics.
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    const std::string k_robot_model_interface_name = "robot_model";
    const std::string k_robot_state_interface_name = "robot_state";

    // Subscriber for external equilibrium Cartesian pose commands.
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr eq_pose_sub_;
    void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Update the joint state vectors from the state interfaces.
    void updateJointStates();

    // Saturate the rate of change of the torque command.
    Vector7d saturateTorqueRate(const Vector7d & tau_d_calculated, const Vector7d & tau_previous);
};

}  // namespace fr3_controllers