#include <fr3_controllers/cartesian_impedance_controller.hpp>
#include <fr3_controllers/robot_utils.hpp>
#include <fr3_controllers/pseudo_inversion.h>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace fr3_controllers {

/* Command Interface */
controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

/* State Interface */
controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

/*  On Init */
CallbackReturn CartesianImpedanceController::on_init() {
    try {
      // Declare parameters that will be configured via the parameter server.
      auto_declare<std::string>("arm_id", "");
      auto_declare<double>("translational_stiffness", 300.0);
      auto_declare<double>("rotational_stiffness", 50.0);
      auto_declare<double>("nullspace_stiffness", 20.0);
      auto_declare<double>("filter_param", 0.005);
    } catch (const std::exception& e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

/* On Configure */
CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    
    arm_id_ = get_node()->get_parameter("arm_id").as_string();

    translational_stiffness_ = get_node()->get_parameter("translational_stiffness").as_double();
    rotational_stiffness_ = get_node()->get_parameter("rotational_stiffness").as_double();
    nullspace_stiffness_ = get_node()->get_parameter("nullspace_stiffness").as_double();
    filter_param_ = get_node()->get_parameter("filter_param").as_double();

    // Initialize the Cartesian stiffness matrix.
    cartesian_stiffness_.setZero();
    cartesian_stiffness_.topLeftCorner(3, 3) = translational_stiffness_ * Eigen::Matrix3d::Identity();
    cartesian_stiffness_.bottomRightCorner(3, 3) = rotational_stiffness_ * Eigen::Matrix3d::Identity();

    // Damping matrices (damping ratio = 1).
    cartesian_damping_.setZero();
    cartesian_damping_.topLeftCorner(3, 3) = 2.0 * sqrt(translational_stiffness_) * Eigen::Matrix3d::Identity();
    cartesian_damping_.bottomRightCorner(3, 3) = 2.0 * sqrt(rotational_stiffness_) * Eigen::Matrix3d::Identity();
    
    nullspace_stiffness_ = nullspace_stiffness_target_;

    // Initialize desired Cartesian pose to a neutral value.
    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_ = position_d_;
    orientation_d_target_ = orientation_d_;

    // Initialize joint state vectors.
    q_.setZero();
    dq_.setZero();
    q_d_nullspace_.setZero();

    // Create the Franka robot model.
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        arm_id_ + "/" + k_robot_model_interface_name,
        arm_id_ + "/" + k_robot_state_interface_name);

    return CallbackReturn::SUCCESS;
}

/*  On Activate */
CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {

    // Assign state interfaces from the robot model.
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    // Initialize joint state vectors.
    q_.setZero();
    dq_.setZero();
    tau_previous_.setZero();
    
    // Initialize nullspace target to current joint positions.
    updateJointStates();
    q_d_nullspace_ = q_;
        
    // Set the desired Cartesian pose to the current end-effector pose.
    std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    Eigen::Matrix4d pose_matrix = Eigen::Map<Eigen::Matrix4d>(pose_array.data());
    Eigen::Affine3d current_transform(pose_matrix);
    position_d_ = current_transform.translation();
    orientation_d_ = Eigen::Quaterniond(current_transform.rotation());
    
    // Create subscription for external equilibrium pose commands.
    eq_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "equilibrium_pose", rclcpp::SystemDefaultsQoS(),
        std::bind(&CartesianImpedanceController::equilibriumPoseCallback, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
}

/*  Update */
controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {

    // Update joint states (positions and velocities) from state interfaces.
    updateJointStates();

    // Get current end-effector pose.
    std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    Eigen::Matrix4d pose_matrix = Eigen::Map<Eigen::Matrix4d>(pose_array.data());
    Eigen::Affine3d current_transform(pose_matrix);
    Eigen::Vector3d current_position = current_transform.translation();
    Eigen::Quaterniond current_orientation(current_transform.rotation());

    // Retrieve Jacobian and Coriolis force from the robot model.
    std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Matrix<double, 6, 7> jacobian = Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
    std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
    Vector7d coriolis = Eigen::Map<Vector7d>(coriolis_array.data());

    // Compute Cartesian error: position and orientation error.
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) = current_position - position_d_;
    // Ensure consistent quaternion sign.
    if (current_orientation.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
        current_orientation.coeffs() = -current_orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion = current_orientation.inverse() * orientation_d_;
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Rotate orientation error into the base frame.
    error.tail(3) = -current_transform.rotation() * error.tail(3);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // Compute task-space torque: tau_task = J^T * (-K*error - D*(J*dq))
    Vector7d tau_task = jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq_));

    // nullspace PD control with damping ratio = 1
    Vector7d tau_nullspace = (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                        (nullspace_stiffness_ * (q_d_nullspace_ - q_) - (2.0 * sqrt(nullspace_stiffness_)) * dq_);
    // Desired torque
    // Vector7d tau_d = tau_task + tau_nullspace + coriolis;
    Vector7d tau_d = tau_task + tau_nullspace ;

    // Saturate the torque rate to avoid discontinuities.
    Vector7d tau_d_saturated = saturateTorqueRate(tau_d, tau_previous_);
    tau_previous_ = tau_d_saturated;

    // Command the computed torques.
    for (int i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(tau_d_saturated(i));
    }

    return controller_interface::return_type::OK;
}

void CartesianImpedanceController::equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Update the desired Cartesian pose (equilibrium pose) from the received message.
    position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond new_orientation(msg->pose.orientation.w,
                                       msg->pose.orientation.x,
                                       msg->pose.orientation.y,
                                       msg->pose.orientation.z);
    // Ensure consistent quaternion sign.
    if (orientation_d_.coeffs().dot(new_orientation.coeffs()) < 0.0) {
      new_orientation.coeffs() = -new_orientation.coeffs();
    }
    orientation_d_ = new_orientation;
}

CartesianImpedanceController::Vector7d 
CartesianImpedanceController::saturateTorqueRate(
    const Vector7d & tau_d_calculated, 
    const Vector7d & tau_previous) {
  
    Vector7d tau_d_saturated;
    for (int i = 0; i < 7; ++i) {
        double diff = tau_d_calculated(i) - tau_previous(i);
        tau_d_saturated(i) = tau_previous(i) + std::max(std::min(diff, delta_tau_max), -delta_tau_max);
    }
    return tau_d_saturated;
}


void CartesianImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    franka_robot_model_->release_interfaces();
    return CallbackReturn::SUCCESS;
}

}  // namespace fr3_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)