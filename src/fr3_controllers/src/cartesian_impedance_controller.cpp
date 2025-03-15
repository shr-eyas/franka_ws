#include <fr3_controllers/cartesian_impedance_controller.hpp>
#include <fr3_controllers/robot_utils.hpp>
#include <fr3_controllers/pseudo_inversion.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace fr3_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  dq_filtered_.setZero();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
    franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                 arm_id_ + "/" + k_robot_state_interface_name));

                                    
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

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  updateJointStates();

  std::array<double, 16> pose_array_init = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Matrix4d pose_matrix_init = Eigen::Map<Eigen::Matrix4d>(pose_array_init.data());
  Eigen::Affine3d init_transform(pose_matrix_init);
  position_d_ = init_transform.translation();
  orientation_d_ = Eigen::Quaterniond(init_transform.rotation());

  dq_filtered_.setZero();
  initial_q_ = q_;
  elapsed_time_ = 0.0;
  RCLCPP_INFO(get_node()->get_logger(), "Hola!");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {

  // 1. Update joint states.
  updateJointStates();
  RCLCPP_DEBUG(get_node()->get_logger(), "Joint positions: [%f, %f, ...]", q_(0), q_(1));
  RCLCPP_DEBUG(get_node()->get_logger(), "Joint velocities: [%f, %f, ...]", dq_(0), dq_(1));

  // 2. Get current end-effector pose.
  std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Matrix4d pose_matrix = Eigen::Map<Eigen::Matrix4d>(pose_array.data());
  Eigen::Affine3d current_transform(pose_matrix);
  Eigen::Vector3d current_position = current_transform.translation();
  Eigen::Quaterniond current_orientation(current_transform.rotation());
  RCLCPP_DEBUG(get_node()->get_logger(), "Current position: (%f, %f, %f)", current_position.x(), current_position.y(), current_position.z());

  // 3. Retrieve Jacobian and Coriolis force.
  std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Matrix<double, 6, 7> jacobian = Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
  RCLCPP_DEBUG(get_node()->get_logger(), "Jacobian (first row): (%f, %f, ...)", jacobian(0,0), jacobian(0,1));
  
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Vector7d coriolis = Eigen::Map<Vector7d>(coriolis_array.data());
  RCLCPP_DEBUG(get_node()->get_logger(), "Coriolis: [%f, %f, ...]", coriolis(0), coriolis(1));

  // 4. Compute Cartesian error.
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
  RCLCPP_DEBUG(get_node()->get_logger(), "Cartesian error norm: %f", error.norm());

  // 5. Compute pseudo-inverse for nullspace handling.
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  RCLCPP_DEBUG(get_node()->get_logger(), "Pseudo-inverse norm: %f", jacobian_transpose_pinv.norm());

  // TEST
  Matrix6d cartesian_stiffness_;

  // Define the stiffness values for each DOF (both translational and rotational).
  double translational_stiffness_x_{0.0};
  double translational_stiffness_y_{300.0};
  double translational_stiffness_z_{300.0};
  double rotational_stiffness_roll_{50.0};
  double rotational_stiffness_pitch_{50.0};
  double rotational_stiffness_yaw_{50.0};

  // Initialize the stiffness matrix with zeros
  cartesian_stiffness_.setZero();

  // Set individual translational stiffness values along the diagonal
  cartesian_stiffness_(0, 0) = translational_stiffness_x_;  // X-axis stiffness
  cartesian_stiffness_(1, 1) = translational_stiffness_y_;  // Y-axis stiffness
  cartesian_stiffness_(2, 2) = translational_stiffness_z_;  // Z-axis stiffness

  // Set individual rotational stiffness values along the diagonal
  cartesian_stiffness_(3, 3) = rotational_stiffness_roll_;  // Roll stiffness
  cartesian_stiffness_(4, 4) = rotational_stiffness_pitch_; // Pitch stiffness
  cartesian_stiffness_(5, 5) = rotational_stiffness_yaw_;   // Yaw stiffness

  // 6. Compute task-space torque.
  Vector7d tau_task = jacobian.transpose() * (-cartesian_stiffness_ * error);
  RCLCPP_DEBUG(get_node()->get_logger(), "tau_task norm: %f", tau_task.norm());

  // Apply the computed task-space torque to the joints
  for (int i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(tau_task(i));
  }

  return controller_interface::return_type::OK;
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

}  // namespace fr3_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)