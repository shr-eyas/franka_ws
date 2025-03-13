#pragma once

#include <franka_msgs/srv/set_full_collision_behavior.hpp>

namespace DefaultRobotBehavior {

inline franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr
getDefaultCollisionBehaviorRequest() {
  auto request = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();

  request->lower_torque_thresholds_nominal = {
      25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_torque_thresholds_nominal = {
      35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->lower_torque_thresholds_acceleration = {
      25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_torque_thresholds_acceleration = {
      35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->lower_force_thresholds_nominal = {
      30.0, 30.0, 30.0, 25.0, 25.0, 25.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_force_thresholds_nominal = {
      40.0, 40.0, 40.0, 35.0, 35.0, 35.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->lower_force_thresholds_acceleration = {
      30.0, 30.0, 30.0, 25.0, 25.0, 25.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_force_thresholds_acceleration = {
      40.0, 40.0, 40.0, 35.0, 35.0, 35.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)

  return request;
}

}  // namespace DefaultRobotBehavior
