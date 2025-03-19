#include "logging_macros.h"

#include <franka_mock/backend/forward_dynamics_backend.h>
#include <franka_mock/backend/forward_dynamics.h>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace aescape {
ForwardDynamicsBackend::ForwardDynamicsBackend(const std::string& urdf_path) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  robot_state_.q = pinocchio::neutral(model_);
}
ForwardDynamicsBackend::ForwardDynamicsBackend(const std::string& urdf_path,
                                               const Eigen::VectorXd& q) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  robot_state_.q = q;
}

bool ForwardDynamicsBackend::updateRobotState(research_interface::robot::RobotState& state) {
  std::lock_guard<std::mutex> lck(mut_);

  for (int i = 0; i < robot_state_.q.size(); ++i) {
    state.q[i] = robot_state_.q(i);
    state.dq[i] = robot_state_.dq(i);
    state.dq_d[i] = robot_state_.dq(i);
    state.tau_J[i] = robot_state_.tau_total(i);
    state.tau_J_d[i] = robot_state_.tau(i);
  }
  return true;
}

bool ForwardDynamicsBackend::updateRobotCommand(
    const research_interface::robot::RobotCommand& cmd) {
  // Update robot state
  Eigen::VectorXd tau(kNumJoints);
  for (size_t i = 0; i < cmd.control.tau_J_d.size(); ++i) {
    tau[i] = cmd.control.tau_J_d[i];
  }
  std::lock_guard<std::mutex> lck(mut_);
  robot_state_ = aescape::updateRobotState(model_, robot_state_, tau, kDeltaTime);
  return true;
}

}  // namespace aescape
