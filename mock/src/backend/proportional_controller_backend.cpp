#include <franka_mock/backend/proportional_controller_backend.h>
#include <iostream>

namespace aescape {

ProportionalControllerBackend::ProportionalControllerBackend(double kp) {
  robot_state_.q = Eigen::VectorXd::Zero(kNumJoints);
  kp_ = kp;
}

ProportionalControllerBackend::ProportionalControllerBackend(const Eigen::VectorXd& q, double kp) {
  robot_state_.q = q;
  kp_ = kp;
}

bool ProportionalControllerBackend::updateRobotState(research_interface::robot::RobotState& state) {
  std::lock_guard<std::mutex> lck(mut_);

  for (int i = 0; i < robot_state_.q.size(); ++i) {
    state.q[i] = robot_state_.q(i);
    // state.dq[i] = robot_state_.dq(i); // zero velocities to limit velocity error in tau calulation in control repo
    // state.dq_d[i] = robot_state_.dq(i); // zero velocities to limit velocity error in tau calulation in control repo
    state.tau_J[i] = robot_state_.tau_total(i);
    state.tau_J_d[i] = robot_state_.tau(i);
  }
  return true;
}

RobotState updateRobotStateTauAsPositionError(const RobotState& prev_robot_state,
                                              const Eigen::VectorXd& tau,
                                              double kp) {
  assert(prev_robot_state.q.size() == kNumJoints);
  assert(tau.size() == kNumJoints);

  RobotState new_robot_state;
  new_robot_state.q = prev_robot_state.q + kp * kReverseStiffnessGain * tau;
  new_robot_state.tau = tau;
  return new_robot_state;
}

bool ProportionalControllerBackend::updateRobotCommand(
    const research_interface::robot::RobotCommand& cmd) {
  // Update robot state
  Eigen::VectorXd tau(kNumJoints);
  for (size_t i = 0; i < cmd.control.tau_J_d.size(); ++i) {
    tau[i] = cmd.control.tau_J_d[i];
  }
  std::lock_guard<std::mutex> lck(mut_);
  robot_state_ = updateRobotStateTauAsPositionError(robot_state_, tau, kp_);

  return true;
}

}  // namespace aescape
