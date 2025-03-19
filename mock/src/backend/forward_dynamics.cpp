#include <franka_mock/backend/forward_dynamics.h>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace aescape {
RobotState updateStateValues(const pinocchio::Model& model,
                             const RobotState& prev_robot_state,
                             const Eigen::VectorXd& tau,
                             const Eigen::VectorXd& tau0,
                             const Eigen::MatrixXd& Minv,
                             double dt) {
  RobotState new_robot_state;
  new_robot_state.tau = tau;
  new_robot_state.tau_total = tau + tau0;
  new_robot_state.ddq = Minv * (tau);
  new_robot_state.dq = prev_robot_state.dq + new_robot_state.ddq * dt;
  new_robot_state.q = pinocchio::integrate(model, prev_robot_state.q, new_robot_state.dq);
  return new_robot_state;
}

RobotState updateRobotStateABA(const pinocchio::Model& model,
                               const RobotState& prev_robot_state,
                               const Eigen::VectorXd& tau,
                               double dt) {
  assert(tau.size() == kNumJoints);
  assert(prev_robot_state.q.size() == kNumJoints);
  assert(prev_robot_state.dq.size() == kNumJoints);
  assert(prev_robot_state.ddq.size() == kNumJoints);

  pinocchio::Data data(model);
  const auto Minv = pinocchio::computeMinverse(model, data, prev_robot_state.q);
  const auto tau0 = pinocchio::computeGeneralizedGravity(model, data, prev_robot_state.q);

  RobotState new_robot_state = updateStateValues(model, prev_robot_state, tau, tau0, Minv, dt);
  return new_robot_state;
}

RobotState updateRobotStateCRBA(const pinocchio::Model& model,
                                const RobotState& prev_robot_state,
                                const Eigen::VectorXd& tau,
                                double dt) {
  assert(tau.size() == kNumJoints);
  assert(prev_robot_state.q.size() == kNumJoints);
  assert(prev_robot_state.dq.size() == kNumJoints);
  assert(prev_robot_state.ddq.size() == kNumJoints);

  pinocchio::Data data(model);
  const auto M = pinocchio::crba(model, data, prev_robot_state.q);
  const auto tau0 = pinocchio::computeGeneralizedGravity(model, data, prev_robot_state.q);

  RobotState new_robot_state =
      updateStateValues(model, prev_robot_state, tau, tau0, M.inverse(), dt);
  return new_robot_state;
}

RobotState updateRobotState(const pinocchio::Model& model,
                            const RobotState& prev_robot_state,
                            const Eigen::VectorXd& tau,
                            double dt,
                            InertiaMatrixMethod method) {
  RobotState new_robot_state;
  if (method == InertiaMatrixMethod::kCRBA) {
    new_robot_state = updateRobotStateCRBA(model, prev_robot_state, tau, dt);
  } else if (method == InertiaMatrixMethod::kABA) {
    new_robot_state = updateRobotStateABA(model, prev_robot_state, tau, dt);
  }
  return new_robot_state;
}

}  // namespace aescape