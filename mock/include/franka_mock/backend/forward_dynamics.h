#pragma once

#include <pinocchio/multibody/model.hpp>
#include <franka_mock/common.h>

namespace aescape {
/*
 * Inertia matrix method to use for forward dynamics
 */
enum class InertiaMatrixMethod {
  kABA,
  kCRBA,
};

/*
 * Update robot state using articulated body algorithm(ABA) method. Uses unconstrained forward dynamics
 *                ddq = ABA(q,v,τ,f)
 *                s.t. M(q)ddq + C(q,dq)dq = τ + τg(q) + J(q)f
 * Note with ABA it may not satisfy the holonomic contact constraint J(q)ddq + dJ(q,dq)dq = 0
 * where M(q) is the joint-space inertia matrix
 *       C(q,dq) is the Coriolis matrix,
 *       τ is the joint torques,
 *       τg(q) is the gravity torques,
 *       J(q) is the contact Jacobian,
 *       f is the contact forces.
 * The equation simplifies to M(q)ddq = τ - τ0 with the bias torques τ0 = C(q,dq)dq - τg(q) - J(q)f.
 * Here tau provided is (τ - τ0) and the new joint accelerations are calculated as ddq = M(q)^-1(tau).
 * @param model pinocchio model
 * @param prev_robot_state robot state at previous time step
 * @param tau joint torques minius bias torques
 * @param dt time step
 * @return new robot state
 */
RobotState updateRobotStateABA(const pinocchio::Model& model,
                               const RobotState& prev_robot_state,
                               const Eigen::VectorXd& tau,
                               double dt);

/*
 * Update robot state using composite rigid body algorithm(CRBA) method. Uses constrained forward dynamics
 *                ddq = CRBA(q,v,τ,f)
 *                s.t. M(q)ddq + C(q,dq)dq = τ + τg(q) + J(q)f and
 *                     J(q)ddq + dJ(q,dq)dq = 0
 * All calculations are done similarly as in updateRobotStateABA with M(q) calculated using CRBA.
 * @param model pinocchio model
 * @param prev_robot_state robot state at previous time step
 * @param tau joint torques with gravity torques removed
 * @param dt time step
 * @return new robot state
 */
RobotState updateRobotStateCRBA(const pinocchio::Model& model,
                                const RobotState& prev_robot_state,
                                const Eigen::VectorXd& tau,
                                double dt);

/* Update robot state using InertiaMatrixMethod specified
 * @param model pinocchio model
 * @param prev_robot_state robot state at previous time step
 * @param tau joint torques
 * @param dt time step
 * @param method inertia matrix method
 * @return new robot state
 */
RobotState updateRobotState(const pinocchio::Model& model,
                            const RobotState& prev_robot_state,
                            const Eigen::VectorXd& tau,
                            double dt,
                            InertiaMatrixMethod method = InertiaMatrixMethod::kCRBA);

}  // namespace aescape