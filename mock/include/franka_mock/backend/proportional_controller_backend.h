#pragma once
#include "backend_interface.h"
#include <franka_mock/common.h>

#include <Eigen/Core>
#include <mutex>

namespace aescape {
// Same for all joints. StiffnessGain = 100.0 in the control repo.
constexpr double kReverseStiffnessGain = 1.0 / 100.0;  //NOLINT
constexpr double kDefaultKp = 0.01;  // Default proportional gain. 1.0 in the control repo.

/*
 * Proportional Controller Backend - Reverse engineer what the control repo does.
 * Treat tau as position error. First reverse the stiffness gain and then
 * scaled tau by a given proportional gain.
 */
class ProportionalControllerBackend final: public BackendInterface {
 public:
  explicit ProportionalControllerBackend(double kp = kDefaultKp);

  // Constructor to build the model and set the initial joint positions
  explicit ProportionalControllerBackend(const Eigen::VectorXd& q, double kp = kDefaultKp);

  bool updateRobotState(research_interface::robot::RobotState& state) override;
  bool updateRobotCommand(const research_interface::robot::RobotCommand& cmd) override;

 private:
  RobotState robot_state_;
  std::mutex mut_;
  double kp_;
};
}  // namespace aescape