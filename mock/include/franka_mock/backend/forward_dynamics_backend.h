#pragma once
#include "backend_interface.h"
#include "forward_dynamics.h"

#include <mutex>
#include <pinocchio/multibody/model.hpp>

namespace aescape {
class ForwardDynamicsBackend final : public BackendInterface {
 public:
  const double kDeltaTime = 0.001;  // time step. Assume to run at 1000Hz.

  // Constructor to build the model with the model neutral joint positions
  explicit ForwardDynamicsBackend(const std::string& urdf_path);

  // Constructor to build the model and set the initial joint positions
  explicit ForwardDynamicsBackend(const std::string& urdf_path, const Eigen::VectorXd& q);

  bool updateRobotState(research_interface::robot::RobotState& state) override;
  bool updateRobotCommand(const research_interface::robot::RobotCommand& cmd) override;

 private:
  RobotState robot_state_;
  pinocchio::Model model_;
  std::mutex mut_;
};
}  // namespace aescape