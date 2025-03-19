#pragma once
#include <Eigen/Core>

namespace aescape {
constexpr int kNumJoints = 7;
constexpr double kDeltaTime = 0.001;

// Franka limits
const Eigen::VectorXd kQMax =
    (Eigen::VectorXd(kNumJoints) << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
        .finished();
const Eigen::VectorXd kQMin =
    (Eigen::VectorXd(kNumJoints) << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
        .finished();
const Eigen::VectorXd kDqMax =
    (Eigen::VectorXd(kNumJoints) << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100)
        .finished();
const Eigen::VectorXd kDqMin = -kDqMax;
const Eigen::VectorXd kTauMax =
    (Eigen::VectorXd(kNumJoints) << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0).finished();
const Eigen::VectorXd kTauMin = -kTauMax;
const Eigen::VectorXd kDdqMax =
    (Eigen::VectorXd(kNumJoints) << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0).finished();
const Eigen::VectorXd kDdqMin = -kDdqMax;

/*
 * Robot state containing joint positions, velocities, accelerations and torques
 */
struct RobotState {
  Eigen::VectorXd q{Eigen::VectorXd::Zero(kNumJoints)};          // joint positions
  Eigen::VectorXd dq{Eigen::VectorXd::Zero(kNumJoints)};         // joint velocities
  Eigen::VectorXd ddq{Eigen::VectorXd::Zero(kNumJoints)};        // joint accelerations
  Eigen::VectorXd tau{Eigen::VectorXd::Zero(kNumJoints)};        // joint torques
  Eigen::VectorXd tau_total{Eigen::VectorXd::Zero(kNumJoints)};  // total torques
};
}  // namespace aescape
