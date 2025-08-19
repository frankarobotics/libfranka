// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/model.h>
#include <franka/robot_model.h>

#include <iostream>

#include <Eigen/Core>

#include <research_interface/robot/service_types.h>

#include <fstream>
#include <sstream>

using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace franka {

Frame operator++(Frame& frame, int /* dummy */) noexcept {
  Frame original = frame;
  frame = static_cast<Frame>(static_cast<std::underlying_type_t<Frame>>(frame) + 1);
  return original;
}

Model::Model(const std::string& urdf_model) {
  robot_model_ = std::make_unique<RobotModel>(urdf_model);
}

// for the tests
Model::Model(std::unique_ptr<RobotModelBase> robot_model) : robot_model_(std::move(robot_model)) {}

// Has to be declared here for proper destruction of robot_model_
Model::~Model() noexcept = default;
Model::Model(Model&&) noexcept = default;
Model& Model::operator=(Model&&) noexcept = default;

std::array<double, 16> Model::pose(Frame frame, const franka::RobotState& robot_state) const {
  return pose(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
}

std::array<double, 16> Model::pose(
    Frame frame,
    const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
    const {
  std::array<double, 16> output;
  switch (frame) {
    case Frame::kJoint1:
      return robot_model_->pose(q, 1);
    case Frame::kJoint2:
      return robot_model_->pose(q, 2);
    case Frame::kJoint3:
      return robot_model_->pose(q, 3);
    case Frame::kJoint4:
      return robot_model_->pose(q, 4);
    case Frame::kJoint5:
      return robot_model_->pose(q, 5);
    case Frame::kJoint6:
      return robot_model_->pose(q, 6);
    case Frame::kJoint7:
      return robot_model_->pose(q, 7);
    case Frame::kFlange:
      return robot_model_->poseFlange(q);
    case Frame::kEndEffector:
      return robot_model_->poseEe(q, F_T_EE);
    case Frame::kStiffness:
      return robot_model_->poseStiffness(q, F_T_EE, EE_T_K);
    default:
      throw std::invalid_argument("Invalid frame given.");
  }

  return output;
}

std::array<double, 42> Model::bodyJacobian(Frame frame,
                                           const franka::RobotState& robot_state) const {
  return bodyJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
}

std::array<double, 42> Model::bodyJacobian(
    Frame frame,
    const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
    const {
  switch (frame) {
    case Frame::kJoint1:
      return robot_model_->bodyJacobian(q, 1);
    case Frame::kJoint2:
      return robot_model_->bodyJacobian(q, 2);
    case Frame::kJoint3:
      return robot_model_->bodyJacobian(q, 3);
    case Frame::kJoint4:
      return robot_model_->bodyJacobian(q, 4);
    case Frame::kJoint5:
      return robot_model_->bodyJacobian(q, 5);
    case Frame::kJoint6:
      return robot_model_->bodyJacobian(q, 6);
    case Frame::kJoint7:
      return robot_model_->bodyJacobian(q, 7);
    case Frame::kFlange:
      return robot_model_->bodyJacobianFlange(q);
    case Frame::kEndEffector:
      return robot_model_->bodyJacobianEe(q, F_T_EE);
    case Frame::kStiffness:
      return robot_model_->bodyJacobianStiffness(q, F_T_EE, EE_T_K);
    default:
      throw std::invalid_argument("Invalid frame given.");
  }
}

std::array<double, 42> Model::zeroJacobian(Frame frame,
                                           const franka::RobotState& robot_state) const {
  return zeroJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
};

std::array<double, 42> Model::zeroJacobian(
    Frame frame,
    const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
    const {
  switch (frame) {
    case Frame::kJoint1:
      return robot_model_->zeroJacobian(q, 1);
    case Frame::kJoint2:
      return robot_model_->zeroJacobian(q, 2);
    case Frame::kJoint3:
      return robot_model_->zeroJacobian(q, 3);
    case Frame::kJoint4:
      return robot_model_->zeroJacobian(q, 4);
    case Frame::kJoint5:
      return robot_model_->zeroJacobian(q, 5);
    case Frame::kJoint6:
      return robot_model_->zeroJacobian(q, 6);
    case Frame::kJoint7:
      return robot_model_->zeroJacobian(q, 7);
    case Frame::kFlange:
      return robot_model_->zeroJacobianFlange(q);
    case Frame::kEndEffector:
      return robot_model_->zeroJacobianEe(q, F_T_EE);
    case Frame::kStiffness:
      return robot_model_->zeroJacobianStiffness(q, F_T_EE, EE_T_K);
    default:
      throw std::invalid_argument("Invalid frame given.");
  }
}

std::array<double, 49> franka::Model::mass(const franka::RobotState& robot_state) const noexcept {
  return mass(robot_state.q, robot_state.I_total, robot_state.m_total, robot_state.F_x_Ctotal);
}

std::array<double, 49> franka::Model::mass(
    const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
    const noexcept {
  std::array<double, 49> output;
  robot_model_->mass(q, I_total, m_total, F_x_Ctotal, output);

  return output;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

std::array<double, 7> franka::Model::coriolis(
    const franka::RobotState& robot_state) const noexcept {
  return coriolis(robot_state.q, robot_state.dq, robot_state.I_total, robot_state.m_total,
                  robot_state.F_x_Ctotal);
}

#pragma GCC diagnostic pop

std::array<double, 7> franka::Model::coriolis(
    const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
    const std::array<double, 7>& dq,       // NOLINT(readability-identifier-length)
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
    const noexcept {
  std::array<double, 7> output;
  robot_model_->coriolis(q, dq, I_total, m_total, F_x_Ctotal, output);

  return output;
}

std::array<double, 7> franka::Model::coriolis(
    const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
    const std::array<double, 7>& dq,       // NOLINT(readability-identifier-length)
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
    const std::array<double, 3>& gravity_earth) const noexcept {
  std::array<double, 7> output;
  robot_model_->coriolis(q, dq, I_total, m_total, F_x_Ctotal, gravity_earth, output);

  return output;
}

std::array<double, 7> franka::Model::gravity(
    const franka::RobotState& robot_state,
    const std::array<double, 3>& gravity_earth) const noexcept {
  return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal, gravity_earth);
};

std::array<double, 7> franka::Model::gravity(const franka::RobotState& robot_state) const noexcept {
  return gravity(robot_state, robot_state.O_ddP_O);
};

std::array<double, 7> franka::Model::gravity(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
    const std::array<double, 3>& gravity_earth) const noexcept {
  std::array<double, 7> output;
  robot_model_->gravity(q, gravity_earth, m_total, F_x_Ctotal, output);

  return output;
}

}  // namespace franka
