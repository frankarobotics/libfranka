// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka/control_types.h>
#include <research_interface/robot/service_types.h>

namespace franka {

template <typename T>
struct MotionGeneratorTraits {};

template <>
struct MotionGeneratorTraits<JointPositions> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::robot::Move::MotionGeneratorMode::kJointPosition;
};

template <>
struct MotionGeneratorTraits<JointVelocities> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::robot::Move::MotionGeneratorMode::kJointVelocity;
};

template <>
struct MotionGeneratorTraits<CartesianPose> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::robot::Move::MotionGeneratorMode::kCartesianPosition;
};

template <>
struct MotionGeneratorTraits<CartesianVelocities> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::robot::Move::MotionGeneratorMode::kCartesianVelocity;
};

template <>
struct MotionGeneratorTraits<Torques> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::robot::Move::MotionGeneratorMode::kNone;
};

}  // namespace franka
