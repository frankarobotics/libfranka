// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <optional>

#include <franka/robot_state.h>

#include "robot_control.h"

class MockRobotControl : public franka::RobotControl {
 public:
  virtual ~MockRobotControl() = default;

  MOCK_METHOD(uint32_t,
              startMotion,
              (research_interface::robot::Move::ControllerMode,
               research_interface::robot::Move::MotionGeneratorMode,
               const research_interface::robot::Move::Deviation&,
               const research_interface::robot::Move::Deviation&,
               bool,
               const std::optional<std::vector<double>>&),
              (override));
  MOCK_METHOD(franka::RobotState,
              updateMotion,
              (const std::optional<research_interface::robot::MotionGeneratorCommand>&,
               const std::optional<research_interface::robot::ControllerCommand>&),
              (override));
  MOCK_METHOD(void,
              finishMotion,
              (uint32_t,
               const std::optional<research_interface::robot::MotionGeneratorCommand>&,
               const std::optional<research_interface::robot::ControllerCommand>&),
              (override));
  MOCK_METHOD(void, cancelMotion, (uint32_t), (override));
  MOCK_METHOD(void, throwOnMotionError, (const franka::RobotState&, uint32_t), (override));
  MOCK_METHOD((std::array<double, 7>),
              getUpperJointVelocityLimits,
              ((const std::array<double, 7>&)),
              (const, override));
  MOCK_METHOD((std::array<double, 7>),
              getLowerJointVelocityLimits,
              ((const std::array<double, 7>&)),
              (const, override));

  franka::RealtimeConfig realtimeConfig() const noexcept override {
    return franka::RealtimeConfig::kIgnore;
  }
};
