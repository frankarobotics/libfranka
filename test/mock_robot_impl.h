// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <gmock/gmock.h>

#include <robot_impl.h>

using namespace franka;
using namespace research_interface;

class RobotImplMock : public Robot::Impl {
 public:
  virtual ~RobotImplMock() = default;
  RobotImplMock(std::unique_ptr<Network> network, size_t log_size, RealtimeConfig realtime_config)
      : Robot::Impl(std::move(network), log_size, realtime_config){};
  MOCK_METHOD(uint32_t,
              startMotion,
              (research_interface::robot::Move::ControllerMode,
               research_interface::robot::Move::MotionGeneratorMode,
               const research_interface::robot::Move::Deviation&,
               const research_interface::robot::Move::Deviation&,
               bool,
               const std::optional<std::vector<double>>&),
              (override));
  MOCK_METHOD(void,
              finishMotion,
              (uint32_t,
               const std::optional<research_interface::robot::MotionGeneratorCommand>&,
               const std::optional<research_interface::robot::ControllerCommand>&),
              (override));
  MOCK_METHOD(void, cancelMotion, (uint32_t), (override));
  MOCK_METHOD1(writeOnce, void(const Torques& control_input));
  MOCK_METHOD1(writeOnce, void(const JointPositions& motion_input));
  MOCK_METHOD1(writeOnce, void(const JointVelocities& motion_input));
  MOCK_METHOD1(writeOnce, void(const CartesianPose& motion_input));
  MOCK_METHOD1(writeOnce, void(const CartesianVelocities& motion_input));
  MOCK_METHOD2(writeOnce, void(const JointPositions& motion_input, const Torques& control_input));
  MOCK_METHOD2(writeOnce, void(const JointVelocities& motion_input, const Torques& control_input));
  MOCK_METHOD2(writeOnce, void(const CartesianPose& motion_input, const Torques& control_input));
  MOCK_METHOD2(writeOnce,
               void(const CartesianVelocities& motion_input, const Torques& control_input));
  MOCK_METHOD0(readOnce, RobotState());
  MOCK_METHOD2(throwOnMotionError, void(const RobotState& robot_state, uint32_t motion_id));
};
