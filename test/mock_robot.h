// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka/robot.h>

using namespace franka;

class RobotMock : public Robot {
 public:
  ~RobotMock() = default;
  RobotMock(std::shared_ptr<Robot::Impl> robot_impl) : Robot(robot_impl) {};
};
