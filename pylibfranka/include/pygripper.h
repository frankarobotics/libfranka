// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/**
 * @file pygripper.h
 * @brief Python bindings for the Franka Robotics Gripper Control
 *
 * This header file provides C++ class that wraps the Franka Robotics Gripper Control
 * for use in Python through pybind11. It offers:
 *    - Gripper homing and movement control
 *    - Grasping functionality with configurable parameters
 *    - State reading and control methods
 */

#pragma once

#include <franka/gripper.h>
#include <pybind11/pybind11.h>
#include <memory>
#include <string>

namespace pylibfranka {

class PyGripper {
 public:
  explicit PyGripper(const std::string& franka_address);
  ~PyGripper() = default;

  bool homing();
  bool grasp(double width,
             double speed,
             double force,
             double epsilon_inner = 0.005,
             double epsilon_outer = 0.005);

  franka::GripperState readOnce();

  bool stop();
  bool move(double width, double speed);
  franka::Gripper::ServerVersion serverVersion();

 private:
  std::unique_ptr<franka::Gripper> gripper_;
};

}  // namespace pylibfranka
