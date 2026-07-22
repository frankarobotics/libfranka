// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka/active_control.h>
#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <research_interface/robot/rbk_types.h>
#include <franka/logging/logger.hpp>

#include "robot_impl.h"

namespace franka {

ActiveControl::ActiveControl(std::shared_ptr<Robot::Impl> robot_impl,
                             uint32_t motion_id,
                             std::unique_lock<std::mutex> control_lock)
    : robot_impl(std::move(robot_impl)),
      motion_id(motion_id),
      control_lock(std::move(control_lock)) {}

ActiveControl::~ActiveControl() {
  if (control_finished) {
    return;
  }
  // A destructor must not throw. Only the NetworkException from an already-interrupted connection is
  // expected and tolerated; anything else is surfaced (still not rethrown) instead of hidden.
  try {
    robot_impl->cancelMotion(motion_id);
  } catch (const NetworkException&) {
  } catch (const std::exception& e) {
    try {
      logging::logError("libfranka: unexpected error cancelling motion in ActiveControl destructor: {}",
                        e.what());
    } catch (...) {  // NOLINT(bugprone-empty-catch)
    }
  } catch (...) {  // NOLINT(bugprone-empty-catch)
  }
}

std::pair<RobotState, Duration> ActiveControl::readOnce() {
  auto robot_state = robot_impl->readOnce();
  robot_impl->throwOnMotionError(robot_state, motion_id);

  if (!last_read_access.has_value()) {
    last_read_access = robot_state.time;
  }

  auto time_since_last_read = robot_state.time - last_read_access.value();
  last_read_access = robot_state.time;

  return std::make_pair(robot_state, time_since_last_read);
}

}  // namespace franka
