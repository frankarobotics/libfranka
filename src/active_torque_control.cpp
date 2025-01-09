// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka/active_torque_control.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "robot_impl.h"

namespace franka {

void ActiveTorqueControl::writeOnce(const Torques& control_input) {
  if (control_finished) {
    throw franka::ControlException("writeOnce must not be called after the motion has finished.");
  }

  if (control_input.motion_finished) {
    robot_impl->finishMotion(motion_id, control_input);
    control_finished = true;
    control_lock.unlock();
    return;
  }

  robot_impl->writeOnce(control_input);
}

}  // namespace franka
