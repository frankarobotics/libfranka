// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/async_control/async_position_control_handler.hpp>
#include <franka/logging/cout_logging_sink.hpp>
#include <franka/logging/logger.hpp>

#include "examples_common.h"

using namespace std::chrono_literals;

const std::vector<double> kDefaultMaximumVelocities{0.655, 0.655, 0.655, 0.655,
                                                    1.315, 1.315, 1.315};
constexpr double kDefaultGoalTolerance = 10.0;

std::atomic<bool> motion_finished =  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    false;

// Capture SIGINT to stop the example gracefully
void signalHandler(int signal) {
  if (signal == SIGINT) {
    motion_finished = true;
  }
}

/**
 * @example The async position control example features a lower frequency control loop which sends
 *   the joint position to the robot. The joint position controller always assumes (for now) that
 *   the user wants to have a zero velocity and acceleration at the target position. Hence, it acts
 *   as a point-to-point control.
 *   Important note: This feature is experimental. Hence, we can't gurantee a stable interface yet.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  std::signal(SIGINT, signalHandler);

  // Initialize the std::cout logger
  auto cout_logger = std::make_shared<franka::CoutLoggingSink>();
  franka::logging::addLogger(cout_logger);

  std::shared_ptr<franka::Robot> robot;
  try {
    robot = std::make_shared<franka::Robot>(argv[1], franka::RealtimeConfig::kIgnore);
  } catch (const std::exception& exception) {
    franka::logging::logError("Could not connect to robot: {}", exception.what());
    return -1;
  }
  setDefaultBehavior(*robot);

  std::array<double, 7> initial_position{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  std::chrono::duration<double> time = 0.0s;
  auto direction = 1.0;
  auto calculate_joint_position_target = [&initial_position, &time,
                                          &direction](std::chrono::milliseconds period)
      -> franka::AsyncPositionControlHandler::JointPositionTarget {
    time += period;

    static auto time_since_last_log = std::chrono::duration<double>::zero();

    std::array<double, 7> target_positions;
    for (size_t i = 0; i < target_positions.size(); ++i) {
      target_positions[i] = initial_position[i] + direction * 0.25;
    }

    time_since_last_log += period;
    if (time_since_last_log >= 1s) {
      // Turn around every second
      direction *= -1.0;
      time_since_last_log = 0s;
    }

    return franka::AsyncPositionControlHandler::JointPositionTarget{.joint_positions =
                                                                        target_positions};
  };

  // The 'robot' object belongs for the time being now to the handler
  franka::AsyncPositionControlHandler::Configuration joint_position_control_configuration{
      .maximum_joint_velocities = kDefaultMaximumVelocities,
      .goal_tolerance = kDefaultGoalTolerance};
  auto async_position_control_handler_result =
      franka::AsyncPositionControlHandler::configure(robot, joint_position_control_configuration);

  if (async_position_control_handler_result.error_message.has_value()) {
    franka::logging::logError(async_position_control_handler_result.error_message.value());
    return -1;
  }

  auto position_control_handler = async_position_control_handler_result.handler;
  auto target_feedback = position_control_handler->getTargetFeedback();

  // This runs the control with 50 Hz
  auto time_step = 20ms;
  while (!motion_finished) {
    auto now = std::chrono::steady_clock::now();

    // Receive some target_feedback
    target_feedback = position_control_handler->getTargetFeedback();

    if (target_feedback.error_message.has_value()) {
      franka::logging::logError(target_feedback.error_message.value());
      return -1;
    }

    // Send the next target
    auto next_joint_position_target = calculate_joint_position_target(time_step);
    auto command_result =
        position_control_handler->setJointPositionTarget(next_joint_position_target);

    if (command_result.error_message.has_value()) {
      franka::logging::logError(command_result.error_message.value());
      return -1;
    }

    // Stop after 10 seconds
    if (time > 10s) {
      position_control_handler->stopControl();
      motion_finished = true;
      break;
    }

    std::this_thread::sleep_until(now + time_step);
  }

  return 0;
}
