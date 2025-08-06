// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <chrono>
#include <iostream>
#include <thread>

#include <franka/active_control.h>
#include <franka/active_torque_control.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example communication_test.cpp
 * An example indicating the network performance.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  uint64_t counter = 0;
  double avg_success_rate = 0.0;
  double min_success_rate = 1.0;
  double max_success_rate = 0.0;
  uint64_t time = 0;
  std::cout.precision(2);
  std::cout << std::fixed;

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl << std::endl;
    std::cout << "Starting communication test." << std::endl;

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    auto rw_interface = robot.startTorqueControl();

    franka::RobotState robot_state;
    franka::Duration period;

    while (!zero_torques.motion_finished) {
      std::tie(robot_state, period) = rw_interface->readOnce();

      time += period.toMSec();
      if (time == 0) {
        rw_interface->writeOnce(zero_torques);
        continue;
      }
      counter++;

      if (counter % 100 == 0) {
        std::cout << "#" << counter
                  << " Current success rate: " << robot_state.control_command_success_rate
                  << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));

      avg_success_rate += robot_state.control_command_success_rate;
      if (robot_state.control_command_success_rate > max_success_rate) {
        max_success_rate = robot_state.control_command_success_rate;
      }
      if (robot_state.control_command_success_rate < min_success_rate) {
        min_success_rate = robot_state.control_command_success_rate;
      }

      if (time >= 10000) {
        std::cout << std::endl << "Finished test, shutting down example" << std::endl;
        zero_torques.motion_finished = true;
      }
      // Sending zero torques - if EE is configured correctly, robot should not move
      rw_interface->writeOnce(zero_torques);
    }
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  avg_success_rate = avg_success_rate / static_cast<double>(counter);

  std::cout << std::endl
            << std::endl
            << "#######################################################" << std::endl;
  uint64_t lost_robot_states = time - counter;
  if (lost_robot_states > 0) {
    std::cout << "The control loop did not get executed " << lost_robot_states << " times in the"
              << std::endl
              << "last " << time << " milliseconds! (lost " << lost_robot_states << " robot states)"
              << std::endl
              << std::endl;
  }

  std::cout << "Control command success rate of " << counter << " samples: " << std::endl;
  std::cout << "Max: " << max_success_rate << std::endl;
  std::cout << "Avg: " << avg_success_rate << std::endl;
  std::cout << "Min: " << min_success_rate << std::endl;

  if (avg_success_rate < 0.90) {
    std::cout << std::endl
              << "WARNING: THIS SETUP IS PROBABLY NOT SUFFICIENT FOR FCI!" << std::endl;
    std::cout << "PLEASE TRY OUT A DIFFERENT PC / NIC" << std::endl;
  } else if (avg_success_rate < 0.95) {
    std::cout << std::endl << "WARNING: MANY PACKETS GOT LOST!" << std::endl;
    std::cout << "PLEASE INSPECT YOUR SETUP AND FOLLOW ADVICE ON" << std::endl
              << "https://frankarobotics.github.io/docs/troubleshooting.html" << std::endl;
  }
  std::cout << "#######################################################" << std::endl << std::endl;

  return 0;
}
