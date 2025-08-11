// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example motion_with_control.cpp
 * An example showing how to use a joint velocity motion generator and torque control.
 *
 * Additionally, this example shows how to capture and write logs in case an exception is thrown
 * during a motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

namespace {

class Controller {
 public:
  Controller(size_t dq_filter_size,
             const std::array<double, 7>& K_P,  // NOLINT(readability-identifier-naming)
             const std::array<double, 7>& K_D)  // NOLINT(readability-identifier-naming)
      : dq_filter_size_(dq_filter_size), K_P_(K_P), K_D_(K_D) {
    std::fill(dq_d_.begin(), dq_d_.end(), 0);
    dq_buffer_ = std::vector<double>(dq_filter_size_ * 7, 0.0);
  }

  inline franka::Torques step(const franka::RobotState& state) {
    updateDQFilter(state);

    std::array<double, 7> tau_J_d;  // NOLINT(readability-identifier-naming)
    for (size_t i = 0; i < 7; i++) {
      tau_J_d[i] = K_P_[i] * (state.q_d[i] - state.q[i]) + K_D_[i] * (dq_d_[i] - getDQFiltered(i));
    }
    return tau_J_d;
  }

  void updateDQFilter(const franka::RobotState& state) {
    for (size_t i = 0; i < 7; i++) {
      dq_buffer_[dq_current_filter_position_ * 7 + i] = state.dq[i];
    }
    dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
  }

  double getDQFiltered(size_t index) const {
    double value = 0;
    for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
      value += dq_buffer_[i];
    }
    return value / static_cast<double>(dq_filter_size_);
  }

 private:
  size_t dq_current_filter_position_{0};
  size_t dq_filter_size_;

  const std::array<double, 7> K_P_;  // NOLINT(readability-identifier-naming)
  const std::array<double, 7> K_D_;  // NOLINT(readability-identifier-naming)

  std::array<double, 7> dq_d_;
  std::vector<double> dq_buffer_;
};

std::vector<double> generateTrajectory(double a_max) {
  // Generating a motion with smooth velocity and acceleration.
  // Squared sine is used for the acceleration/deceleration phase.
  std::vector<double> trajectory;
  constexpr double kTimeStep = 0.001;          // [s]
  constexpr double kAccelerationTime = 1;      // time spend accelerating and decelerating [s]
  constexpr double kConstantVelocityTime = 1;  // time spend with constant speed [s]
  // obtained during the speed up
  // and slow down [rad/s^2]
  double acceleration = 0;  // [rad/s^2]
  double velocity = 0;      // [rad/s]
  double time = 0;          // [s]
  while (time < (2 * kAccelerationTime + kConstantVelocityTime)) {
    if (time <= kAccelerationTime) {
      acceleration = pow(sin(time * M_PI / kAccelerationTime), 2) * a_max;
    } else if (time <= (kAccelerationTime + kConstantVelocityTime)) {
      acceleration = 0;
    } else {
      const double deceleration_time = (kAccelerationTime + kConstantVelocityTime) -
                                       time;  // time spent in the deceleration phase
      acceleration = -pow(sin(deceleration_time * M_PI / kAccelerationTime), 2) * a_max;
    }
    velocity += acceleration * kTimeStep;
    time += kTimeStep;
    trajectory.push_back(velocity);
  }
  return trajectory;
}

}  // anonymous namespace

void writeLogToFile(const std::vector<franka::Record>& log);

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Parameters
  const size_t joint_number{3};
  const size_t filter_size{5};

  // NOLINTNEXTLINE(readability-identifier-naming)
  const std::array<double, 7> K_P{{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}};
  // NOLINTNEXTLINE(readability-identifier-naming)
  const std::array<double, 7> K_D{{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
  const double max_acceleration{1.0};

  Controller controller(filter_size, K_P, K_D);

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
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    size_t index = 0;
    std::vector<double> trajectory = generateTrajectory(max_acceleration);

    robot.control(
        [&](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
          return controller.step(robot_state);
        },
        [&](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {
          index += period.toMSec();

          if (index >= trajectory.size()) {
            index = trajectory.size() - 1;
          }

          franka::JointVelocities velocities{{0, 0, 0, 0, 0, 0, 0}};
          velocities.dq[joint_number] = trajectory[index];

          if (index >= trajectory.size() - 1) {
            return franka::MotionFinished(velocities);
          }
          return velocities;
        });
  } catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    writeLogToFile(e.log);
    return -1;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

void writeLogToFile(const std::vector<franka::Record>& log) {
  if (log.empty()) {
    return;
  }
  try {
    Poco::Path temp_dir_path(Poco::Path::temp());
    temp_dir_path.pushDirectory("libfranka-logs");

    Poco::File temp_dir(temp_dir_path);
    temp_dir.createDirectories();

    std::string now_string =
        Poco::DateTimeFormatter::format(Poco::Timestamp{}, "%Y-%m-%d-%h-%m-%S-%i");
    std::string filename = std::string{"log-" + now_string + ".csv"};
    Poco::File log_file(Poco::Path(temp_dir_path, filename));
    if (!log_file.createFile()) {
      std::cout << "Failed to write log file." << std::endl;
      return;
    }
    std::ofstream log_stream(log_file.path().c_str());
    log_stream << franka::logToCSV(log);

    std::cout << "Log file written to: " << log_file.path() << std::endl;
  } catch (...) {
    std::cout << "Failed to write log file." << std::endl;
  }
}
