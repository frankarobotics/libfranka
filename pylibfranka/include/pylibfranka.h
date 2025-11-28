// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/**
 * @file pylibfranka.h
 * @brief Python bindings for the Franka Robotics Robot Control Library
 *
 * This header file provides C++ classes that wrap the Franka Robotics Robot Control Library
 * for use in Python through pybind11. It offers:
 * PyRobot: A wrapper for the Franka robot control functionality, providing:
 *    - Active control methods (torque, joint position, joint velocity control)
 *    - Configuration methods (collision behavior, impedance settings, etc.)
 *    - State reading and control methods
 */

#pragma once

// C++ standard library headers
#include <array>
#include <memory>
#include <optional>
#include <string>

// Third-party library headers
#include <franka/active_control.h>
#include <franka/active_control_base.h>
#include <franka/active_motion_generator.h>
#include <franka/active_torque_control.h>
#include <franka/control_types.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>

namespace pylibfranka {

class PyRobot {
 public:
  explicit PyRobot(const std::string& robot_ip_address,
                   franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kEnforce);
  ~PyRobot() = default;

  // Active control methods
  /**
   * Starts torque control mode.
   */
  auto startTorqueControl() -> std::unique_ptr<franka::ActiveControlBase>;

  /**
   * Starts the joint position control mode.
   * @param control_type The type of controller to use (JointImpedance or CartesianImpedance).
   */
  auto startJointPositionControl(franka::ControllerMode controller_mode)
      -> std::unique_ptr<franka::ActiveControlBase>;

  /**
   * Starts the joint velocity control mode.
   * @param control_type The type of controller to use (JointImpedance or CartesianImpedance).
   */
  auto startJointVelocityControl(franka::ControllerMode controller_mode)
      -> std::unique_ptr<franka::ActiveControlBase>;

  /**
   * Starts the Cartesian pose control mode.
   * @param control_type The type of controller to use (JointImpedance or CartesianImpedance).
   */
  auto startCartesianPoseControl(franka::ControllerMode controller_mode)
      -> std::unique_ptr<franka::ActiveControlBase>;

  /**
   * Starts the Cartesian velocity control mode.
   * @param control_type The type of controller to use (JointImpedance or CartesianImpedance).
   */
  auto startCartesianVelocityControl(franka::ControllerMode controller_mode)
      -> std::unique_ptr<franka::ActiveControlBase>;

  // Configuration methods
  void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                            const std::array<double, 7>& upper_torque_thresholds,
                            const std::array<double, 6>& lower_force_thresholds,
                            const std::array<double, 6>& upper_force_thresholds);

  void setJointImpedance(const std::array<double, 7>& K_theta);
  void setCartesianImpedance(const std::array<double, 6>& K_x);
  void setK(const std::array<double, 16>& EE_T_K);
  void setEE(const std::array<double, 16>& NE_T_EE);
  void setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,
               const std::array<double, 9>& load_inertia);
  void automaticErrorRecovery();

  // State methods
  franka::RobotState readOnce();
  void stop();

  std::unique_ptr<franka::Model> loadModel();

 private:
  std::unique_ptr<franka::Robot> robot_;
};

}  // namespace pylibfranka
