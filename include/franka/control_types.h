// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

/**
 * @file control_types.h
 * Contains helper types for returning motion generation and joint-level torque commands.
 */

namespace franka {

/**
 * Available controller modes for a franka::Robot.
 */
enum class ControllerMode { kJointImpedance, kCartesianImpedance };

/**
 * Used to decide whether to enforce realtime mode for a control loop thread.
 *
 * @see Robot::Robot
 */
enum class RealtimeConfig { kEnforce, kIgnore };

/**
 * Helper type for control and motion generation loops.
 *
 * Used to determine whether to terminate a loop after the control callback has returned.
 *
 * @see @ref callback-docs "Documentation on callbacks"
 */
struct Finishable {
  /**
   * Determines whether to finish a currently running motion.
   */
  bool motion_finished = false;
};

/**
 * Stores joint-level torque commands without gravity and friction.
 */
class Torques : public Finishable {
 public:
  /**
   * Creates a new Torques instance.
   *
   * @param[in] torques Desired joint-level torques without gravity and friction in [Nm].
   */
  Torques(const std::array<double, 7>& torques) noexcept;

  /**
   * Creates a new Torques instance.
   *
   * @param[in] torques Desired joint-level torques without gravity and friction in [Nm].
   *
   * @throw std::invalid_argument if the given initializer list has an invalid number of arguments.
   */
  Torques(std::initializer_list<double> torques);

  /**
   * Desired torques in [Nm].
   */
  std::array<double, 7> tau_J{};  // NOLINT(readability-identifier-naming)
};

/**
 * Stores values for joint position motion generation.
 */
class JointPositions : public Finishable {
 public:
  /**
   * Creates a new JointPositions instance.
   *
   * @param[in] joint_positions Desired joint angles in [rad].
   */
  JointPositions(const std::array<double, 7>& joint_positions) noexcept;

  /**
   * Creates a new JointPositions instance.
   *
   * @param[in] joint_positions Desired joint angles in [rad].
   *
   * @throw std::invalid_argument if the given initializer list has an invalid number of arguments.
   */
  JointPositions(std::initializer_list<double> joint_positions);

  /**
   * Desired joint angles in [rad].
   */
  std::array<double, 7> q{};
};

/**
 * Stores values for joint velocity motion generation.
 */
class JointVelocities : public Finishable {
 public:
  /**
   * Creates a new JointVelocities instance.
   *
   * @param[in] joint_velocities Desired joint velocities in [rad/s].
   *
   */
  JointVelocities(const std::array<double, 7>& joint_velocities) noexcept;

  /**
   * Creates a new JointVelocities instance.
   *
   * @param[in] joint_velocities Desired joint velocities in [rad/s].
   *
   * @throw std::invalid_argument if the given initializer list has an invalid number of arguments.
   */
  JointVelocities(std::initializer_list<double> joint_velocities);

  /**
   * Desired joint velocities in [rad/s].
   */
  std::array<double, 7> dq{};
};

/**
 * Stores values for Cartesian pose motion generation.
 */
class CartesianPose : public Finishable {
 public:
  /**
   * Creates a new CartesianPose instance.
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation matrix \f$^O
   * {\mathbf{T}_{EE}}_{d}\f$, column major, that transforms from the end effector frame \f$EE\f$ to
   * base frame \f$O\f$. Equivalently, it is the desired end effector pose in base frame.
   */
  CartesianPose(const std::array<double, 16>& cartesian_pose) noexcept;

  /**
   * Creates a new CartesianPose instance.
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation matrix \f$^O
   * {\mathbf{T}_{EE}}_{d}\f$, column major, that transforms from the end effector frame \f$EE\f$ to
   * base frame \f$O\f$. Equivalently, it is the desired end effector pose in base frame.
   * @param[in] elbow Elbow configuration (see @ref elbow member for more details).
   */
  CartesianPose(const std::array<double, 16>& cartesian_pose,
                const std::array<double, 2>& elbow) noexcept;

  /**
   * Creates a new CartesianPose instance.
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation matrix \f$^O
   * {\mathbf{T}_{EE}}_{d}\f$, column major, that transforms from the end effector frame \f$EE\f$ to
   * base frame \f$O\f$. Equivalently, it is the desired end effector pose in base frame.
   *
   * @throw std::invalid_argument if the given initializer list has an invalid number of arguments.
   */
  CartesianPose(std::initializer_list<double> cartesian_pose);

  /**
   * Creates a new CartesianPose instance.
   *
   * @param[in] cartesian_pose Desired vectorized homogeneous transformation matrix \f$^O
   * {\mathbf{T}_{EE}}_{d}\f$, column major, that transforms from the end effector frame \f$EE\f$ to
   * base frame \f$O\f$. Equivalently, it is the desired end effector pose in base frame.
   *
   * @param[in] elbow Elbow configuration (see @ref elbow member for more details).
   *
   * @throw std::invalid_argument if a given initializer list has an invalid number of arguments.
   */
  CartesianPose(std::initializer_list<double> cartesian_pose, std::initializer_list<double> elbow);

  /**
   * Homogeneous transformation \f$^O{\mathbf{T}_{EE}}_{d}\f$, column major, that transforms from
   * the end effector frame \f$EE\f$ to base frame \f$O\f$.
   * Equivalently, it is the desired end effector pose in base frame.
   */
  std::array<double, 16> O_T_EE{};  // NOLINT(readability-identifier-naming)

  /**
   * Elbow configuration.
   *
   * The values of the array are:
   *  - [0] Position of the 3rd joint in [rad].
   *  - [1] Sign of the 4th joint. Can be +1 or -1.
   */
  std::array<double, 2> elbow{};

  /**
   * Determines whether there is a stored elbow configuration.
   *
   * @return True if there is a stored elbow configuration, false otherwise.
   */

  bool hasElbow() const noexcept;
};

/**
 * Stores values for Cartesian velocity motion generation.
 */
class CartesianVelocities : public Finishable {
 public:
  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dy in
   * [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}.
   */
  CartesianVelocities(const std::array<double, 6>& cartesian_velocities) noexcept;

  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dy in
   * [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}.
   * @param[in] elbow Elbow configuration (see @ref elbow member for more details).
   */
  CartesianVelocities(const std::array<double, 6>& cartesian_velocities,
                      const std::array<double, 2>& elbow) noexcept;

  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dy in
   * [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}.
   *
   * @throw std::invalid_argument if the given initializer list has an invalid number of arguments.
   */
  CartesianVelocities(std::initializer_list<double> cartesian_velocities);

  /**
   * Creates a new CartesianVelocities instance.
   *
   * @param[in] cartesian_velocities Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dy in
   * [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}.
   * @param[in] elbow Elbow configuration (see @ref elbow member for more details).
   *
   * @throw std::invalid_argument if a given initializer list has an invalid number of arguments.
   */
  CartesianVelocities(std::initializer_list<double> cartesian_velocities,
                      std::initializer_list<double> elbow);

  /**
   * Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dy in [m/s], dz in [m/s], omegax in
   * [rad/s], omegay in [rad/s], omegaz in [rad/s]}.
   */
  std::array<double, 6> O_dP_EE{};  // NOLINT(readability-identifier-naming)

  /**
   * Elbow configuration.
   *
   * The values of the array are:
   *  - [0] Position of the 3rd joint in [rad].
   *  - [1] Sign of the 4th joint. Can be +1 or -1.
   */
  std::array<double, 2> elbow{};

  /**
   * Determines whether there is a stored elbow configuration.
   *
   * @return True if there is a stored elbow configuration, false otherwise.
   */
  bool hasElbow() const noexcept;
};

/**
 * Helper method to indicate that a motion should stop after processing the given command.
 *
 * @param[in] command Last command to be executed before the motion terminates.
 *
 * @return Command with motion_finished set to true.
 *
 * @see @ref callback-docs "Documentation on callbacks"
 */
inline Torques MotionFinished(Torques command) noexcept {  // NOLINT(readability-identifier-naming)
  command.motion_finished = true;
  return command;
}

/**
 * Helper method to indicate that a motion should stop after processing the given command.
 *
 * @param[in] command Last command to be executed before the motion terminates.
 *
 * @return Command with motion_finished set to true.
 *
 * @see @ref callback-docs "Documentation on callbacks"
 */
inline JointPositions MotionFinished(  // NOLINT(readability-identifier-naming)
    JointPositions command) noexcept {
  command.motion_finished = true;
  return command;
}

/**
 * Helper method to indicate that a motion should stop after processing the given command.
 *
 * @param[in] command Last command to be executed before the motion terminates.
 *
 * @return Command with motion_finished set to true.
 *
 * @see @ref callback-docs "Documentation on callbacks"
 */
inline JointVelocities MotionFinished(  // NOLINT(readability-identifier-naming)
    JointVelocities command) noexcept {
  command.motion_finished = true;
  return command;
}

/**
 * Helper method to indicate that a motion should stop after processing the given command.
 *
 * @param[in] command Last command to be executed before the motion terminates.
 *
 * @return Command with motion_finished set to true.
 *
 * @see @ref callback-docs "Documentation on callbacks"
 */
inline CartesianPose MotionFinished(  // NOLINT(readability-identifier-naming)
    CartesianPose command) noexcept {
  command.motion_finished = true;
  return command;
}

/**
 * Helper method to indicate that a motion should stop after processing the given command.
 *
 * @param[in] command Last command to be executed before the motion terminates.
 *
 * @return Command with motion_finished set to true.
 *
 * @see @ref callback-docs "Documentation on callbacks"
 */
inline CartesianVelocities MotionFinished(  // NOLINT(readability-identifier-naming)
    CartesianVelocities command) noexcept {
  command.motion_finished = true;
  return command;
}

}  // namespace franka
