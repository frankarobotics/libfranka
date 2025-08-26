// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <ostream>

#include <franka/duration.h>
#include <franka/errors.h>

/**
 * @file robot_state.h
 * Contains the franka::RobotState types.
 */

namespace franka {

/**
 * Describes the robot's current mode.
 */
enum class RobotMode {
  kOther,
  kIdle,
  kMove,
  kGuiding,
  kReflex,
  kUserStopped,
  kAutomaticErrorRecovery
};

/**
 * Describes the robot state.
 */
struct RobotState {
  /**
   * \f$^{O}T_{EE}\f$
   * Measured end effector pose in @ref o-frame "base frame".
   * Pose is represented as a 4x4 matrix in column-major format.
   */
  std::array<double, 16> O_T_EE{};  // NOLINT(readability-identifier-naming)

  /**
   * \f${^OT_{EE}}_{d}\f$
   * Last desired end effector pose of motion generation in @ref o-frame "base frame".
   * Pose is represented as a 4x4 matrix in column-major format.
   */
  std::array<double, 16> O_T_EE_d{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$^{F}T_{EE}\f$
   * End effector frame pose in flange frame.
   * Pose is represented as a 4x4 matrix in column-major format.
   *
   * @see F_T_NE
   * @see NE_T_EE
   * @see Robot for an explanation of the F, NE and EE frames.
   */
  std::array<double, 16> F_T_EE{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$^{F}T_{NE}\f$
   * Nominal end effector frame pose in flange frame.
   * Pose is represented as a 4x4 matrix in column-major format.
   *
   * @see F_T_EE
   * @see NE_T_EE
   * @see Robot for an explanation of the F, NE and EE frames.
   */
  std::array<double, 16> F_T_NE{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$^{NE}T_{EE}\f$
   * End effector frame pose in nominal end effector frame.
   * Pose is represented as a 4x4 matrix in column-major format.
   *
   * @see Robot::setEE to change this frame.
   * @see F_T_EE
   * @see F_T_NE
   * @see Robot for an explanation of the F, NE and EE frames.
   */
  std::array<double, 16> NE_T_EE{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$^{EE}T_{K}\f$
   * Stiffness frame pose in end effector frame.
   * Pose is represented as a 4x4 matrix in column-major format.
   *
   * See also @ref k-frame "K frame".
   */
  std::array<double, 16> EE_T_K{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$m_{EE}\f$
   * Configured mass of the end effector.
   */
  double m_ee{};

  /**
   * \f$I_{EE}\f$
   * Configured rotational inertia matrix of the end effector load with respect to center of mass.
   */
  std::array<double, 9> I_ee{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$^{F}x_{C_{EE}}\f$
   * Configured center of mass of the end effector load with respect to flange frame.
   */
  std::array<double, 3> F_x_Cee{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$m_{load}\f$
   * Configured mass of the external load.
   */
  double m_load{};

  /**
   * \f$I_{load}\f$
   * Configured rotational inertia matrix of the external load with respect to center of mass.
   */
  std::array<double, 9> I_load{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$^{F}x_{C_{load}}\f$
   * Configured center of mass of the external load with respect to flange frame.
   */
  std::array<double, 3> F_x_Cload{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$m_{total}\f$
   * Sum of the mass of the end effector and the external load.
   */
  double m_total{};

  /**
   * \f$I_{total}\f$
   * Combined rotational inertia matrix of the end effector load and the external load with respect
   * to the center of mass.
   */
  std::array<double, 9> I_total{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$^{F}x_{C_{total}}\f$
   * Combined center of mass of the end effector load and the external load with respect to flange
   * frame.
   */
  std::array<double, 3> F_x_Ctotal{};  // NOLINT(readability-identifier-naming)

  /**
   * Elbow configuration.
   *
   * The values of the array are:
   *  - elbow[0]: Position of the 3rd joint in \f$[rad]\f$.
   *  - elbow[1]: Flip direction of the elbow (4th joint):
   *    - +1 if \f$q_4 > q_{elbow-flip}\f$
   *    - 0 if \f$q_4 == q_{elbow-flip} \f$
   *    - -1 if \f$q_4 < q_{elbow-flip} \f$
   *    .
   *    with \f$q_{elbow-flip}\f$ as specified in the robot interface specification page in the FCI
   * Documentation.
   */
  std::array<double, 2> elbow{};

  /**
   * Desired elbow configuration.
   *
   * The values of the array are:
   *  - elbow_d[0]: Position of the 3rd joint in \f$[rad]\f$.
   *  - elbow_d[1]: Flip direction of the elbow (4th joint):
   *    - +1 if \f$q_4 > q_{elbow-flip}\f$
   *    - 0 if \f$q_4 == q_{elbow-flip} \f$
   *    - -1 if \f$q_4 < q_{elbow-flip} \f$
   *    .
   *    with \f$q_{elbow-flip}\f$ as specified in the robot interface specification page in the FCI
   * Documentation.
   */
  std::array<double, 2> elbow_d{};

  /**
   * Commanded elbow configuration.
   *
   * The values of the array are:
   *  - elbow_c[0]: Position of the 3rd joint in \f$[rad]\f$.
   *  - elbow_c[1]: Flip direction of the elbow (4th joint):
   *    - +1 if \f$q_4 > q_{elbow-flip}\f$
   *    - 0 if \f$q_4 == q_{elbow-flip} \f$
   *    - -1 if \f$q_4 < q_{elbow-flip} \f$
   *    .
   *    with \f$q_{elbow-flip}\f$ as specified in the robot interface specification page in the FCI
   * Documentation.
   */
  std::array<double, 2> elbow_c{};

  /**
   * Commanded elbow velocity.
   *
   * The values of the array are:
   *  - delbow_c[0] Velocity of the 3rd joint in \f$\frac{rad}{s}\f$
   *  - delbow_c[1] is always 0.
   */
  std::array<double, 2> delbow_c{};

  /**
   * Commanded elbow acceleration.
   *
   * The values of the array are:
   *  - ddelbow_c[0] Acceleration of the 3rd joint in \f$\frac{rad}{s^2}\f$
   *  - ddelbow_c[1] is always 0.
   */
  std::array<double, 2> ddelbow_c{};

  /**
   * \f$\tau_{J}\f$
   * Measured link-side joint torque sensor signals. Unit: \f$[Nm]\f$
   */
  std::array<double, 7> tau_J{};  // NOLINT(readability-identifier-naming)

  /**
   * \f${\tau_J}_d\f$
   * Desired link-side joint torque sensor signals without gravity. Unit: \f$[Nm]\f$
   */
  std::array<double, 7> tau_J_d{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$\dot{\tau_{J}}\f$
   * Derivative of measured link-side joint torque sensor signals. Unit: \f$[\frac{Nm}{s}]\f$
   */
  std::array<double, 7> dtau_J{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$q\f$
   * Measured joint position. Unit: \f$[rad]\f$
   */
  std::array<double, 7> q{};

  /**
   * \f$q_d\f$
   * Desired joint position. Unit: \f$[rad]\f$
   */
  std::array<double, 7> q_d{};

  /**
   * \f$\dot{q}\f$
   * Measured joint velocity. Unit: \f$[\frac{rad}{s}]\f$
   */
  std::array<double, 7> dq{};

  /**
   * \f$\dot{q}_d\f$
   * Desired joint velocity. Unit: \f$[\frac{rad}{s}]\f$
   */
  std::array<double, 7> dq_d{};

  /**
   * \f$\ddot{q}_d\f$
   * Desired joint acceleration. Unit: \f$[\frac{rad}{s^2}]\f$
   */
  std::array<double, 7> ddq_d{};

  /**
   * Indicates which contact level is activated in which joint. After contact disappears, value
   * turns to zero.
   *
   * @see Robot::setCollisionBehavior for setting sensitivity values.
   */
  std::array<double, 7> joint_contact{};

  /**
   * Indicates which contact level is activated in which Cartesian dimension \f$(x,y,z,R,P,Y)\f$.
   * After contact disappears, the value turns to zero.
   *
   * @see Robot::setCollisionBehavior for setting sensitivity values.
   */
  std::array<double, 6> cartesian_contact{};

  /**
   * Indicates which contact level is activated in which joint. After contact disappears, the value
   * stays the same until a reset command is sent.
   *
   * @see Robot::setCollisionBehavior for setting sensitivity values.
   * @see Robot::automaticErrorRecovery for performing a reset after a collision.
   */
  std::array<double, 7> joint_collision{};

  /**
   * Indicates which contact level is activated in which Cartesian dimension \f$(x,y,z,R,P,Y)\f$.
   * After contact disappears, the value stays the same until a reset command is sent.
   *
   * @see Robot::setCollisionBehavior for setting sensitivity values.
   * @see Robot::automaticErrorRecovery for performing a reset after a collision.
   */
  std::array<double, 6> cartesian_collision{};

  /**
   * \f$\hat{\tau}_{\text{ext}}\f$
   * Low-pass filtered torques generated by external forces on the joints. It does not include
   * configured end-effector and load nor the mass and dynamics of the robot. tau_ext_hat_filtered
   * is the error between tau_J and the expected torques given by the robot model. Unit: \f$[Nm]\f$.
   */
  std::array<double, 7> tau_ext_hat_filtered{};

  /**
   * \f$^OF_{K,\text{ext}}\f$
   * Estimated external wrench (force, torque) acting on stiffness frame, expressed
   * relative to the @ref o-frame "base frame". Forces applied by the robot to the environment are
   * positive, while forces applied by the environment on the robot are negative. Becomes
   * \f$[0,0,0,0,0,0]\f$ when near or in a singularity. See also @ref k-frame "Stiffness frame K".
   * Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
   */
  std::array<double, 6> O_F_ext_hat_K{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$^{K}F_{K,\text{ext}}\f$
   * Estimated external wrench (force, torque) acting on stiffness frame,
   * expressed relative to the stiffness frame. Forces applied by the robot to the environment are
   * positive, while forces applied by the environment on the robot are negative. Becomes
   * \f$[0,0,0,0,0,0]\f$ when near or in a singularity. See also @ref k-frame "Stiffness frame K".
   * Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
   */
  std::array<double, 6> K_F_ext_hat_K{};  // NOLINT(readability-identifier-naming)

  /**
   * \f${^OdP_{EE}}_{d}\f$
   * Desired end effector twist in @ref o-frame "base frame".
   * Unit: \f$[\frac{m}{s},\frac{m}{s},\frac{m}{s},\frac{rad}{s},\frac{rad}{s},\frac{rad}{s}]\f$.
   */
  std::array<double, 6> O_dP_EE_d{};  // NOLINT(readability-identifier-naming)

  /**
   * \f${^OddP}_O\f$
   * Linear component of the acceleration of the robot's base, expressed in frame parallel to the
   * @ref o-frame "base frame", i.e. the base's translational acceleration. If the base is resting
   * this shows the direction of the gravity vector.
   * It is harcoded for now to `{0, 0, -9.81}`.
   */
  std::array<double, 3> O_ddP_O{};  // NOLINT(readability-identifier-naming)

  /**
   * \f${^OT_{EE}}_{c}\f$
   * Last commanded end effector pose of motion generation in @ref o-frame "base frame".
   * Pose is represented as a 4x4 matrix in column-major format.
   */
  std::array<double, 16> O_T_EE_c{};  // NOLINT(readability-identifier-naming)

  /**
   * \f${^OdP_{EE}}_{c}\f$
   * Last commanded end effector twist in @ref o-frame "base frame".
   * Unit: \f$[\frac{m}{s},\frac{m}{s},\frac{m}{s},\frac{rad}{s},\frac{rad}{s},\frac{rad}{s}]\f$.
   */
  std::array<double, 6> O_dP_EE_c{};  // NOLINT(readability-identifier-naming)

  /**
   * \f${^OddP_{EE}}_{c}\f$
   * Last commanded end effector acceleration in @ref o-frame "base frame".
   * Unit:
   * \f$[\frac{m}{s^2},\frac{m}{s^2},\frac{m}{s^2},\frac{rad}{s^2},\frac{rad}{s^2},\frac{rad}{s^2}]\f$.
   */
  std::array<double, 6> O_ddP_EE_c{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$\theta\f$
   * Motor position. Unit: \f$[rad]\f$
   */
  std::array<double, 7> theta{};

  /**
   * \f$\dot{\theta}\f$
   * Motor velocity. Unit: \f$[\frac{rad}{s}]\f$
   */
  std::array<double, 7> dtheta{};

  /**
   * \f$\ddot{x}\f$
   * Translational acceleration data from the accelerometers located on the top of the PCB
   * of Joint 1 through Joint 6 (no accelerometers in the last joint). Unit: \f$[\frac{m}{s^2}]\f$
   */
  std::array<std::array<float, 3>, 6> accelerometer_top{};

  /**
   * \f$\ddot{x}\f$
   * Translational acceleration data from the accelerometers located on the bottom of the PCB
   * of Joint 1 through Joint 6 (no accelerometers in the last joint). Unit: \f$[\frac{m}{s^2}]\f$
   */
  std::array<std::array<float, 3>, 6> accelerometer_bottom{};

  /**
   * Current error state.
   */
  Errors current_errors{};

  /**
   * Contains the errors that aborted the previous motion.
   */
  Errors last_motion_errors{};

  /**
   * Percentage of the last 100 control commands that were successfully received by the robot.
   *
   * Shows a value of zero if no control or motion generator loop is currently running.
   *
   * Range: \f$[0, 1]\f$.
   */
  double control_command_success_rate{};

  /**
   * Current robot mode.
   */
  RobotMode robot_mode = RobotMode::kUserStopped;

  /**
   * Strictly monotonically increasing timestamp since robot start.
   *
   * Inside of control loops @ref callback-docs "time_step" parameter of Robot::control can be used
   * instead.
   */
  Duration time{};
};

/**
 * Streams the robot state as JSON object: {"field_name_1": [0,0,0,0,0,0,0], "field_name_2":
 * [0,0,0,0,0,0], ...}
 *
 * @param[in] ostream Ostream instance
 * @param[in] robot_state RobotState instance to stream
 *
 * @return Ostream instance
 */
std::ostream& operator<<(std::ostream& ostream, const franka::RobotState& robot_state);

/**
 * Streams RobotMode in human-readable form

 * @param[in] ostream Ostream instance
 * @param[in] robot_mode RobotMode to stream
 *
 * @return Ostream instance
 */
std::ostream& operator<<(std::ostream& ostream, RobotMode robot_mode);

}  // namespace franka
