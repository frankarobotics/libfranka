// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/lowpass_filter.h>
#include <franka/robot_model_base.h>
#include <franka/robot_state.h>
#include <research_interface/robot/service_types.h>
#include <franka/commands/get_robot_model_command.hpp>

/**
 * @file robot.h
 * Contains the franka::Robot type.
 */
namespace franka {

class Model;

class ActiveControlBase;

/**
 * Maintains a network connection to the robot, provides the current robot state, gives access to
 * the model library and allows to control the robot.
 *
 * @note
 * The members of this class are threadsafe.
 *
 * @anchor o-frame
 * @par Base frame O
 * The base frame is located at the center of the robot's base. Its z-axis is identical with the
 * axis of rotation of the first joint.
 *
 * @anchor f-frame
 * @par Flange frame F
 * The flange frame is located at the center of the flange surface. Its z-axis is identical with the
 * axis of rotation of the last joint. This frame is fixed and cannot be changed.
 *
 * @anchor ne-frame
 * @par Nominal end effector frame NE
 * The nominal end effector frame is configured outside of libfranka (in DESK) and cannot be changed
 * here. It may be used to set end effector frames which are rarely changed.
 *
 * @anchor ee-frame
 * @par end effector frame EE
 * By default, the end effector frame EE is the same as the nominal end effector frame NE
 * (i.e. the transformation between NE and EE is the identity transformation). It may be used to set
 * end effector frames which are changed more frequently (such as a tool that is grasped with the
 * end effector). With Robot::setEE, a custom transformation matrix can be set.
 *
 * @anchor k-frame
 * @par Stiffness frame K
 * This frame describes the transformation from the end effector frame EE to the stiffness frame
 * K. The stiffness frame is used for Cartesian impedance control, and for measuring and applying
 * forces. The values set using Robot::setCartesianImpedance are used in the direction
 * of the stiffness frame. It can be set with Robot::setK. This frame allows to modify the
 * compliance behavior of the robot (e.g. to have a low stiffness around a specific point which
 * is not the end effector). The stiffness frame does not affect where the robot will move to.
 * The user should always command the desired end effector frame (and not the desired stiffness
 * frame).
 */
class Robot {
 public:
  /**
   * Number of joints of the robot.
   */
  static constexpr size_t kNumJoints = 7;

  /**
   * Version of the robot server.
   */
  using ServerVersion = uint16_t;

  /**
   * Establishes a connection with the robot.
   *
   * @param[in] franka_address IP/hostname of the robot.
   * @param[in] realtime_config if set to Enforce, an exception will be thrown if realtime priority
   * cannot be set when required. Setting realtime_config to Ignore disables this behavior.
   * @param[in] log_size sets how many last states should be kept for logging purposes.
   * The log is provided when a ControlException is thrown.
   *
   * @throw NetworkException if the connection is unsuccessful.
   * @throw IncompatibleVersionException if this version of `libfranka` is not supported.
   */
  explicit Robot(const std::string& franka_address,
                 RealtimeConfig realtime_config = RealtimeConfig::kEnforce,
                 size_t log_size = 50);

  /**
   * Move-constructs a new Robot instance.
   *
   * @param[in] other Other Robot instance.
   */
  Robot(Robot&& other) noexcept;

  /**
   * Move-assigns this Robot from another Robot instance.
   *
   * @param[in] other Other Robot instance.
   *
   * @return Robot instance.
   */
  Robot& operator=(Robot&& other) noexcept;

  /**
   * Closes the connection.
   */
  virtual ~Robot() noexcept;

  /**
   * @name Motion generation and joint-level torque commands
   *
   * The following methods allow to perform motion generation and/or send joint-level torque
   * commands without gravity and friction by providing callback functions.
   *
   * Only one of these methods can be active at the same time; a franka::ControlException is thrown
   * otherwise.
   *
   * @anchor callback-docs
   * When a robot state is received, the callback function is used to calculate the response: the
   * desired values for that time step. After sending back the response, the callback function will
   * be called again with the most recently received robot state. Since the robot is controlled with
   * a 1 kHz frequency, the callback functions have to compute their result in a short time frame
   * in order to be accepted. Callback functions take two parameters:
   *
   * * A franka::RobotState showing the current robot state.
   * * A franka::Duration to indicate the time since the last callback invocation. Thus, the
   *   duration is zero on the first invocation of the callback function!
   *
   * The following incomplete example shows the general structure of a callback function:
   *
   * @code{.cpp}
   * double time = 0.0;
   * auto control_callback = [&time](const franka::RobotState& robot_state,
   *                                 franka::Duration time_step) -> franka::JointPositions {
   *   time += time_step.toSec();  // Update time at the beginning of the callback.
   *
   *   franka::JointPositions output = getJointPositions(time);
   *
   *   if (time >= max_time) {
   *     // Return MotionFinished at the end of the trajectory.
   *     return franka::MotionFinished(output);
   *   }
   *
   *   return output;
   * }
   * @endcode
   *
   * @{
   */

  /**
   * Starts a control loop for sending joint-level torque commands.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. False by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(std::function<Torques(const RobotState&, franka::Duration)> control_callback,
               bool limit_rate = false,
               double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for sending joint-level torque commands and joint positions.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. False by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque or joint position commands are NaN or
   * infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<Torques(const RobotState&, franka::Duration)> control_callback,
      std::function<JointPositions(const RobotState&, franka::Duration)> motion_generator_callback,
      bool limit_rate = false,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for sending joint-level torque commands and joint velocities.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. False by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque or joint velocity commands are NaN or
   * infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<Torques(const RobotState&, franka::Duration)> control_callback,
      std::function<JointVelocities(const RobotState&, franka::Duration)> motion_generator_callback,
      bool limit_rate = false,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for sending joint-level torque commands and Cartesian poses.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. False by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque or Cartesian pose command elements are NaN
   * or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<Torques(const RobotState&, franka::Duration)> control_callback,
      std::function<CartesianPose(const RobotState&, franka::Duration)> motion_generator_callback,
      bool limit_rate = false,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for sending joint-level torque commands and Cartesian velocities.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] control_callback Callback function providing joint-level torque commands.
   * See @ref callback-docs "here" for more details.
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] limit_rate True if rate limiting should be activated. False by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint-level torque or Cartesian velocity command elements are
   * NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(std::function<Torques(const RobotState&, franka::Duration)> control_callback,
               std::function<CartesianVelocities(const RobotState&, franka::Duration)>
                   motion_generator_callback,
               bool limit_rate = false,
               double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for a joint position motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   * @param[in] limit_rate True if rate limiting should be activated. False by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint position commands are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<JointPositions(const RobotState&, franka::Duration)> motion_generator_callback,
      ControllerMode controller_mode = ControllerMode::kJointImpedance,
      bool limit_rate = false,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for a joint velocity motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   * @param[in] limit_rate True if rate limiting should be activated. False by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if joint velocity commands are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<JointVelocities(const RobotState&, franka::Duration)> motion_generator_callback,
      ControllerMode controller_mode = ControllerMode::kJointImpedance,
      bool limit_rate = false,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for a Cartesian pose motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   * @param[in] limit_rate True if rate limiting should be activated. False by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if Cartesian pose command elements are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(
      std::function<CartesianPose(const RobotState&, franka::Duration)> motion_generator_callback,
      ControllerMode controller_mode = ControllerMode::kJointImpedance,
      bool limit_rate = false,
      double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * Starts a control loop for a Cartesian velocity motion generator with a given controller mode.
   *
   * Sets realtime priority for the current thread.
   * Cannot be executed while another control or motion generator loop is active.
   *
   * @param[in] motion_generator_callback Callback function for motion generation. See @ref
   * callback-docs "here" for more details.
   * @param[in] controller_mode Controller to use to execute the motion.
   * @param[in] limit_rate True if rate limiting should be activated. False by default.
   * This could distort your motion!
   * @param[in] cutoff_frequency Cutoff frequency for a first order low-pass filter applied on
   * the user commanded signal. Set to franka::kMaxCutoffFrequency to disable.
   *
   * @throw ControlException if an error related to motion generation occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw RealtimeException if realtime priority cannot be set for the current thread.
   * @throw std::invalid_argument if Cartesian velocity command elements are NaN or infinity.
   *
   * @see Robot::Robot to change behavior if realtime priority cannot be set.
   */
  void control(std::function<CartesianVelocities(const RobotState&, franka::Duration)>
                   motion_generator_callback,
               ControllerMode controller_mode = ControllerMode::kJointImpedance,
               bool limit_rate = false,
               double cutoff_frequency = kDefaultCutoffFrequency);

  /**
   * @}
   */

  /**
   * Starts a loop for reading the current robot state.
   *
   * Cannot be executed while a control or motion generator loop is running.
   *
   * This minimal example will print the robot state 100 times:
   * @code{.cpp}
   * franka::Robot robot("robot.franka.de");
   * size_t count = 0;
   * robot.read([&count](const franka::RobotState& robot_state) {
   *   std::cout << robot_state << std::endl;
   *   return count++ < 100;
   * });
   * @endcode
   *
   * @param[in] read_callback Callback function for robot state reading.
   *
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void read(std::function<bool(const RobotState&)> read_callback);

  /**
   * Waits for a robot state update and returns it.
   *
   * Cannot be executed while a control or motion generator loop is running.
   *
   * @return Current robot state.
   *
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see Robot::read for a way to repeatedly receive the robot state.
   */
  virtual RobotState readOnce();

  /**
   * @name Commands
   *
   * Commands are executed by communicating with the robot over the network.
   * These functions should therefore not be called from within control or motion generator loops.
   * @{
   */

  /**
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @return std::string Provides the URDF model of the attached robot arm as json string
   */
  auto getRobotModel() -> std::string;

  /**
   * Computes upper limits for joint velocities based on current joint positions.
   *
   * @param[in] joint_positions Joint positions.
   * @return Upper limits for joint velocities.
   */
  auto getUpperJointVelocityLimits(const std::array<double, kNumJoints>& joint_positions)
      -> std::array<double, kNumJoints>;

  /**
   * Computes lower limits for joint velocities based on current joint positions.
   *
   * @param[in] joint_positions Joint positions.
   * @return Lower limits for joint velocities.
   */
  auto getLowerJointVelocityLimits(const std::array<double, kNumJoints>& joint_positions)
      -> std::array<double, kNumJoints>;

  /**
   * Changes the collision behavior.
   *
   * Set separate torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] lower_torque_thresholds_acceleration Contact torque thresholds during
   * acceleration/deceleration for each joint in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_acceleration Collision torque thresholds during
   * acceleration/deceleration for each joint in \f$[Nm]\f$.
   * @param[in] lower_torque_thresholds_nominal Contact torque thresholds for each joint
   * in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_nominal Collision torque thresholds for each joint
   * in \f$[Nm]\f$.
   * @param[in] lower_force_thresholds_acceleration Contact force thresholds during
   * acceleration/deceleration for \f$(x,y,z,R,P,Y)\f$ in \f$[N]\f$.
   * @param[in] upper_force_thresholds_acceleration Collision force thresholds during
   * acceleration/deceleration for \f$(x,y,z,R,P,Y)\f$ in \f$[N]\f$.
   * @param[in] lower_force_thresholds_nominal Contact force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   * @param[in] upper_force_thresholds_nominal Collision force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see RobotState::cartesian_contact
   * @see RobotState::cartesian_collision
   * @see RobotState::joint_contact
   * @see RobotState::joint_collision
   * @see Robot::automaticErrorRecovery for performing a reset after a collision.
   */
  void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                            const std::array<double, 7>& upper_torque_thresholds_acceleration,
                            const std::array<double, 7>& lower_torque_thresholds_nominal,
                            const std::array<double, 7>& upper_torque_thresholds_nominal,
                            const std::array<double, 6>& lower_force_thresholds_acceleration,
                            const std::array<double, 6>& upper_force_thresholds_acceleration,
                            const std::array<double, 6>& lower_force_thresholds_nominal,
                            const std::array<double, 6>& upper_force_thresholds_nominal);

  /**
   * Changes the collision behavior.
   *
   * Set common torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] lower_torque_thresholds Contact torque thresholds for each joint in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds Collision torque thresholds for each joint in \f$[Nm]\f$.
   * @param[in] lower_force_thresholds Contact force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   * @param[in] upper_force_thresholds Collision force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see RobotState::cartesian_contact
   * @see RobotState::cartesian_collision
   * @see RobotState::joint_contact
   * @see RobotState::joint_collision
   * @see Robot::automaticErrorRecovery for performing a reset after a collision.
   */
  void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                            const std::array<double, 7>& upper_torque_thresholds,
                            const std::array<double, 6>& lower_force_thresholds,
                            const std::array<double, 6>& upper_force_thresholds);

  /**
   * Sets the impedance for each joint in the internal controller.
   *
   * User-provided torques are not affected by this setting.
   *
   * @param[in] K_theta Joint impedance values \f$K_{\theta_{1-7}} = \in [0,14250]
   * \frac{Nm}{rad}\f$
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void setJointImpedance(
      const std::array<double, 7>& K_theta);  // NOLINT(readability-identifier-naming)

  /**
   * Sets the Cartesian stiffness/compliance (for x, y, z, roll, pitch, yaw) in the internal
   * controller.
   *
   * The values set using Robot::setCartesianImpedance are used in the direction of the
   * stiffness frame, which can be set with Robot::setK.
   *
   * Inputs received by the torque controller are not affected by this setting.
   *
   * @param[in] K_x Cartesian impedance values \f$K_x=(K_{x_{x,y,z}} \in [10,3000] \frac{N}{m},
   * K_{x_{R,P,Y}} \in [1,300] \frac{Nm}{rad})\f$
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void setCartesianImpedance(
      const std::array<double, 6>& K_x);  // NOLINT(readability-identifier-naming)

  /**
   * Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).
   *
   * If a flag is set to true, movement is unlocked.
   *
   * @note
   * Guiding mode can be enabled by pressing the two opposing buttons near the robot's flange.
   *
   * @param[in] guiding_mode Unlocked movement in (x, y, z, R, P, Y) in guiding mode.
   * @param[in] elbow True if the elbow is free in guiding mode, false otherwise.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void setGuidingMode(const std::array<bool, 6>& guiding_mode, bool elbow);

  /**
   * Sets the transformation \f$^{EE}T_K\f$ from end effector frame to stiffness frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] EE_T_K Vectorized EE-to-K transformation matrix \f$^{EE}T_K\f$, column-major.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see Robot for an explanation of the stiffness frame.
   */
  void setK(const std::array<double, 16>& EE_T_K);  // NOLINT(readability-identifier-naming)

  /**
   * Sets the transformation \f$^{NE}T_{EE}\f$ from nominal end effector to end effector frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] NE_T_EE Vectorized NE-to-EE transformation matrix \f$^{NE}T_{EE}\f$, column-major.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   * @see RobotState::NE_T_EE for end effector pose in @ref ne-frame "nominal end effector frame
   * NE".
   * @see RobotState::O_T_EE for end effector pose in @ref o-frame "world base frame O".
   * @see RobotState::F_T_EE for end effector pose in @ref f-frame "flange frame F".
   */
  void setEE(const std::array<double, 16>& NE_T_EE);  // NOLINT(readability-identifier-naming)

  /**
   * Sets dynamic parameters of a payload.
   *
   * @note
   * This is not for setting end effector parameters, which have to be set in the administrator's
   * interface.
   *
   * @param[in] load_mass Mass of the load in \f$[kg]\f$.
   * @param[in] F_x_Cload Translation from flange to center of mass of load
   * \f$^Fx_{C_\text{load}}\f$ in \f$[m]\f$.
   * @param[in] load_inertia Inertia matrix \f$I_\text{load}\f$ in \f$[kg \times m^2]\f$,
   * column-major.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
               const std::array<double, 9>& load_inertia);

  /**
   * Runs automatic error recovery on the robot.
   *
   * Automatic error recovery e.g. resets the robot after a collision occurred.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void automaticErrorRecovery();

  /**
   * Starts a new torque controller
   *
   * @return unique_ptr of ActiveTorqueControl for the started motion
   *
   * @throw ControlException if an error related to torque control or motion generation
   * occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual std::unique_ptr<ActiveControlBase> startTorqueControl();

  /**
   * Starts a new joint position motion generator
   *
   * @param control_type research_interface::robot::Move::ControllerMode control type for the
   *   operation
   * @return unique_ptr of ActiveMotionGenerator for the started motion
   *
   * @throw ControlException if an error related to torque control or motion generation
   * occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual std::unique_ptr<ActiveControlBase> startJointPositionControl(
      const research_interface::robot::Move::ControllerMode& control_type);

  /**
   * Starts a new async joint position motion generator
   *
   * @param control_type research_interface::robot::Move::ControllerMode control type for the
   *   operation
   * @param maximum_velocities maximum joint velocities for the motion generator
   * @return unique_ptr of ActiveMotionGenerator for the started motion
   *
   * @throw ControlException if an error related to torque control or motion generation
   *   occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual std::unique_ptr<ActiveControlBase> startAsyncJointPositionControl(
      const research_interface::robot::Move::ControllerMode& control_type,
      const std::optional<std::vector<double>>& maximum_velocities);

  /**
   * Starts a new joint velocity motion generator
   *
   * @param control_type research_interface::robot::Move::ControllerMode control type for the
   * operation
   * @return unique_ptr of ActiveMotionGenerator for the started motion
   * @throw ControlException if an error related to torque control or motion generation
   * occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual std::unique_ptr<ActiveControlBase> startJointVelocityControl(
      const research_interface::robot::Move::ControllerMode& control_type);

  /**
   * Starts a new cartesian position motion generator
   *
   * @param control_type research_interface::robot::Move::ControllerMode control type for the
   * operation
   * @return unique_ptr of ActiveMotionGenerator for the started motion
   * @throw ControlException if an error related to torque control or motion generation
   * occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual std::unique_ptr<ActiveControlBase> startCartesianPoseControl(
      const research_interface::robot::Move::ControllerMode& control_type);

  /**
   * Starts a new cartesian velocity motion generator
   *
   * @param control_type research_interface::robot::Move::ControllerMode control type for the
   * operation
   * @return unique_ptr of ActiveMotionGenerator for the started motion
   *
   * @throw ControlException if an error related to torque control or motion generation
   * occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual std::unique_ptr<ActiveControlBase> startCartesianVelocityControl(
      const research_interface::robot::Move::ControllerMode& control_type);

  /**
   * Stops all currently running motions.
   *
   * If a control or motion generator loop is running in another thread, it will be preempted
   * with a franka::ControlException.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void stop();

  /**
   * @}
   */

  /**
   * Loads the model library from the robot.
   *
   * @return Model instance.
   *
   * @throw ModelException if the model library cannot be loaded.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  Model loadModel();

  // Loads the model library for the unittests mockRobotModel
  Model loadModel(std::unique_ptr<RobotModelBase> robot_model);

  /**
   * Returns the software version reported by the connected server.
   *
   * @return Software version of the connected server.
   */
  ServerVersion serverVersion() const noexcept;

  /// @cond DO_NOT_DOCUMENT
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  /// @endcond

  class Impl;

 protected:
  /**
   * Constructs a new Robot given a Robot::Impl. This enables unittests with Robot::Impl-Mocks.
   *
   * @param robot_impl Robot::Impl to use
   */
  Robot(std::shared_ptr<Impl> robot_impl);

  /**
   * Default constructor to enable mocking and testing.
   */
  Robot() = default;

 private:
  /**
   * Starts a new motion generator and controller
   *
   * @tparam MotionGeneratorType the franka control type
   * @param control_mode defines the type of motion / control that shall be started
   * @param controller_mode the controller-mode that shall be used
   * @return unique_ptr of ActiveMotionGenerator for the started motion
   *
   * @throw ControlException if an error related to torque control or motion generation
   *   occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.*
   * @throw NetworkException if the connection is lost,e.g.after a timeout.
   * @throw std::invalid_argument if joint - level torque commands are NaN or infinity.
   */
  template <typename MotionGeneratorType>
  std::unique_ptr<ActiveControlBase> startControl(
      const research_interface::robot::Move::ControllerMode& controller_type);

  std::shared_ptr<Impl> impl_;
  std::mutex control_mutex_;

  /**
   * Starts a new async motion generator and controller
   *
   * @tparam MotionGeneratorType the franka control type
   * @param controller_type defines the type of motion / control that shall be started
   * @param maximum_velocities maximum joint velocities for the motion generator
   * @return unique_ptr of ActiveMotionGenerator for the started motion
   *
   * @throw ControlException if an error related to torque control or motion generation
   *   occurred.
   * @throw InvalidOperationException if a conflicting operation is already running.*
   * @throw NetworkException if the connection is lost,e.g.after a timeout.
   * @throw std::invalid_argument if joint - level torque commands are NaN or infinity.
   */
  template <typename MotionGeneratorType>
  auto startAsyncControl(const research_interface::robot::Move::ControllerMode& controller_type,
                         const std::optional<std::vector<double>>& maximum_velocities)
      -> std::unique_ptr<ActiveControlBase>;
};

}  // namespace franka
