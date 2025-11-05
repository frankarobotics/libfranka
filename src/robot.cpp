// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/active_torque_control.h>
#include <franka/robot.h>

#include <utility>

#include "control_loop.h"
#include "motion_generator_traits.h"
#include "network.h"
#include "robot_impl.h"

namespace franka {

void assertOwningLock(const std::unique_lock<std::mutex>& lock) {
  if (!lock.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }
}

Robot::Robot(const std::string& franka_address, RealtimeConfig realtime_config, size_t log_size)
    : impl_{new Robot::Impl(
          std::make_unique<Network>(franka_address, research_interface::robot::kCommandPort),
          log_size,
          realtime_config)} {}

// Has to be declared here, as the Impl type is incomplete in the header.
Robot::~Robot() noexcept = default;

Robot::Robot(Robot&& other) noexcept : impl_(std::move(other.impl_)) {
  std::lock_guard<std::mutex> mutex(other.control_mutex_);
}

Robot& Robot::operator=(Robot&& other) noexcept {
  if (&other != this) {
    std::unique_lock<std::mutex> this_lock(control_mutex_, std::defer_lock);
    std::unique_lock<std::mutex> other_lock(other.control_mutex_, std::defer_lock);
    std::lock(this_lock, other_lock);
    impl_ = std::move(other.impl_);
  }
  return *this;
}

Robot::ServerVersion Robot::serverVersion() const noexcept {
  return impl_->serverVersion();
}

void Robot::control(std::function<Torques(const RobotState&, franka::Duration)> control_callback,
                    bool limit_rate,
                    double cutoff_frequency) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  ControlLoop<Torques> control_loop(*impl_, std::move(control_callback), limit_rate,
                                    cutoff_frequency);
  control_loop.loop();
}

void Robot::control(
    std::function<Torques(const RobotState&, franka::Duration)> control_callback,
    std::function<JointPositions(const RobotState&, franka::Duration)> motion_generator_callback,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  ControlLoop<JointPositions> control_loop(*impl_, std::move(control_callback),
                                           std::move(motion_generator_callback), limit_rate,
                                           cutoff_frequency);
  control_loop.loop();
}

void Robot::control(
    std::function<Torques(const RobotState&, franka::Duration)> control_callback,
    std::function<JointVelocities(const RobotState&, franka::Duration)> motion_generator_callback,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  ControlLoop<JointVelocities> control_loop(*impl_, std::move(control_callback),
                                            std::move(motion_generator_callback), limit_rate,
                                            cutoff_frequency);
  control_loop.loop();
}

void Robot::control(
    std::function<Torques(const RobotState&, franka::Duration)> control_callback,
    std::function<CartesianPose(const RobotState&, franka::Duration)> motion_generator_callback,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  ControlLoop<CartesianPose> control_loop(*impl_, std::move(control_callback),
                                          std::move(motion_generator_callback), limit_rate,
                                          cutoff_frequency);
  control_loop.loop();
}

void Robot::control(std::function<Torques(const RobotState&, franka::Duration)> control_callback,
                    std::function<CartesianVelocities(const RobotState&, franka::Duration)>
                        motion_generator_callback,
                    bool limit_rate,
                    double cutoff_frequency) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  ControlLoop<CartesianVelocities> control_loop(*impl_, std::move(control_callback),
                                                std::move(motion_generator_callback), limit_rate,
                                                cutoff_frequency);
  control_loop.loop();
}

void Robot::control(
    std::function<JointPositions(const RobotState&, franka::Duration)> motion_generator_callback,
    ControllerMode controller_mode,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  ControlLoop<JointPositions> control_loop(
      *impl_, controller_mode, std::move(motion_generator_callback), limit_rate, cutoff_frequency);
  control_loop.loop();
}

void Robot::control(
    std::function<JointVelocities(const RobotState&, franka::Duration)> motion_generator_callback,
    ControllerMode controller_mode,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  ControlLoop<JointVelocities> control_loop(
      *impl_, controller_mode, std::move(motion_generator_callback), limit_rate, cutoff_frequency);
  control_loop.loop();
}

void Robot::control(
    std::function<CartesianPose(const RobotState&, franka::Duration)> motion_generator_callback,
    ControllerMode controller_mode,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  ControlLoop<CartesianPose> control_loop(
      *impl_, controller_mode, std::move(motion_generator_callback), limit_rate, cutoff_frequency);
  control_loop.loop();
}

void Robot::control(std::function<CartesianVelocities(const RobotState&, franka::Duration)>
                        motion_generator_callback,
                    ControllerMode controller_mode,
                    bool limit_rate,
                    double cutoff_frequency) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  ControlLoop<CartesianVelocities> control_loop(
      *impl_, controller_mode, std::move(motion_generator_callback), limit_rate, cutoff_frequency);
  control_loop.loop();
}

// NOLINTNEXTLINE(performance-unnecessary-value-param)
void Robot::read(std::function<bool(const RobotState&)> read_callback) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  while (true) {
    RobotState robot_state = impl_->updateMotion(std::nullopt, std::nullopt);
    if (!read_callback(robot_state)) {
      break;
    }
  }
}

RobotState Robot::readOnce() {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  return impl_->readOnce();
}

auto Robot::getRobotModel() -> std::string {
  auto get_robot_model =
      impl_->executeCommand<research_interface::robot::GetRobotModel, GetRobotModelResult>();
  return get_robot_model.robot_model_urdf;
}

auto Robot::getUpperJointVelocityLimits(const std::array<double, kNumJoints>& joint_positions)
    -> std::array<double, kNumJoints> {
  return impl_->getUpperJointVelocityLimits(joint_positions);
}

auto Robot::getLowerJointVelocityLimits(const std::array<double, kNumJoints>& joint_positions)
    -> std::array<double, kNumJoints> {
  return impl_->getLowerJointVelocityLimits(joint_positions);
}

void Robot::setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                                 const std::array<double, 7>& upper_torque_thresholds_acceleration,
                                 const std::array<double, 7>& lower_torque_thresholds_nominal,
                                 const std::array<double, 7>& upper_torque_thresholds_nominal,
                                 const std::array<double, 6>& lower_force_thresholds_acceleration,
                                 const std::array<double, 6>& upper_force_thresholds_acceleration,
                                 const std::array<double, 6>& lower_force_thresholds_nominal,
                                 const std::array<double, 6>& upper_force_thresholds_nominal) {
  impl_->executeCommand<research_interface::robot::SetCollisionBehavior>(
      lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
      lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
      lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

void Robot::setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                                 const std::array<double, 7>& upper_torque_thresholds,
                                 const std::array<double, 6>& lower_force_thresholds,
                                 const std::array<double, 6>& upper_force_thresholds) {
  impl_->executeCommand<research_interface::robot::SetCollisionBehavior>(
      lower_torque_thresholds, upper_torque_thresholds, lower_torque_thresholds,
      upper_torque_thresholds, lower_force_thresholds, upper_force_thresholds,
      lower_force_thresholds, upper_force_thresholds);
}

void Robot::setJointImpedance(
    const std::array<double, 7>& K_theta) {  // NOLINT(readability-identifier-naming)
  impl_->executeCommand<research_interface::robot::SetJointImpedance>(K_theta);
}

void Robot::setCartesianImpedance(
    const std::array<double, 6>& K_x) {  // NOLINT(readability-identifier-naming)
  impl_->executeCommand<research_interface::robot::SetCartesianImpedance>(K_x);
}

void Robot::setGuidingMode(const std::array<bool, 6>& guiding_mode, bool elbow) {
  impl_->executeCommand<research_interface::robot::SetGuidingMode>(guiding_mode, elbow);
}

void Robot::setK(const std::array<double, 16>& EE_T_K) {  // NOLINT(readability-identifier-naming)
  impl_->executeCommand<research_interface::robot::SetEEToK>(EE_T_K);
}

void Robot::setEE(const std::array<double, 16>& NE_T_EE) {  // NOLINT(readability-identifier-naming)
  impl_->executeCommand<research_interface::robot::SetNEToEE>(NE_T_EE);
}

void Robot::setLoad(
    double load_mass,
    const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
    const std::array<double, 9>& load_inertia) {
  impl_->executeCommand<research_interface::robot::SetLoad>(load_mass, F_x_Cload, load_inertia);
}

void Robot::automaticErrorRecovery() {
  impl_->executeCommand<research_interface::robot::AutomaticErrorRecovery>();
}

template <typename MotionGeneratorType>
std::unique_ptr<ActiveControlBase> Robot::startControl(
    const research_interface::robot::Move::ControllerMode& controller_type) {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  research_interface::robot::Move::MotionGeneratorMode motion_generator_mode =
      MotionGeneratorTraits<MotionGeneratorType>::kMotionGeneratorMode;

  uint32_t motion_id = impl_->startMotion(
      controller_type, motion_generator_mode, ControlLoop<MotionGeneratorType>::kDefaultDeviation,
      ControlLoop<MotionGeneratorType>::kDefaultDeviation, false, std::nullopt);
  return std::unique_ptr<ActiveControlBase>(new ActiveMotionGenerator<MotionGeneratorType>(
      impl_, motion_id, std::move(control_lock), controller_type));
}

template <typename MotionGeneratorType>
auto Robot::startAsyncControl(
    const research_interface::robot::Move::ControllerMode& controller_type,
    const std::optional<std::vector<double>>& maximum_velocities)
    -> std::unique_ptr<ActiveControlBase> {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  research_interface::robot::Move::MotionGeneratorMode motion_generator_mode =
      MotionGeneratorTraits<MotionGeneratorType>::kMotionGeneratorMode;

  uint32_t motion_id = impl_->startMotion(
      controller_type, motion_generator_mode, ControlLoop<MotionGeneratorType>::kDefaultDeviation,
      ControlLoop<MotionGeneratorType>::kDefaultDeviation, true, maximum_velocities);
  return std::unique_ptr<ActiveControlBase>(new ActiveMotionGenerator<MotionGeneratorType>(
      impl_, motion_id, std::move(control_lock), controller_type));
}

std::unique_ptr<ActiveControlBase> Robot::startTorqueControl() {
  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  assertOwningLock(control_lock);

  // Torque control has no variable rate at the moment
  uint32_t motion_id =
      impl_->startMotion(research_interface::robot::Move::ControllerMode::kExternalController,
                         research_interface::robot::Move::MotionGeneratorMode::kNone,
                         ControlLoop<JointVelocities>::kDefaultDeviation,
                         ControlLoop<JointVelocities>::kDefaultDeviation, false, std::nullopt);

  return std::unique_ptr<ActiveControlBase>(
      new ActiveTorqueControl(impl_, motion_id, std::move(control_lock)));
}

std::unique_ptr<ActiveControlBase> Robot::startJointPositionControl(
    const research_interface::robot::Move::ControllerMode& control_type) {
  return startControl<JointPositions>(control_type);
}

std::unique_ptr<ActiveControlBase> Robot::startAsyncJointPositionControl(
    const research_interface::robot::Move::ControllerMode& control_type,
    const std::optional<std::vector<double>>& maximum_velocities) {
  return startAsyncControl<JointPositions>(control_type, maximum_velocities);
}

std::unique_ptr<ActiveControlBase> Robot::startJointVelocityControl(
    const research_interface::robot::Move::ControllerMode& control_type) {
  return startControl<JointVelocities>(control_type);
}

std::unique_ptr<ActiveControlBase> Robot::startCartesianPoseControl(
    const research_interface::robot::Move::ControllerMode& control_type) {
  return startControl<CartesianPose>(control_type);
}

std::unique_ptr<ActiveControlBase> Robot::startCartesianVelocityControl(
    const research_interface::robot::Move::ControllerMode& control_type) {
  return startControl<CartesianVelocities>(control_type);
}

void Robot::stop() {
  impl_->executeCommand<research_interface::robot::StopMove>();
}

Model Robot::loadModel() {
  return impl_->loadModel(getRobotModel());
}

Model Robot::loadModel(std::unique_ptr<RobotModelBase> robot_model) {
  return impl_->loadModel(std::move(robot_model));
}

Robot::Robot(std::shared_ptr<Impl> robot_impl) : impl_(std::move(robot_impl)){};

template std::unique_ptr<ActiveControlBase> Robot::startControl<JointVelocities>(
    const research_interface::robot::Move::ControllerMode& controller_type);
template std::unique_ptr<ActiveControlBase> Robot::startControl<JointPositions>(
    const research_interface::robot::Move::ControllerMode& controller_type);
template std::unique_ptr<ActiveControlBase> Robot::startControl<CartesianPose>(
    const research_interface::robot::Move::ControllerMode& controller_type);
template std::unique_ptr<ActiveControlBase> Robot::startControl<CartesianVelocities>(
    const research_interface::robot::Move::ControllerMode& controller_type);

}  // namespace franka
