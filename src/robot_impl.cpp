// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "robot_impl.h"

#include <sstream>

#include "franka/control_tools.h"
#include "franka/logging/logger.hpp"
#include "franka/rate_limiting.h"
#include "load_calculations.h"

namespace franka {

namespace {

inline ControlException createControlException(const char* message,
                                               research_interface::robot::Move::Status move_status,
                                               const Errors& reflex_errors,
                                               const std::vector<Record>& log) {
  std::ostringstream message_stream;
  message_stream << message;
  if (move_status == decltype(move_status)::kReflexAborted) {
    message_stream << " " << reflex_errors;

    if (log.size() >= 2) {
      // Count number of lost packets in the last and before last packets.
      uint64_t lost_packets =
          log[log.size() - 1].state.time.toMSec() - log[log.size() - 2].state.time.toMSec() - 1;
      // Read second to last control_command_success_rate since the last one will always be zero and
      // consider in the success rate assumming all previous packets were lost.
      message_stream << std::endl
                     << "control_command_success_rate: "
                     << (log[log.size() - 2].state.control_command_success_rate *
                         (1 - static_cast<double>(lost_packets) / 100));
      // Packets lost in a row
      if (lost_packets > 0) {
        message_stream << " packets lost in a row in the last sample: " << lost_packets;
      }
    }
  }

  logging::logError(message_stream.str());
  return ControlException(message_stream.str(), log);
}

}  // anonymous namespace

Robot::Impl::Impl(std::unique_ptr<Network> network, size_t log_size, RealtimeConfig realtime_config)
    : network_{std::move(network)}, logger_{log_size}, realtime_config_{realtime_config} {
  if (network_ == nullptr) {
    throw std::invalid_argument("libfranka robot: Invalid network argument");
  }

  // Configure the thread priority to being set to the highest possible value.
  bool throw_on_error = realtime_config_ == RealtimeConfig::kEnforce;
  std::string error_message;
  logging::logInfo("libfranka: Setting thread priority to the highest possible value.");
  if (!setCurrentThreadToHighestSchedulerPriority(&error_message) && throw_on_error) {
    logging::logError(error_message);
    throw RealtimeException(error_message);
  }

  if (throw_on_error && !hasRealtimeKernel()) {
    logging::logError("libfranka: Running kernel does not have realtime capabilities.");
    throw RealtimeException("libfranka: Running kernel does not have realtime capabilities.");
  }

  connect<research_interface::robot::Connect, research_interface::robot::kVersion>(*network_,
                                                                                   &ri_version_);
  updateState(network_->udpBlockingReceive<research_interface::robot::RobotState>());

  auto get_robot_model =
      this->executeCommand<research_interface::robot::GetRobotModel, GetRobotModelResult>();
  joint_velocity_limits_config_ = JointVelocityLimitsConfig(get_robot_model.robot_model_urdf);
}

RobotState Robot::Impl::updateMotion(
    const std::optional<research_interface::robot::MotionGeneratorCommand>& motion_command,
    const std::optional<research_interface::robot::ControllerCommand>& control_command) {
  network_->tcpThrowIfConnectionClosed();

  research_interface::robot::RobotCommand robot_command =
      sendRobotCommand(motion_command, control_command);

  RobotState state = convertRobotState(receiveRobotState());
  logger_.log(state, robot_command);

  return state;
}

void Robot::Impl::throwOnMotionError(const RobotState& robot_state, uint32_t motion_id) {
  if (robot_state.robot_mode != RobotMode::kMove ||
      motion_generator_mode_ != current_move_motion_generator_mode_ ||
      controller_mode_ != current_move_controller_mode_) {
    // We detect a move error by changes in the robot state and we will receive a TCP response to
    // the Move command.
    auto response =
        network_->tcpBlockingReceiveResponse<research_interface::robot::Move>(motion_id);
    try {
      handleCommandResponse<research_interface::robot::Move>(response);
    } catch (const CommandException& e) {
      throw createControlException(e.what(), response.status, robot_state.last_motion_errors,
                                   logger_.flush());
    }
    throw ProtocolException("Unexpected reply to a Move command");
  }
}

RobotState Robot::Impl::readOnce() {
  current_state_ = convertRobotState(receiveRobotState());
  return current_state_;
}

void Robot::Impl::writeOnce(const Torques& control_input) {
  research_interface::robot::ControllerCommand control_command =
      createControllerCommand(control_input);
  network_->tcpThrowIfConnectionClosed();

  sendRobotCommand(std::nullopt, control_command);
}

template <typename MotionGeneratorType>
void Robot::Impl::writeOnce(const MotionGeneratorType& motion_generator_input,
                            const Torques& control_input) {
  auto motion_command = createMotionCommand(motion_generator_input);
  auto control_command = createControllerCommand(control_input);
  network_->tcpThrowIfConnectionClosed();
  sendRobotCommand(motion_command, control_command);
}

template <typename MotionGeneratorType>
void Robot::Impl::writeOnce(const MotionGeneratorType& motion_generator_input) {
  auto motion_command = createMotionCommand(motion_generator_input);
  network_->tcpThrowIfConnectionClosed();
  sendRobotCommand(motion_command, std::nullopt);
}

research_interface::robot::RobotCommand Robot::Impl::sendRobotCommand(
    const std::optional<research_interface::robot::MotionGeneratorCommand>& motion_command,
    const std::optional<research_interface::robot::ControllerCommand>& control_command) const {
  research_interface::robot::RobotCommand robot_command{};
  if (!motion_command.has_value() && !control_command.has_value()) {
    // No new command, return
    return robot_command;
  }

  robot_command.message_id = message_id_;
  if (motion_command.has_value()) {
    if (current_move_motion_generator_mode_ ==
            research_interface::robot::MotionGeneratorMode::kIdle ||
        current_move_motion_generator_mode_ ==
            research_interface::robot::MotionGeneratorMode::kNone) {
      throw ControlException(
          "libfranka robot: Trying to send motion command, but no motion generator running!");
    }
    robot_command.motion = *motion_command;
  }

  if (control_command.has_value()) {
    if (current_move_controller_mode_ !=
        research_interface::robot::ControllerMode::kExternalController) {
      throw ControlException(
          "libfranka robot: Trying to send control command, but no controller running!");
    }
    robot_command.control = *control_command;
  }

  if ((current_move_motion_generator_mode_ !=
           research_interface::robot::MotionGeneratorMode::kIdle &&
       current_move_motion_generator_mode_ !=
           research_interface::robot::MotionGeneratorMode::kNone) &&
      current_move_controller_mode_ ==
          research_interface::robot::ControllerMode::kExternalController &&
      (!motion_command.has_value() || !control_command.has_value())) {
    throw ControlException("libfranka robot: Trying to send partial robot command!");
  }

  network_->udpSend<research_interface::robot::RobotCommand>(robot_command);

  return robot_command;
}

research_interface::robot::RobotState Robot::Impl::receiveRobotState() {
  research_interface::robot::RobotState latest_accepted_state;
  latest_accepted_state.message_id = message_id_;

  // If states are already available on the socket, use the one with the most recent message ID.
  research_interface::robot::RobotState received_state{};
  while (network_->udpReceive(&received_state)) {
    if (received_state.message_id > latest_accepted_state.message_id) {
      latest_accepted_state = received_state;
    }
  }

  // If there was no valid state on the socket, we need to wait.
  while (latest_accepted_state.message_id == message_id_) {
    received_state = network_->udpBlockingReceive<decltype(received_state)>();
    if (received_state.message_id > latest_accepted_state.message_id) {
      latest_accepted_state = received_state;
    }
  }

  updateState(latest_accepted_state);
  return latest_accepted_state;
}

void Robot::Impl::updateState(const research_interface::robot::RobotState& robot_state) {
  robot_mode_ = robot_state.robot_mode;
  motion_generator_mode_ = robot_state.motion_generator_mode;
  controller_mode_ = robot_state.controller_mode;
  message_id_ = robot_state.message_id;
}

Robot::ServerVersion Robot::Impl::serverVersion() const noexcept {
  return ri_version_;
}

bool Robot::Impl::motionGeneratorRunning() const noexcept {
  return motion_generator_mode_ != research_interface::robot::MotionGeneratorMode::kIdle &&
         motion_generator_mode_ != research_interface::robot::MotionGeneratorMode::kNone;
}

bool Robot::Impl::controllerRunning() const noexcept {
  return controller_mode_ == research_interface::robot::ControllerMode::kExternalController;
}

RealtimeConfig Robot::Impl::realtimeConfig() const noexcept {
  return realtime_config_;
}

std::array<double, RobotControl::kNumJoints> Robot::Impl::getUpperJointVelocityLimits(
    const std::array<double, RobotControl::kNumJoints>& joint_positions) const {
  return joint_velocity_limits_config_.getUpperJointVelocityLimits(joint_positions);
}

std::array<double, RobotControl::kNumJoints> Robot::Impl::getLowerJointVelocityLimits(
    const std::array<double, RobotControl::kNumJoints>& joint_positions) const {
  return joint_velocity_limits_config_.getLowerJointVelocityLimits(joint_positions);
}

uint32_t Robot::Impl::startMotion(
    research_interface::robot::Move::ControllerMode controller_mode,
    research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
    const research_interface::robot::Move::Deviation& maximum_path_deviation,
    const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation,
    bool use_async_motion_generator,
    const std::optional<std::vector<double>>& maximum_velocities) {
  if (motionGeneratorRunning() || controllerRunning()) {
    throw ControlException("libfranka robot: Attempted to start multiple motions!");
  }

  using MotionGeneratorMode = research_interface::robot::Move::MotionGeneratorMode;
  switch (motion_generator_mode) {
    case MotionGeneratorMode::kJointPosition:
      current_move_motion_generator_mode_ =
          decltype(current_move_motion_generator_mode_)::kJointPosition;
      break;
    case MotionGeneratorMode::kJointVelocity:
      current_move_motion_generator_mode_ =
          decltype(current_move_motion_generator_mode_)::kJointVelocity;
      break;
    case MotionGeneratorMode::kCartesianPosition:
      current_move_motion_generator_mode_ =
          decltype(current_move_motion_generator_mode_)::kCartesianPosition;
      break;
    case MotionGeneratorMode::kCartesianVelocity:
      current_move_motion_generator_mode_ =
          decltype(current_move_motion_generator_mode_)::kCartesianVelocity;
      break;
    case MotionGeneratorMode::kNone:
      current_move_motion_generator_mode_ = decltype(current_move_motion_generator_mode_)::kNone;
      break;
    default:
      throw std::invalid_argument("libfranka: Invalid motion generator mode given.");
  }

  switch (controller_mode) {
    case decltype(controller_mode)::kJointImpedance:
      current_move_controller_mode_ = decltype(current_move_controller_mode_)::kJointImpedance;
      break;
    case decltype(controller_mode)::kCartesianImpedance:
      current_move_controller_mode_ = decltype(current_move_controller_mode_)::kCartesianImpedance;
      break;
    case decltype(controller_mode)::kExternalController:
      current_move_controller_mode_ = decltype(current_move_controller_mode_)::kExternalController;
      break;
    default:
      throw std::invalid_argument("libfranka robot: Invalid controller mode given.");
  }

  const uint32_t move_command_id = executeCommand<research_interface::robot::Move>(
      controller_mode, motion_generator_mode, maximum_path_deviation, maximum_goal_pose_deviation,
      use_async_motion_generator, maximum_velocities);

  RobotState robot_state{};
  while (motion_generator_mode_ != current_move_motion_generator_mode_ ||
         controller_mode_ != current_move_controller_mode_) {
    try {
      if (network_->tcpReceiveResponse<research_interface::robot::Move>(
              move_command_id, [this](const auto& response) {
                return this->handleCommandResponse<research_interface::robot::Move>(response);
              })) {
        break;
      }
    } catch (const CommandException& e) {
      throw ControlException(e.what());
    }

    robot_state = updateMotion(std::nullopt, std::nullopt);
  }

  logger_.flush();

  return move_command_id;
}

void Robot::Impl::finishMotion(
    uint32_t motion_id,
    const std::optional<research_interface::robot::MotionGeneratorCommand>& motion_command,
    const std::optional<research_interface::robot::ControllerCommand>& control_command) {
  if (!motionGeneratorRunning() && !controllerRunning()) {
    current_move_motion_generator_mode_ = research_interface::robot::MotionGeneratorMode::kIdle;
    current_move_controller_mode_ = research_interface::robot::ControllerMode::kOther;
    return;
  }

  auto motion_finished_command = motion_command;
  auto controller_finished_command = control_command;
  if (motion_command.has_value()) {
    motion_finished_command->motion_generation_finished = true;
  } else if (control_command.has_value()) {
    controller_finished_command->torque_command_finished = true;
  } else {
    logging::logError("libfranka robot: No motion generator or control command given!");
    throw ControlException("libfranka robot: No motion generator or control command given!");
  }

  // The TCP response for the finished Move might arrive while the robot state still shows that the
  // motion is running, or afterwards. To handle both situations, we do not process TCP packages in
  // this loop and explicitly wait for the Move response over TCP afterwards.
  RobotState robot_state{};
  while (motionGeneratorRunning() || controllerRunning()) {
    robot_state = updateMotion(motion_finished_command, controller_finished_command);
  }

  auto response = network_->tcpBlockingReceiveResponse<research_interface::robot::Move>(motion_id);
  if (response.status == research_interface::robot::Move::Status::kReflexAborted) {
    throw createControlException("Motion finished commanded, but the robot is still moving!",
                                 response.status, robot_state.last_motion_errors, logger_.flush());
  }
  try {
    handleCommandResponse<research_interface::robot::Move>(response);
  } catch (const CommandException& e) {
    throw createControlException(e.what(), response.status, robot_state.last_motion_errors,
                                 logger_.flush());
  }
  current_move_motion_generator_mode_ = research_interface::robot::MotionGeneratorMode::kIdle;
  current_move_controller_mode_ = research_interface::robot::ControllerMode::kOther;
}

void Robot::Impl::finishMotion(uint32_t motion_id, const Torques& control_input) {
  research_interface::robot::ControllerCommand controller_command =
      createControllerCommand(control_input);

  if (motionGeneratorRunning()) {
    logging::logError(
        "libfranka robot: Motion generator is still running! Can't stop external "
        "control");
    throw std::invalid_argument(
        "libfranka robot: Motion generator is still running! Can't stop external control");
  }

  finishMotion(motion_id, std::nullopt, controller_command);
}

research_interface::robot::MotionGeneratorCommand Robot::Impl::createMotionCommand(
    const JointPositions& motion_input) {
  checkFinite(motion_input.q);

  research_interface::robot::MotionGeneratorCommand motion_command{};
  motion_command.q_c = motion_input.q;

  return motion_command;
}

research_interface::robot::MotionGeneratorCommand Robot::Impl::createMotionCommand(
    const JointVelocities& motion_input) {
  checkFinite(motion_input.dq);

  research_interface::robot::MotionGeneratorCommand motion_command{};
  motion_command.dq_c = motion_input.dq;

  return motion_command;
}

research_interface::robot::MotionGeneratorCommand Robot::Impl::createMotionCommand(
    const CartesianPose& motion_input) {
  checkMatrix(motion_input.O_T_EE);

  research_interface::robot::MotionGeneratorCommand motion_command{};
  motion_command.O_T_EE_c = motion_input.O_T_EE;

  if (motion_input.hasElbow()) {
    motion_command.valid_elbow = true;
    motion_command.elbow_c = motion_input.elbow;
    checkElbow(motion_command.elbow_c);
  } else {
    motion_command.valid_elbow = false;
    motion_command.elbow_c = {};
  }

  return motion_command;
}

research_interface::robot::MotionGeneratorCommand Robot::Impl::createMotionCommand(
    const CartesianVelocities& motion_input) {
  checkFinite(motion_input.O_dP_EE);

  research_interface::robot::MotionGeneratorCommand motion_command{};
  motion_command.O_dP_EE_c = motion_input.O_dP_EE;

  if (motion_input.hasElbow()) {
    motion_command.valid_elbow = true;
    motion_command.elbow_c = motion_input.elbow;
    checkElbow(motion_command.elbow_c);
  } else {
    motion_command.valid_elbow = false;
    motion_command.elbow_c = {};
  }

  return motion_command;
}

research_interface::robot::ControllerCommand Robot::Impl::createControllerCommand(
    const Torques& control_input) {
  checkFinite(control_input.tau_J);

  research_interface::robot::ControllerCommand control_command{};
  control_command.tau_J_d = control_input.tau_J;

  return control_command;
}

void Robot::Impl::cancelMotion(uint32_t motion_id) {
  if (!network_->isTcpSocketAlive()) {
    logging::logWarn("libfranka robot: TCP connection is closed. Cannot cancel motion.");
    return;
  }

  try {
    executeCommand<research_interface::robot::StopMove>();
  } catch (const CommandException& e) {
    throw ControlException(e.what());
  }

  research_interface::robot::RobotState robot_state;
  do {
    robot_state = receiveRobotState();
  } while (motionGeneratorRunning() || controllerRunning());

  // Ignore Move response.
  // TODO (FWA): It is not guaranteed that the Move response won't come later
  network_->tcpReceiveResponse<research_interface::robot::Move>(motion_id, [](auto) {});
  current_move_motion_generator_mode_ = research_interface::robot::MotionGeneratorMode::kIdle;
  current_move_controller_mode_ = research_interface::robot::ControllerMode::kOther;
}

Model Robot::Impl::loadModel(const std::string& urdf_model) {
  return Model(urdf_model);
}

// for the tests
Model Robot::Impl::loadModel(std::unique_ptr<RobotModelBase> robot_model) {
  return Model(std::move(robot_model));
}

RobotState convertRobotState(const research_interface::robot::RobotState& robot_state) noexcept {
  RobotState converted;
  converted.O_T_EE = robot_state.O_T_EE;
  converted.O_T_EE_d = robot_state.O_T_EE_d;
  converted.F_T_NE = robot_state.F_T_NE;
  converted.NE_T_EE = robot_state.NE_T_EE;
  converted.F_T_EE = robot_state.F_T_EE;
  converted.EE_T_K = robot_state.EE_T_K;
  converted.m_ee = robot_state.m_ee;
  converted.F_x_Cee = robot_state.F_x_Cee;
  converted.I_ee = robot_state.I_ee;
  converted.m_load = robot_state.m_load;
  converted.F_x_Cload = robot_state.F_x_Cload;
  converted.I_load = robot_state.I_load;
  converted.m_total = robot_state.m_ee + robot_state.m_load;
  converted.F_x_Ctotal = combineCenterOfMass(robot_state.m_ee, robot_state.F_x_Cee,
                                             robot_state.m_load, robot_state.F_x_Cload);
  converted.I_total = combineInertiaTensor(
      robot_state.m_ee, robot_state.F_x_Cee, robot_state.I_ee, robot_state.m_load,
      robot_state.F_x_Cload, robot_state.I_load, converted.m_total, converted.F_x_Ctotal);
  converted.elbow = robot_state.elbow;
  converted.elbow_d = robot_state.elbow_d;
  converted.elbow_c = robot_state.elbow_c;
  converted.delbow_c = robot_state.delbow_c;
  converted.ddelbow_c = robot_state.ddelbow_c;
  converted.tau_J = robot_state.tau_J;
  converted.tau_J_d = robot_state.tau_J_d;
  converted.dtau_J = robot_state.dtau_J;
  converted.q = robot_state.q;
  converted.dq = robot_state.dq;
  converted.q_d = robot_state.q_d;
  converted.dq_d = robot_state.dq_d;
  converted.ddq_d = robot_state.ddq_d;
  converted.joint_contact = robot_state.joint_contact;
  converted.cartesian_contact = robot_state.cartesian_contact;
  converted.joint_collision = robot_state.joint_collision;
  converted.cartesian_collision = robot_state.cartesian_collision;
  converted.tau_ext_hat_filtered = robot_state.tau_ext_hat_filtered;
  converted.O_F_ext_hat_K = robot_state.O_F_ext_hat_K;
  converted.K_F_ext_hat_K = robot_state.K_F_ext_hat_K;
  converted.O_dP_EE_d = robot_state.O_dP_EE_d;
  converted.O_ddP_O = robot_state.O_ddP_O;
  converted.O_T_EE_c = robot_state.O_T_EE_c;
  converted.O_dP_EE_c = robot_state.O_dP_EE_c;
  converted.O_ddP_EE_c = robot_state.O_ddP_EE_c;
  converted.theta = robot_state.theta;
  converted.dtheta = robot_state.dtheta;
  converted.accelerometer_top = robot_state.accelerometer_top;
  converted.accelerometer_bottom = robot_state.accelerometer_bottom;
  converted.current_errors = robot_state.errors;
  converted.last_motion_errors = robot_state.reflex_reason;
  converted.control_command_success_rate = robot_state.control_command_success_rate;
  converted.time = Duration(robot_state.message_id);

  converted.robot_mode = RobotMode::kOther;
  switch (robot_state.robot_mode) {
    case research_interface::robot::RobotMode::kOther:
      converted.robot_mode = RobotMode::kOther;
      break;
    case research_interface::robot::RobotMode::kIdle:
      converted.robot_mode = RobotMode::kIdle;
      break;
    case research_interface::robot::RobotMode::kMove:
      converted.robot_mode = RobotMode::kMove;
      break;
    case research_interface::robot::RobotMode::kGuiding:
      converted.robot_mode = RobotMode::kGuiding;
      break;
    case research_interface::robot::RobotMode::kReflex:
      converted.robot_mode = RobotMode::kReflex;
      break;
    case research_interface::robot::RobotMode::kUserStopped:
      converted.robot_mode = RobotMode::kUserStopped;
      break;
    case research_interface::robot::RobotMode::kAutomaticErrorRecovery:
      converted.robot_mode = RobotMode::kAutomaticErrorRecovery;
      break;
  }

  return converted;
}

template void Robot::Impl::writeOnce<JointPositions>(const JointPositions& motion_generator_input);
template void Robot::Impl::writeOnce<JointVelocities>(
    const JointVelocities& motion_generator_input);
template void Robot::Impl::writeOnce<CartesianPose>(const CartesianPose& motion_generator_input);
template void Robot::Impl::writeOnce<CartesianVelocities>(
    const CartesianVelocities& motion_generator_input);

template void Robot::Impl::writeOnce<JointPositions>(const JointPositions& motion_generator_input,
                                                     const Torques& control_input);

template void Robot::Impl::writeOnce<JointVelocities>(const JointVelocities& motion_generator_input,
                                                      const Torques& control_input);

template void Robot::Impl::writeOnce<CartesianPose>(const CartesianPose& motion_generator_input,
                                                    const Torques& control_input);

template void Robot::Impl::writeOnce<CartesianVelocities>(
    const CartesianVelocities& motion_generator_input,
    const Torques& control_input);

void Robot::Impl::writeOnce(const JointPositions& motion_generator_input) {
  writeOnce<JointPositions>(motion_generator_input);
}

void Robot::Impl::writeOnce(const JointVelocities& motion_generator_input) {
  writeOnce<JointVelocities>(motion_generator_input);
}

void Robot::Impl::writeOnce(const CartesianPose& motion_generator_input) {
  writeOnce<CartesianPose>(motion_generator_input);
}

void Robot::Impl::writeOnce(const CartesianVelocities& motion_generator_input) {
  writeOnce<CartesianVelocities>(motion_generator_input);
}

void Robot::Impl::writeOnce(const JointPositions& motion_generator_input,
                            const Torques& control_input) {
  writeOnce<JointPositions>(motion_generator_input, control_input);
}

void Robot::Impl::writeOnce(const JointVelocities& motion_generator_input,
                            const Torques& control_input) {
  writeOnce<JointVelocities>(motion_generator_input, control_input);
}

void Robot::Impl::writeOnce(const CartesianPose& motion_generator_input,
                            const Torques& control_input) {
  writeOnce<CartesianPose>(motion_generator_input, control_input);
}

void Robot::Impl::writeOnce(const CartesianVelocities& motion_generator_input,
                            const Torques& control_input) {
  writeOnce<CartesianVelocities>(motion_generator_input, control_input);
}

}  // namespace franka
