// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <chrono>
#include <sstream>
#include <type_traits>

#include <franka/joint_velocity_limits.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_model_base.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_traits.h>
#include <research_interface/robot/service_types.h>

#include "franka/commands/get_robot_model_command.hpp"
#include "logging/robot_state_logger.hpp"
#include "network.h"
#include "robot_control.h"

namespace franka {

RobotState convertRobotState(const research_interface::robot::RobotState& robot_state) noexcept;

/**
 * Implementation of the RobotControl interface.
 */
class Robot::Impl : public RobotControl {
 public:
  /**
   * Constructor for the RobotControl implementation.
   *
   * @param network the network connection to the robot
   * @param log_size the size of the log
   * @param realtime_config the realtime configuration
   */
  explicit Impl(std::unique_ptr<Network> network,
                size_t log_size,
                RealtimeConfig realtime_config = RealtimeConfig::kEnforce);

  // Inherited via RobotControl
  auto realtimeConfig() const noexcept -> RealtimeConfig override;

  auto getUpperJointVelocityLimits(
      const std::array<double, RobotControl::kNumJoints>& joint_positions) const
      -> std::array<double, RobotControl::kNumJoints> override;
  auto getLowerJointVelocityLimits(
      const std::array<double, RobotControl::kNumJoints>& joint_positions) const
      -> std::array<double, RobotControl::kNumJoints> override;
  auto startMotion(research_interface::robot::Move::ControllerMode controller_mode,
                   research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
                   const research_interface::robot::Move::Deviation& maximum_path_deviation,
                   const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation,
                   bool use_async_motion_generator,
                   const std::optional<std::vector<double>>& maximum_velocities)
      -> uint32_t override;
  auto cancelMotion(uint32_t motion_id) -> void override;
  auto finishMotion(
      uint32_t motion_id,
      const std::optional<research_interface::robot::MotionGeneratorCommand>& motion_command,
      const std::optional<research_interface::robot::ControllerCommand>& control_command)
      -> void override;
  auto updateMotion(
      const std::optional<research_interface::robot::MotionGeneratorCommand>& motion_command,
      const std::optional<research_interface::robot::ControllerCommand>& control_command)
      -> RobotState override;
  auto throwOnMotionError(const RobotState& robot_state, uint32_t motion_id) -> void override;

  /**
   * Blocks and waits for the next received robot state.
   *
   * @return RobotState the current robot state
   */
  virtual RobotState readOnce();

  /**
   * Updates the joint-level based torque commands of an active joint effort control
   *
   * @param control_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const Torques& control_input);

  /**
   * Updates the motion generator with the given motion generator input
   *
   * @param motion_generator_input the new joint position motion generator input
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const JointPositions& motion_generator_input);

  /**
   * Updates the motion generator with the given motion generator input
   *
   * @param motion_generator_input the new joint velocity motion generator input
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const JointVelocities& motion_generator_input);

  /**
   * Updates the motion generator with the given motion generator input
   *
   * @param motion_generator_input the new Cartesian pose motion generator input
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const CartesianPose& motion_generator_input);

  /**
   * Updates the motion generator with the given motion generator input
   *
   * @param motion_generator_input the new Cartesian velocity motion generator input
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const CartesianVelocities& motion_generator_input);

  /**
   * Updates the motion generator and the controller with the given motion generator and control
   * input
   *
   * @param motion_generator_input the new joint position motion generator input
   * @param control_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const JointPositions& motion_generator_input,
                         const Torques& control_input);

  /**
   * Updates the motion generator and the controller with the given motion generator and control
   * input
   *
   * @param motion_generator_input the new joint velocity motion generator input
   * @param control_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const JointVelocities& motion_generator_input,
                         const Torques& control_input);

  /**
   * Updates the motion generator and the controller with the given motion generator and control
   * input
   *
   * @param motion_generator_input the new Cartesian pose motion generator input
   * @param control_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const CartesianPose& motion_generator_input, const Torques& control_input);

  /**
   * Updates the motion generator and the controller with the given motion generator and control
   * input
   *
   * @param motion_generator_input the new Cartesian velocity motion generator input
   * @param control_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const CartesianVelocities& motion_generator_input,
                         const Torques& control_input);

  /**
   * @return ServerVersion the software version of the connected robot
   */
  ServerVersion serverVersion() const noexcept;

  /**
   * Finishes a running torque-control
   *
   * @param motion_id the id of the running control process
   * @param control_input the final control-input
   */
  void finishMotion(uint32_t motion_id, const Torques& control_input);

  /**
   * Executes the given command.
   *
   * @tparam T The type of the command to execute.
   * @tparam ReturnType The return type of the command.
   * @tparam TArgs The arguments of the command.
   */
  template <typename T, typename ReturnType = uint32_t, typename... TArgs>
  ReturnType executeCommand(TArgs... /* args */);

  /**
   * Loads the model with the given stringified URDF model.
   *
   * @param urdf_model the stringified URDF model
   * @return Model the loaded model
   */
  static Model loadModel(const std::string& urdf_model);

  // for the unit tests
  static Model loadModel(std::unique_ptr<RobotModelBase> robot_model);

  static research_interface::robot::ControllerCommand createControllerCommand(
      const Torques& control_input);

  static research_interface::robot::MotionGeneratorCommand createMotionCommand(
      const JointPositions& motion_input);

  static research_interface::robot::MotionGeneratorCommand createMotionCommand(
      const JointVelocities& motion_input);

  static research_interface::robot::MotionGeneratorCommand createMotionCommand(
      const CartesianPose& motion_input);

  static research_interface::robot::MotionGeneratorCommand createMotionCommand(
      const CartesianVelocities& motion_input);

 protected:
  bool motionGeneratorRunning() const noexcept;
  bool controllerRunning() const noexcept;

 private:
  RobotState current_state_;

  template <typename MotionGeneratorType>
  void writeOnce(const MotionGeneratorType& motion_generator_input);

  template <typename MotionGeneratorType>
  void writeOnce(const MotionGeneratorType& motion_generator_input, const Torques& control_input);

  std::string commandNotPossibleMsg() const {
    std::stringstream stringstream;
    stringstream << " command rejected: command not possible in the current mode ("
                 << static_cast<franka::RobotMode>(robot_mode_) << ")!";
    if (robot_mode_ == research_interface::robot::RobotMode::kOther) {
      stringstream << " Did you open the brakes?";
    }
    return stringstream.str();
  }

  template <typename T>
  using IsBaseOfGetterSetter =
      std::is_base_of<research_interface::robot::GetterSetterCommandBase<T, T::kCommand>, T>;

  template <typename T>
  std::enable_if_t<IsBaseOfGetterSetter<T>::value> handleCommandResponse(
      const typename T::Response& response) const {
    using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

    switch (response.status) {
      case T::Status::kSuccess:
        break;
      case T::Status::kCommandNotPossibleRejected:
        throw CommandException("libfranka: "s + research_interface::robot::CommandTraits<T>::kName +
                               commandNotPossibleMsg());
      case T::Status::kInvalidArgumentRejected:
        throw CommandException("libfranka: "s + research_interface::robot::CommandTraits<T>::kName +
                               " command rejected: invalid argument!");
      case T::Status::kCommandRejectedDueToActivatedSafetyFunctions:
        throw CommandException("libfranka: "s + research_interface::robot::CommandTraits<T>::kName +
                               " command rejected due to activated safety function! Please disable "
                               "all safety functions. ");
      default:
        throw ProtocolException("libfranka: Unexpected response while handling "s +
                                research_interface::robot::CommandTraits<T>::kName + " command!");
    }
  }

  template <typename T>
  std::enable_if_t<!IsBaseOfGetterSetter<T>::value> handleCommandResponse(
      const typename T::Response& response) const {
    using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

    switch (response.status) {
      case T::Status::kSuccess:
        break;
      case T::Status::kCommandNotPossibleRejected:
        throw CommandException("libfranka: "s + research_interface::robot::CommandTraits<T>::kName +
                               commandNotPossibleMsg());
      case T::Status::kCommandRejectedDueToActivatedSafetyFunctions:
        throw CommandException("libfranka: "s + research_interface::robot::CommandTraits<T>::kName +
                               " command rejected due to activated safety function! Please disable "
                               "all safety functions.");
      default:
        throw ProtocolException("libfranka: Unexpected response while handling "s +
                                research_interface::robot::CommandTraits<T>::kName + " command!");
    }
  }

  template <typename T,
            typename ReturnType,
            typename = std::enable_if_t<!std::is_same_v<ReturnType, void>>>
  ReturnType handleCommandResponse(const typename T::Response& response) const;

  research_interface::robot::RobotCommand sendRobotCommand(
      const std::optional<research_interface::robot::MotionGeneratorCommand>& motion_command,
      const std::optional<research_interface::robot::ControllerCommand>& control_command) const;
  research_interface::robot::RobotState receiveRobotState();
  void updateState(const research_interface::robot::RobotState& robot_state);

  std::unique_ptr<Network> network_;  // NOLINT(readability-identifier-naming)

  RobotStateLogger logger_;  // NOLINT(readability-identifier-naming)

  const RealtimeConfig realtime_config_;  // NOLINT(readability-identifier-naming)
  uint16_t ri_version_;                   // NOLINT(readability-identifier-naming)

  research_interface::robot::RobotMode robot_mode_ =  // NOLINT(readability-identifier-naming)
      research_interface::robot::RobotMode::kOther;
  research_interface::robot::MotionGeneratorMode
      motion_generator_mode_;  // NOLINT(readability-identifier-naming)
  research_interface::robot::MotionGeneratorMode
      current_move_motion_generator_mode_ =  // NOLINT(readability-identifier-naming)
      research_interface::robot::MotionGeneratorMode::kIdle;
  research_interface::robot::ControllerMode
      controller_mode_ =  // NOLINT(readability-identifier-naming)
      research_interface::robot::ControllerMode::kOther;
  research_interface::robot::ControllerMode
      current_move_controller_mode_;                        // NOLINT(readability-identifier-naming)
  uint64_t message_id_;                                     // NOLINT(readability-identifier-naming)
  JointVelocityLimitsConfig joint_velocity_limits_config_;  // NOLINT(readability-identifier-naming)
};

template <>
inline GetRobotModelResult
Robot::Impl::handleCommandResponse<research_interface::robot::GetRobotModel, GetRobotModelResult>(
    const research_interface::robot::GetRobotModel::Response& response) const {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

  switch (response.status) {
    case research_interface::robot::GetRobotModel::Status::kSuccess:
      return GetRobotModelResult{.robot_model_urdf = response.robot_model};
    case research_interface::robot::GetRobotModel::Status::kCommandNotPossibleRejected:
      throw CommandException("libfranka: "s +
                             research_interface::robot::CommandTraits<
                                 research_interface::robot::GetRobotModel>::kName +
                             commandNotPossibleMsg());
    case research_interface::robot::GetRobotModel::Status::
        kCommandRejectedDueToActivatedSafetyFunctions:
      throw CommandException("libfranka: "s +
                             research_interface::robot::CommandTraits<
                                 research_interface::robot::GetRobotModel>::kName +
                             " command rejected due to activated safety function! Please disable "
                             "all safety functions.");
    default:
      throw ProtocolException("libfranka: Unexpected response while handling "s +
                              research_interface::robot::CommandTraits<
                                  research_interface::robot::GetRobotModel>::kName +
                              " command!");
  }
}

template <>
inline void Robot::Impl::handleCommandResponse<research_interface::robot::Move>(
    const research_interface::robot::Move::Response& response) const {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

  switch (response.status) {
    case research_interface::robot::Move::Status::kSuccess:
      break;
    case research_interface::robot::Move::Status::kMotionStarted:
      if (motionGeneratorRunning()) {
        throw ProtocolException(
            "libfranka: "s +
            research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
            " received unexpected motion started message.");
      }
      break;
    case research_interface::robot::Move::Status::kEmergencyAborted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command aborted: User Stop pressed!");
    case research_interface::robot::Move::Status::kReflexAborted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command aborted: motion aborted by reflex!");
    case research_interface::robot::Move::Status::kInputErrorAborted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command aborted: invalid input provided!");
    case research_interface::robot::Move::Status::kCommandNotPossibleRejected:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          commandNotPossibleMsg());
    case research_interface::robot::Move::Status::kStartAtSingularPoseRejected:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command rejected: cannot start at singular pose!");
    case research_interface::robot::Move::Status::kInvalidArgumentRejected:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command rejected: maximum path deviation out of range!");
    case research_interface::robot::Move::Status::kPreempted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command preempted!");
    case research_interface::robot::Move::Status::kAborted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command aborted!");
    case research_interface::robot::Move::Status::kPreemptedDueToActivatedSafetyFunctions:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command preempted due to activated safety function! Please disable all safety "
          "functions.");
    case research_interface::robot::Move::Status::kCommandRejectedDueToActivatedSafetyFunctions:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command rejected due to activated safety function! Please disable all safety "
          "functions.");
    default:
      throw ProtocolException(
          "libfranka: Unexpected response while handling "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command!");
  }
}

template <>
inline void Robot::Impl::handleCommandResponse<research_interface::robot::StopMove>(
    const research_interface::robot::StopMove::Response& response) const {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

  switch (response.status) {
    case research_interface::robot::StopMove::Status::kSuccess:
      break;
    case research_interface::robot::StopMove::Status::kCommandNotPossibleRejected:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::StopMove>::kName +
          commandNotPossibleMsg());
    case research_interface::robot::StopMove::Status::kAborted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::StopMove>::kName +
          commandNotPossibleMsg());
    case research_interface::robot::StopMove::Status::kEmergencyAborted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::StopMove>::kName +
          " command aborted: User Stop pressed!");
    case research_interface::robot::StopMove::Status::kReflexAborted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::StopMove>::kName +
          " command aborted: motion aborted by reflex!");
    case research_interface::robot::StopMove::Status::kCommandRejectedDueToActivatedSafetyFunctions:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command rejected due to activated safety function! Please disable all safety "
          "functions.");
    default:
      throw ProtocolException(
          "libfranka: Unexpected response while handling "s +
          research_interface::robot::CommandTraits<research_interface::robot::StopMove>::kName +
          " command!");
  }
}

template <>
inline void Robot::Impl::handleCommandResponse<research_interface::robot::AutomaticErrorRecovery>(
    const research_interface::robot::AutomaticErrorRecovery::Response& response) const {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

  switch (response.status) {
    case research_interface::robot::AutomaticErrorRecovery::Status::kSuccess:
      break;
    case research_interface::robot::AutomaticErrorRecovery::Status::kEmergencyAborted:
      throw CommandException("libfranka: "s +
                             research_interface::robot::CommandTraits<
                                 research_interface::robot::AutomaticErrorRecovery>::kName +
                             " command aborted: User Stop pressed!");
    case research_interface::robot::AutomaticErrorRecovery::Status::kReflexAborted:
      throw CommandException("libfranka: "s +
                             research_interface::robot::CommandTraits<
                                 research_interface::robot::AutomaticErrorRecovery>::kName +
                             " command aborted: motion aborted by reflex!");
    case research_interface::robot::AutomaticErrorRecovery::Status::kCommandNotPossibleRejected:
      throw CommandException("libfranka: "s +
                             research_interface::robot::CommandTraits<
                                 research_interface::robot::AutomaticErrorRecovery>::kName +
                             commandNotPossibleMsg());
    case research_interface::robot::AutomaticErrorRecovery::Status::
        kManualErrorRecoveryRequiredRejected:
      throw CommandException("libfranka: "s +
                             research_interface::robot::CommandTraits<
                                 research_interface::robot::AutomaticErrorRecovery>::kName +
                             " command rejected: manual error recovery required!");
    case research_interface::robot::AutomaticErrorRecovery::Status::kAborted:
      throw CommandException("libfranka: "s +
                             research_interface::robot::CommandTraits<
                                 research_interface::robot::AutomaticErrorRecovery>::kName +
                             " command aborted!");
    case research_interface::robot::AutomaticErrorRecovery::Status::
        kCommandRejectedDueToActivatedSafetyFunctions:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command rejected due to activated safety function! Please disable all safety "
          "functions.");
    default:
      throw ProtocolException("libfranka: Unexpected response while handling "s +
                              research_interface::robot::CommandTraits<
                                  research_interface::robot::AutomaticErrorRecovery>::kName +
                              " command!");
  }
}

template <typename T, typename ReturnType, typename... TArgs>
ReturnType Robot::Impl::executeCommand(TArgs... args) {
  uint32_t command_id = network_->tcpSendRequest<T>(args...);
  typename T::Response response = network_->tcpBlockingReceiveResponse<T>(command_id);
  handleCommandResponse<T>(response);
  return command_id;
}

template <>
inline GetRobotModelResult
Robot::Impl::executeCommand<research_interface::robot::GetRobotModel, GetRobotModelResult>() {
  uint32_t command_id = network_->tcpSendRequest<research_interface::robot::GetRobotModel>();
  research_interface::robot::GetRobotModel::Response response =
      network_->tcpBlockingReceiveResponse<research_interface::robot::GetRobotModel>(command_id);
  auto get_robot_model_result =
      handleCommandResponse<research_interface::robot::GetRobotModel, GetRobotModelResult>(
          response);
  return get_robot_model_result;
}

}  // namespace franka
