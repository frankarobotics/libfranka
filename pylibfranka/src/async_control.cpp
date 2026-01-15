// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka/async_control/async_position_control_handler.hpp>
#include <pylibfranka/async_control.hpp>

namespace pylibfranka {

void bind_async_control(py::module& m) {
  auto async_position_control_handler =
      py::class_<franka::AsyncPositionControlHandler,
                 std::shared_ptr<franka::AsyncPositionControlHandler>>(
          m, "AsyncPositionControlHandler", R"pbdoc(
    Handler for asynchronous joint position control
    )pbdoc")
          .def_static("configure", &franka::AsyncPositionControlHandler::configure, R"pbdoc(
    Configure an AsyncPositionControlHandler with the given configuration.

    @param configuration Configuration parameters
    @return ConfigurationResult containing the handler or an error message
      )pbdoc")
          .def("set_joint_position_target",
               &franka::AsyncPositionControlHandler::setJointPositionTarget, R"pbdoc(
    Set a new joint position target.

    @param target Joint position target
    @return CommandResult containing the motion UUID, success flag, and error message
      )pbdoc")
          .def("get_target_feedback", &franka::AsyncPositionControlHandler::getTargetFeedback,
               py::arg("robot_state") = std::nullopt, R"pbdoc(
    Get feedback on the current target.

    @param robot_state Optional current robot state for more detailed feedback
    @return TargetFeedback containing status and error message
      )pbdoc")
          .def("stop_control", &franka::AsyncPositionControlHandler::stopControl, R"pbdoc(
    Stop the position control handler.
      )pbdoc");

  py::class_<franka::AsyncPositionControlHandler::Configuration>(async_position_control_handler,
                                                                 "Configuration", R"pbdoc(
    Configuration parameters for AsyncPositionControlHandler. 
    )pbdoc")
      .def(py::init([](const std::vector<double>& maximum_joint_velocities, double goal_tolerance) {
             if (maximum_joint_velocities.size() != franka::Robot::kNumJoints) {
               throw std::invalid_argument("joint_velocities must have exactly" +
                                           std::to_string(franka::Robot::kNumJoints) + " values");
             }
             franka::AsyncPositionControlHandler::Configuration config{
                 .maximum_joint_velocities = maximum_joint_velocities,
                 .goal_tolerance = goal_tolerance};
             return config;
           }),
           py::arg("maximum_joint_velocities"), py::arg("goal_tolerance"), R"pbdoc(
        Create Configuration.
        
        @param maximum_joint_velocities Maximum joint velocities [rad/s] (7,)
        @param goal_tolerance Goal tolerance [rad]
    )pbdoc")
      .def_readwrite("maximum_joint_velocities",
                     &franka::AsyncPositionControlHandler::Configuration::maximum_joint_velocities)
      .def_readwrite("goal_tolerance",
                     &franka::AsyncPositionControlHandler::Configuration::goal_tolerance);

  py::class_<franka::AsyncPositionControlHandler::ConfigurationResult>(
      async_position_control_handler, "ConfigurationResult", R"pbdoc(
    Result of configuring an AsyncPositionControlHandler.
    )pbdoc")
      .def_readwrite("handler", &franka::AsyncPositionControlHandler::ConfigurationResult::handler)
      .def_readwrite("error_message",
                     &franka::AsyncPositionControlHandler::ConfigurationResult::error_message);

  py::class_<franka::AsyncPositionControlHandler::JointPositionTarget>(
      async_position_control_handler, "JointPositionTarget", R"pbdoc(
    Joint position target for AsyncPositionControlHandler.
    )pbdoc")
      .def(py::init([](const std::vector<double>& joints) {
             if (joints.size() != franka::Robot::kNumJoints) {
               throw std::invalid_argument("joint_positions must have exactly" +
                                           std::to_string(franka::Robot::kNumJoints) + " values");
             }
             franka::AsyncPositionControlHandler::JointPositionTarget tgt;
             std::copy(joints.begin(), joints.end(), tgt.joint_positions.begin());
             return tgt;
           }),
           py::arg("joint_positions"), R"pbdoc(
        Create JointPositionTarget.

        @param joint_positions Target joint positions [rad] (7,)
        )pbdoc")
      .def_readwrite("joint_positions",
                     &franka::AsyncPositionControlHandler::JointPositionTarget::joint_positions);

  py::class_<franka::AsyncPositionControlHandler::CommandResult>(async_position_control_handler,
                                                                 "CommandResult", R"pbdoc(
    Result of setting a joint position target.
    )pbdoc")
      .def_readwrite("motion_uuid",
                     &franka::AsyncPositionControlHandler::CommandResult::motion_uuid)
      .def_readwrite("was_successful",
                     &franka::AsyncPositionControlHandler::CommandResult::was_successful)
      .def_readwrite("error_message",
                     &franka::AsyncPositionControlHandler::CommandResult::error_message);

  py::class_<franka::AsyncPositionControlHandler::TargetFeedback>(async_position_control_handler,
                                                                  "TargetFeedback", R"pbdoc(
    Feedback from a target position command.
    )pbdoc")
      .def_readwrite("status", &franka::AsyncPositionControlHandler::TargetFeedback::status)
      .def_readwrite("error_message",
                     &franka::AsyncPositionControlHandler::TargetFeedback::error_message);
}

}  // namespace pylibfranka