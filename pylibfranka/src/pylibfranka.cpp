// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include "pylibfranka.h"
#include "pygripper.h"

// C++ standard library headers

// Third-party library headers
#include <franka/duration.h>
#include <franka/exception.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <research_interface/robot/service_types.h>

namespace py = pybind11;

namespace {

/**
 * Convert franka::ControllerMode to research_interface::robot::Move::ControllerMode
 * @param mode franka::ControllerMode the mode to convert
 * @return research_interface::robot::Move::ControllerMode
 */
auto convertControllerMode(franka::ControllerMode mode)
    -> research_interface::robot::Move::ControllerMode {
  switch (mode) {
    case franka::ControllerMode::kJointImpedance:
      return research_interface::robot::Move::ControllerMode::kJointImpedance;
    case franka::ControllerMode::kCartesianImpedance:
      return research_interface::robot::Move::ControllerMode::kCartesianImpedance;
    default:
      throw std::invalid_argument("Invalid franka::ControllerMode");
  }
}
}  // namespace

namespace pylibfranka {

PyGripper::PyGripper(const std::string& franka_address)
    : gripper_(std::make_unique<franka::Gripper>(franka_address)) {}

bool PyGripper::homing() {
  return gripper_->homing();
}

bool PyGripper::grasp(double width,
                      double speed,
                      double force,
                      double epsilon_inner,
                      double epsilon_outer) {
  return gripper_->grasp(width, speed, force, epsilon_inner, epsilon_outer);
}

franka::GripperState PyGripper::readOnce() {
  return gripper_->readOnce();
}

bool PyGripper::stop() {
  return gripper_->stop();
}
bool PyGripper::move(double width, double speed) {
  return gripper_->move(width, speed);
}

franka::Gripper::ServerVersion PyGripper::serverVersion() {
  return gripper_->serverVersion();
}

PyRobot::PyRobot(const std::string& franka_address, franka::RealtimeConfig realtime_config)
    : robot_(std::make_unique<franka::Robot>(franka_address, realtime_config)) {}

auto PyRobot::startTorqueControl() -> std::unique_ptr<franka::ActiveControlBase> {
  return robot_->startTorqueControl();
}

auto PyRobot::startJointPositionControl(franka::ControllerMode controller_mode)
    -> std::unique_ptr<franka::ActiveControlBase> {
  return robot_->startJointPositionControl(convertControllerMode(controller_mode));
}

auto PyRobot::startJointVelocityControl(franka::ControllerMode controller_mode)
    -> std::unique_ptr<franka::ActiveControlBase> {
  return robot_->startJointVelocityControl(convertControllerMode(controller_mode));
}

auto PyRobot::startCartesianPoseControl(franka::ControllerMode controller_mode)
    -> std::unique_ptr<franka::ActiveControlBase> {
  return robot_->startCartesianPoseControl(convertControllerMode(controller_mode));
}

void PyRobot::setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                                   const std::array<double, 7>& upper_torque_thresholds,
                                   const std::array<double, 6>& lower_force_thresholds,
                                   const std::array<double, 6>& upper_force_thresholds) {
  robot_->setCollisionBehavior(lower_torque_thresholds, upper_torque_thresholds,
                               lower_torque_thresholds, upper_torque_thresholds,
                               lower_force_thresholds, upper_force_thresholds,
                               lower_force_thresholds, upper_force_thresholds);
}

void PyRobot::setJointImpedance(const std::array<double, 7>& K_theta) {
  robot_->setJointImpedance(K_theta);
}

void PyRobot::setCartesianImpedance(const std::array<double, 6>& K_x) {
  robot_->setCartesianImpedance(K_x);
}

void PyRobot::setK(const std::array<double, 16>& EE_T_K) {
  robot_->setK(EE_T_K);
}

void PyRobot::setEE(const std::array<double, 16>& NE_T_EE) {
  robot_->setEE(NE_T_EE);
}

void PyRobot::setLoad(double load_mass,
                      const std::array<double, 3>& F_x_Cload,
                      const std::array<double, 9>& load_inertia) {
  robot_->setLoad(load_mass, F_x_Cload, load_inertia);
}

void PyRobot::automaticErrorRecovery() {
  robot_->automaticErrorRecovery();
}

franka::RobotState PyRobot::readOnce() {
  return robot_->readOnce();
}

std::unique_ptr<franka::Model> PyRobot::loadModel() {
  return std::make_unique<franka::Model>(robot_->loadModel());
}

void PyRobot::stop() {
  robot_->stop();
}

PYBIND11_MODULE(_pylibfranka, m) {
  m.doc() = "Python bindings for Franka Robotics Robot Control Library";

  // Bind exceptions
  py::register_exception<franka::Exception>(m, "FrankaException");
  py::register_exception<franka::CommandException>(m, "CommandException", PyExc_RuntimeError);
  py::register_exception<franka::NetworkException>(m, "NetworkException", PyExc_RuntimeError);
  py::register_exception<franka::ControlException>(m, "ControlException", PyExc_RuntimeError);
  py::register_exception<franka::InvalidOperationException>(m, "InvalidOperationException",
                                                            PyExc_RuntimeError);
  py::register_exception<franka::RealtimeException>(m, "RealtimeException", PyExc_RuntimeError);

  // Bind Duration
  py::class_<franka::Duration>(m, "Duration", R"pbdoc(
        Time duration representation.
    )pbdoc")
      .def("to_sec", &franka::Duration::toSec, R"pbdoc(
        Convert duration to seconds.

        @return Duration in seconds as float
    )pbdoc");

  // Bind enums
  py::enum_<franka::ControllerMode>(m, "ControllerMode", R"pbdoc(
        Controller mode for motion control.
    )pbdoc")
      .value("JointImpedance", franka::ControllerMode::kJointImpedance,
             "Joint impedance control mode")
      .value("CartesianImpedance", franka::ControllerMode::kCartesianImpedance,
             "Cartesian impedance control mode");

  // Bind RealtimeConfig enum
  py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig", R"pbdoc(
        Real-time configuration options.
    )pbdoc")
      .value("kEnforce", franka::RealtimeConfig::kEnforce, "Enforce real-time requirements")
      .value("kIgnore", franka::RealtimeConfig::kIgnore, "Ignore real-time requirements");

  // Bind RobotMode enum
  py::enum_<franka::RobotMode>(m, "RobotMode", R"pbdoc(
        Robot operating mode.
    )pbdoc")
      .value("Other", franka::RobotMode::kOther, "Other mode")
      .value("Idle", franka::RobotMode::kIdle, "Idle mode")
      .value("Move", franka::RobotMode::kMove, "Move mode")
      .value("Guiding", franka::RobotMode::kGuiding, "Guiding mode")
      .value("Reflex", franka::RobotMode::kReflex, "Reflex mode")
      .value("UserStopped", franka::RobotMode::kUserStopped, "User stopped mode")
      .value("AutomaticErrorRecovery", franka::RobotMode::kAutomaticErrorRecovery,
             "Automatic error recovery mode");

  // Bind Errors struct
  py::class_<franka::Errors>(m, "Errors", R"pbdoc(
        Robot error state containing boolean flags for all possible errors.
    )pbdoc")
      .def(py::init<>())
      .def("__bool__", &franka::Errors::operator bool, R"pbdoc(
        Check if any error is present.

        @return True if any error flag is set
    )pbdoc")
      .def("__str__", &franka::Errors::operator std::string, R"pbdoc(
        Get string representation of active errors.

        @return Comma-separated list of active error names
    )pbdoc")
      .def_property_readonly(
          "joint_position_limits_violation",
          [](const franka::Errors& self) { return self.joint_position_limits_violation; })
      .def_property_readonly(
          "cartesian_position_limits_violation",
          [](const franka::Errors& self) { return self.cartesian_position_limits_violation; })
      .def_property_readonly(
          "self_collision_avoidance_violation",
          [](const franka::Errors& self) { return self.self_collision_avoidance_violation; })
      .def_property_readonly(
          "joint_velocity_violation",
          [](const franka::Errors& self) { return self.joint_velocity_violation; })
      .def_property_readonly(
          "cartesian_velocity_violation",
          [](const franka::Errors& self) { return self.cartesian_velocity_violation; })
      .def_property_readonly(
          "force_control_safety_violation",
          [](const franka::Errors& self) { return self.force_control_safety_violation; })
      .def_property_readonly("joint_reflex",
                             [](const franka::Errors& self) { return self.joint_reflex; })
      .def_property_readonly("cartesian_reflex",
                             [](const franka::Errors& self) { return self.cartesian_reflex; })
      .def_property_readonly(
          "max_goal_pose_deviation_violation",
          [](const franka::Errors& self) { return self.max_goal_pose_deviation_violation; })
      .def_property_readonly(
          "max_path_pose_deviation_violation",
          [](const franka::Errors& self) { return self.max_path_pose_deviation_violation; })
      .def_property_readonly("cartesian_velocity_profile_safety_violation",
                             [](const franka::Errors& self) {
                               return self.cartesian_velocity_profile_safety_violation;
                             })
      .def_property_readonly("joint_position_motion_generator_start_pose_invalid",
                             [](const franka::Errors& self) {
                               return self.joint_position_motion_generator_start_pose_invalid;
                             })
      .def_property_readonly("joint_motion_generator_position_limits_violation",
                             [](const franka::Errors& self) {
                               return self.joint_motion_generator_position_limits_violation;
                             })
      .def_property_readonly("joint_motion_generator_velocity_limits_violation",
                             [](const franka::Errors& self) {
                               return self.joint_motion_generator_velocity_limits_violation;
                             })
      .def_property_readonly("joint_motion_generator_velocity_discontinuity",
                             [](const franka::Errors& self) {
                               return self.joint_motion_generator_velocity_discontinuity;
                             })
      .def_property_readonly("joint_motion_generator_acceleration_discontinuity",
                             [](const franka::Errors& self) {
                               return self.joint_motion_generator_acceleration_discontinuity;
                             })
      .def_property_readonly("cartesian_position_motion_generator_start_pose_invalid",
                             [](const franka::Errors& self) {
                               return self.cartesian_position_motion_generator_start_pose_invalid;
                             })
      .def_property_readonly("cartesian_motion_generator_elbow_limit_violation",
                             [](const franka::Errors& self) {
                               return self.cartesian_motion_generator_elbow_limit_violation;
                             })
      .def_property_readonly("cartesian_motion_generator_velocity_limits_violation",
                             [](const franka::Errors& self) {
                               return self.cartesian_motion_generator_velocity_limits_violation;
                             })
      .def_property_readonly("cartesian_motion_generator_velocity_discontinuity",
                             [](const franka::Errors& self) {
                               return self.cartesian_motion_generator_velocity_discontinuity;
                             })
      .def_property_readonly("cartesian_motion_generator_acceleration_discontinuity",
                             [](const franka::Errors& self) {
                               return self.cartesian_motion_generator_acceleration_discontinuity;
                             })
      .def_property_readonly("cartesian_motion_generator_elbow_sign_inconsistent",
                             [](const franka::Errors& self) {
                               return self.cartesian_motion_generator_elbow_sign_inconsistent;
                             })
      .def_property_readonly("cartesian_motion_generator_start_elbow_invalid",
                             [](const franka::Errors& self) {
                               return self.cartesian_motion_generator_start_elbow_invalid;
                             })
      .def_property_readonly(
          "cartesian_motion_generator_joint_position_limits_violation",
          [](const franka::Errors& self) {
            return self.cartesian_motion_generator_joint_position_limits_violation;
          })
      .def_property_readonly(
          "cartesian_motion_generator_joint_velocity_limits_violation",
          [](const franka::Errors& self) {
            return self.cartesian_motion_generator_joint_velocity_limits_violation;
          })
      .def_property_readonly("cartesian_motion_generator_joint_velocity_discontinuity",
                             [](const franka::Errors& self) {
                               return self.cartesian_motion_generator_joint_velocity_discontinuity;
                             })
      .def_property_readonly(
          "cartesian_motion_generator_joint_acceleration_discontinuity",
          [](const franka::Errors& self) {
            return self.cartesian_motion_generator_joint_acceleration_discontinuity;
          })
      .def_property_readonly("cartesian_position_motion_generator_invalid_frame",
                             [](const franka::Errors& self) {
                               return self.cartesian_position_motion_generator_invalid_frame;
                             })
      .def_property_readonly("force_controller_desired_force_tolerance_violation",
                             [](const franka::Errors& self) {
                               return self.force_controller_desired_force_tolerance_violation;
                             })
      .def_property_readonly(
          "controller_torque_discontinuity",
          [](const franka::Errors& self) { return self.controller_torque_discontinuity; })
      .def_property_readonly(
          "start_elbow_sign_inconsistent",
          [](const franka::Errors& self) { return self.start_elbow_sign_inconsistent; })
      .def_property_readonly(
          "communication_constraints_violation",
          [](const franka::Errors& self) { return self.communication_constraints_violation; })
      .def_property_readonly("power_limit_violation",
                             [](const franka::Errors& self) { return self.power_limit_violation; })
      .def_property_readonly("joint_p2p_insufficient_torque_for_planning",
                             [](const franka::Errors& self) {
                               return self.joint_p2p_insufficient_torque_for_planning;
                             })
      .def_property_readonly("tau_j_range_violation",
                             [](const franka::Errors& self) { return self.tau_j_range_violation; })
      .def_property_readonly("instability_detected",
                             [](const franka::Errors& self) { return self.instability_detected; })
      .def_property_readonly(
          "joint_move_in_wrong_direction",
          [](const franka::Errors& self) { return self.joint_move_in_wrong_direction; })
      .def_property_readonly("cartesian_spline_motion_generator_violation",
                             [](const franka::Errors& self) {
                               return self.cartesian_spline_motion_generator_violation;
                             })
      .def_property_readonly(
          "joint_via_motion_generator_planning_joint_limit_violation",
          [](const franka::Errors& self) {
            return self.joint_via_motion_generator_planning_joint_limit_violation;
          })
      .def_property_readonly(
          "base_acceleration_initialization_timeout",
          [](const franka::Errors& self) { return self.base_acceleration_initialization_timeout; })
      .def_property_readonly("base_acceleration_invalid_reading", [](const franka::Errors& self) {
        return self.base_acceleration_invalid_reading;
      });

  // Bind franka::RobotState
  py::class_<franka::RobotState>(m, "RobotState", R"pbdoc(
        Current state of the Franka robot.

        Contains all robot state information including joint positions, velocities,
        torques, Cartesian poses, and error states. All arrays are NumPy arrays.
    )pbdoc")
      .def_readwrite("q", &franka::RobotState::q)
      .def_readwrite("q_d", &franka::RobotState::q_d)
      .def_readwrite("dq", &franka::RobotState::dq)
      .def_readwrite("dq_d", &franka::RobotState::dq_d)
      .def_readwrite("ddq_d", &franka::RobotState::ddq_d)
      .def_readwrite("tau_J", &franka::RobotState::tau_J)
      .def_readwrite("tau_J_d", &franka::RobotState::tau_J_d)
      .def_readwrite("dtau_J", &franka::RobotState::dtau_J)
      .def_readwrite("tau_ext_hat_filtered", &franka::RobotState::tau_ext_hat_filtered)
      .def_readwrite("theta", &franka::RobotState::theta)
      .def_readwrite("dtheta", &franka::RobotState::dtheta)
      .def_readwrite("O_T_EE", &franka::RobotState::O_T_EE)
      .def_readwrite("O_T_EE_d", &franka::RobotState::O_T_EE_d)
      .def_readwrite("O_T_EE_c", &franka::RobotState::O_T_EE_c)
      .def_readwrite("F_T_EE", &franka::RobotState::F_T_EE)
      .def_readwrite("F_T_NE", &franka::RobotState::F_T_NE)
      .def_readwrite("NE_T_EE", &franka::RobotState::NE_T_EE)
      .def_readwrite("EE_T_K", &franka::RobotState::EE_T_K)
      .def_readwrite("m_ee", &franka::RobotState::m_ee)
      .def_readwrite("I_ee", &franka::RobotState::I_ee)
      .def_readwrite("F_x_Cee", &franka::RobotState::F_x_Cee)
      .def_readwrite("m_load", &franka::RobotState::m_load)
      .def_readwrite("I_load", &franka::RobotState::I_load)
      .def_readwrite("F_x_Cload", &franka::RobotState::F_x_Cload)
      .def_readwrite("m_total", &franka::RobotState::m_total)
      .def_readwrite("I_total", &franka::RobotState::I_total)
      .def_readwrite("F_x_Ctotal", &franka::RobotState::F_x_Ctotal)
      .def_readwrite("elbow", &franka::RobotState::elbow)
      .def_readwrite("elbow_d", &franka::RobotState::elbow_d)
      .def_readwrite("elbow_c", &franka::RobotState::elbow_c)
      .def_readwrite("delbow_c", &franka::RobotState::delbow_c)
      .def_readwrite("ddelbow_c", &franka::RobotState::ddelbow_c)
      .def_readwrite("joint_contact", &franka::RobotState::joint_contact)
      .def_readwrite("cartesian_contact", &franka::RobotState::cartesian_contact)
      .def_readwrite("joint_collision", &franka::RobotState::joint_collision)
      .def_readwrite("cartesian_collision", &franka::RobotState::cartesian_collision)
      .def_readwrite("O_F_ext_hat_K", &franka::RobotState::O_F_ext_hat_K)
      .def_readwrite("K_F_ext_hat_K", &franka::RobotState::K_F_ext_hat_K)
      .def_readwrite("O_dP_EE_d", &franka::RobotState::O_dP_EE_d)
      .def_readwrite("O_dP_EE_c", &franka::RobotState::O_dP_EE_c)
      .def_readwrite("O_ddP_EE_c", &franka::RobotState::O_ddP_EE_c)
      .def_readwrite("O_ddP_O", &franka::RobotState::O_ddP_O)
      .def_readwrite("current_errors", &franka::RobotState::current_errors)
      .def_readwrite("last_motion_errors", &franka::RobotState::last_motion_errors)
      .def_readwrite("control_command_success_rate",
                     &franka::RobotState::control_command_success_rate)
      .def_readwrite("robot_mode", &franka::RobotState::robot_mode)
      .def_readwrite("time", &franka::RobotState::time);

  // Bind franka::Model
  py::class_<franka::Model>(m, "Model", R"pbdoc(
        Robot dynamics model for computing dynamics quantities.
    )pbdoc")
      .def(
          "coriolis",
          [](franka::Model& self, const franka::RobotState& robot_state) {
            return self.coriolis(robot_state);
          },
          R"pbdoc(
        Compute Coriolis force vector.

        @param robot_state Current robot state
        @return Coriolis force vector [Nm] as numpy array (7,)
    )pbdoc")
      .def(
          "gravity",
          [](franka::Model& self, const franka::RobotState& robot_state) {
            return self.gravity(robot_state);
          },
          R"pbdoc(
        Compute gravity force vector.

        @param robot_state Current robot state
        @return Gravity force vector [Nm] as numpy array (7,)
    )pbdoc")
      .def(
          "mass",
          [](franka::Model& self, const franka::RobotState& robot_state) {
            return self.mass(robot_state);
          },
          R"pbdoc(
        Compute mass matrix.

        @param robot_state Current robot state
        @return Mass matrix [kg*m²] as numpy array (7, 7)
    )pbdoc")
      .def(
          "zero_jacobian",
          [](franka::Model& self, const franka::RobotState& robot_state) {
            return self.zeroJacobian(franka::Frame::kEndEffector, robot_state);
          },
          R"pbdoc(
        Compute zero Jacobian for end effector frame.

        @param robot_state Current robot state
        @return Jacobian matrix as numpy array (6, 7)
    )pbdoc")
      .def(
          "zero_jacobian",
          [](franka::Model& self, const franka::Frame& frame,
             const franka::RobotState& robot_state) {
            return self.zeroJacobian(frame, robot_state);
          },
          R"pbdoc(
        Compute zero Jacobian for specified frame.

        @param frame Frame to compute Jacobian for
        @param robot_state Current robot state
        @return Jacobian matrix as numpy array (6, 7)
    )pbdoc");

  // Bind ActiveControlBase
  py::class_<franka::ActiveControlBase>(m, "ActiveControlBase", R"pbdoc(
        Base class for active robot control providing real-time command interface.
    )pbdoc")
      .def(
          "readOnce",
          [](franka::ActiveControlBase& self) {
            auto result = self.readOnce();
            return py::make_tuple(result.first, result.second);
          },
          R"pbdoc(
        Read robot state once.

        @return Tuple of (RobotState, Duration) containing the robot state and time since last read operation
    )pbdoc")
      .def("writeOnce",
           py::overload_cast<const franka::Torques&>(&franka::ActiveControlBase::writeOnce),
           R"pbdoc(
        Write torque command.

        @param command Torque command
    )pbdoc")
      .def("writeOnce",
           py::overload_cast<const franka::JointPositions&>(&franka::ActiveControlBase::writeOnce),
           R"pbdoc(
        Write joint position command.

        @param command Joint position command
    )pbdoc")
      .def("writeOnce",
           py::overload_cast<const franka::JointVelocities&>(&franka::ActiveControlBase::writeOnce),
           R"pbdoc(
        Write joint velocity command.

        @param command Joint velocity command
    )pbdoc")
      .def("writeOnce",
           py::overload_cast<const franka::CartesianPose&>(&franka::ActiveControlBase::writeOnce),
           R"pbdoc(
        Write Cartesian pose command.

        @param command Cartesian pose command
    )pbdoc")
      .def("writeOnce",
           py::overload_cast<const franka::CartesianVelocities&>(
               &franka::ActiveControlBase::writeOnce),
           R"pbdoc(
        Write Cartesian velocity command.

        @param command Cartesian velocity command
    )pbdoc");

  // Bind control types
  py::class_<franka::Torques>(m, "Torques", R"pbdoc(
        Torque control command.
    )pbdoc")
      .def(py::init<const std::array<double, 7>&>(), R"pbdoc(
        Create torque command.

        @param tau_J Joint torques [Nm] (7,)
    )pbdoc")
      .def_readwrite("tau_J", &franka::Torques::tau_J, "Joint torques [Nm] (7,)")
      .def_readwrite("motion_finished", &franka::Torques::motion_finished,
                     "Set to True to finish motion");

  py::class_<franka::JointPositions>(m, "JointPositions", R"pbdoc(
        Joint position control command.
    )pbdoc")
      .def(py::init<const std::array<double, 7>&>(), R"pbdoc(
        Create joint position command.

        @param q Joint positions [rad] (7,)
    )pbdoc")
      .def_readwrite("q", &franka::JointPositions::q, "Joint positions [rad] (7,)")
      .def_readwrite("motion_finished", &franka::JointPositions::motion_finished,
                     "Set to True to finish motion");

  py::class_<franka::JointVelocities>(m, "JointVelocities", R"pbdoc(
        Joint velocity control command.
    )pbdoc")
      .def(py::init<const std::array<double, 7>&>(), R"pbdoc(
        Create joint velocity command.

        @param dq Joint velocities [rad/s] (7,)
    )pbdoc")
      .def_readwrite("dq", &franka::JointVelocities::dq, "Joint velocities [rad/s] (7,)")
      .def_readwrite("motion_finished", &franka::JointVelocities::motion_finished,
                     "Set to True to finish motion");

  py::class_<franka::CartesianPose>(m, "CartesianPose", R"pbdoc(
        Cartesian pose control command.
    )pbdoc")
      .def(py::init<const std::array<double, 16>&>(), R"pbdoc(
        Create Cartesian pose command.

        @param O_T_EE Homogeneous transformation matrix (16,) in column-major order
    )pbdoc")
      .def_readwrite("O_T_EE", &franka::CartesianPose::O_T_EE,
                     "End effector pose (16,) column-major")
      .def_readwrite("motion_finished", &franka::CartesianPose::motion_finished,
                     "Set to True to finish motion");

  py::class_<franka::CartesianVelocities>(m, "CartesianVelocities", R"pbdoc(
        Cartesian velocity control command.
    )pbdoc")
      .def(py::init<const std::array<double, 6>&>(), R"pbdoc(
        Create Cartesian velocity command.

        @param O_dP_EE End effector twist [vx, vy, vz, wx, wy, wz] in [m/s, rad/s] (6,)
    )pbdoc")
      .def_readwrite("O_dP_EE", &franka::CartesianVelocities::O_dP_EE,
                     "End effector twist [m/s, rad/s] (6,)")
      .def_readwrite("motion_finished", &franka::CartesianVelocities::motion_finished,
                     "Set to True to finish motion");
  // Bind PyRobot
  py::class_<PyRobot>(m, "Robot", R"pbdoc(
        Main interface for controlling a Franka robot.

        Provides real-time control capabilities including torque, position,
        and velocity control modes.
    )pbdoc")
      .def(py::init<const std::string&, franka::RealtimeConfig>(), py::arg("robot_ip_address"),
           py::arg("realtime_config") = franka::RealtimeConfig::kEnforce, R"pbdoc(
        Connect to a Franka robot.

        @param robot_ip_address IP address or hostname of the robot
        @param realtime_config Real-time scheduling requirements (default: kEnforce)
    )pbdoc")
      .def("start_torque_control", &PyRobot::startTorqueControl, R"pbdoc(
        Start torque control mode.

        @return ActiveControlBase interface for sending torque commands
    )pbdoc")
      .def("start_joint_position_control", &PyRobot::startJointPositionControl, R"pbdoc(
        Start joint position control mode.

        @param control_type Controller mode (JointImpedance or CartesianImpedance)
        @return ActiveControlBase interface for sending position commands
    )pbdoc")
      .def("start_joint_velocity_control", &PyRobot::startJointVelocityControl, R"pbdoc(
        Start joint velocity control mode.

        @param control_type Controller mode (JointImpedance or CartesianImpedance)
        @return ActiveControlBase interface for sending velocity commands
    )pbdoc")
      .def("start_cartesian_pose_control", &PyRobot::startCartesianPoseControl, R"pbdoc(
        Start Cartesian pose control mode.

        @param control_type Controller mode (JointImpedance or CartesianImpedance)
        @return ActiveControlBase interface for sending Cartesian pose commands
    )pbdoc")
      .def("set_collision_behavior", &PyRobot::setCollisionBehavior, R"pbdoc(
        Configure collision detection thresholds.

        @param lower_torque_thresholds Lower torque thresholds [Nm] (7,)
        @param upper_torque_thresholds Upper torque thresholds [Nm] (7,)
        @param lower_force_thresholds Lower Cartesian force thresholds [N, Nm] (6,)
        @param upper_force_thresholds Upper Cartesian force thresholds [N, Nm] (6,)
    )pbdoc")
      .def("set_joint_impedance", &PyRobot::setJointImpedance, R"pbdoc(
        Set joint impedance for internal controller.

        @param K_theta Joint stiffness values [Nm/rad] (7,)
    )pbdoc")
      .def("set_cartesian_impedance", &PyRobot::setCartesianImpedance, R"pbdoc(
        Set Cartesian impedance for internal controller.

        @param K_x Cartesian stiffness values [N/m, Nm/rad] (6,)
    )pbdoc")
      .def("set_K", &PyRobot::setK, R"pbdoc(
        Set stiffness frame K in end effector frame.

        @param EE_T_K Homogeneous transformation matrix (16,) in column-major order
    )pbdoc")
      .def("set_EE", &PyRobot::setEE, R"pbdoc(
        Set end effector frame relative to nominal end effector frame.

        @param NE_T_EE Homogeneous transformation matrix (16,) in column-major order
    )pbdoc")
      .def("set_load", &PyRobot::setLoad, R"pbdoc(
        Set external load parameters.

        @param load_mass Mass of the external load [kg]
        @param F_x_Cload Center of mass of load in flange frame [m] (3,)
        @param load_inertia Inertia tensor of load [kg*m²] (9,) in column-major order
    )pbdoc")
      .def("automatic_error_recovery", &PyRobot::automaticErrorRecovery, R"pbdoc(
        Attempt automatic error recovery.
    )pbdoc")
      .def("read_once", &PyRobot::readOnce, R"pbdoc(
        Read current robot state once.

        @return Current robot state
    )pbdoc")
      .def("load_model", &PyRobot::loadModel, R"pbdoc(
        Load robot dynamics model.

        @return Model object for computing dynamics quantities
    )pbdoc")
      .def("stop", &PyRobot::stop, R"pbdoc(
        Stop currently running motion.
    )pbdoc");

  // Bind franka::GripperState
  py::class_<franka::GripperState>(m, "GripperState", R"pbdoc(
        Current state of the Franka Hand gripper.
    )pbdoc")
      .def_readwrite("width", &franka::GripperState::width, "Current gripper width [m]")
      .def_readwrite("max_width", &franka::GripperState::max_width, "Maximum gripper width [m]")
      .def_readwrite("is_grasped", &franka::GripperState::is_grasped, "True if object is grasped")
      .def_readwrite("temperature", &franka::GripperState::temperature, "Gripper temperature [°C]")
      .def_readwrite("time", &franka::GripperState::time, "Timestamp");

  // Bind PyGripper
  py::class_<PyGripper>(m, "Gripper", R"pbdoc(
        Interface for controlling a Franka Hand gripper.
    )pbdoc")
      .def(py::init<const std::string&>(), R"pbdoc(
        Connect to gripper.

        Establishes connection to the Franka Hand gripper at the specified address.

        @param franka_address IP address or hostname of the robot
        :raises NetworkException: if the connection cannot be established
    )pbdoc")
      .def("homing", &PyGripper::homing, R"pbdoc(
        Perform homing to find maximum width.

        Homing moves the gripper fingers to the maximum width to calibrate the position.
        This should be performed after powering on the gripper or when the gripper state is uncertain.

        @return True if successful
        :raises CommandException: if the command fails
        :raises NetworkException: if the connection is lost
    )pbdoc")
      .def("grasp", &PyGripper::grasp, py::arg("width"), py::arg("speed"), py::arg("force"),
           py::arg("epsilon_inner") = 0.005, py::arg("epsilon_outer") = 0.005, R"pbdoc(
        Grasp an object.

        The gripper closes at the specified speed and force until an object is detected
        or the target width is reached. The grasp is considered successful if the final
        width is within the specified tolerances.

        @param width Target grasp width [m]
        @param speed Closing speed [m/s]
        @param force Grasping force [N]
        @param epsilon_inner Inner tolerance for grasp check [m] (default: 0.005)
        @param epsilon_outer Outer tolerance for grasp check [m] (default: 0.005)
        @return True if grasp successful
        :raises CommandException: if the command fails
        :raises NetworkException: if the connection is lost
    )pbdoc")
      .def("read_once", &PyGripper::readOnce, R"pbdoc(
        Read current gripper state once.

        Queries the gripper for its current state including width, temperature,
        and grasp status.

        @return Current gripper state
        :raises NetworkException: if the connection is lost
    )pbdoc")
      .def("stop", &PyGripper::stop, R"pbdoc(
        Stop current gripper motion.

        Stops any ongoing gripper movement immediately.

        @return True if successful
        :raises CommandException: if the command fails
        :raises NetworkException: if the connection is lost
    )pbdoc")
      .def("move", &PyGripper::move, R"pbdoc(
        Move gripper fingers to a specific width.

        Moves the gripper to the specified width at the given speed. Use this for
        opening and closing the gripper without force control.

        @param width Target width [m]
        @param speed Movement speed [m/s]
        @return True if successful
        :raises CommandException: if the command fails
        :raises NetworkException: if the connection is lost
    )pbdoc")
      .def("server_version", &PyGripper::serverVersion, R"pbdoc(
        Get gripper server version.

        @return Server version information
    )pbdoc");
}

}  // namespace pylibfranka
