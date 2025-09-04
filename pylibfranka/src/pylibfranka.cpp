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

std::unique_ptr<franka::ActiveControlBase> PyRobot::startTorqueControl() {
  return robot_->startTorqueControl();
}

std::unique_ptr<franka::ActiveControlBase> PyRobot::startJointPositionControl(
    const franka::ControllerMode& control_type) {
  research_interface::robot::Move::ControllerMode mode;
  if (control_type == franka::ControllerMode::kJointImpedance) {
    mode = research_interface::robot::Move::ControllerMode::kJointImpedance;
  } else {
    mode = research_interface::robot::Move::ControllerMode::kCartesianImpedance;
  }
  return robot_->startJointPositionControl(mode);
}

std::unique_ptr<franka::ActiveControlBase> PyRobot::startJointVelocityControl(
    const franka::ControllerMode& control_type) {
  research_interface::robot::Move::ControllerMode mode;
  if (control_type == franka::ControllerMode::kJointImpedance) {
    mode = research_interface::robot::Move::ControllerMode::kJointImpedance;
  } else {
    mode = research_interface::robot::Move::ControllerMode::kCartesianImpedance;
  }
  return robot_->startJointVelocityControl(mode);
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
  // Bind exceptions
  py::register_exception<franka::Exception>(m, "FrankaException");
  py::register_exception<franka::CommandException>(m, "CommandException", PyExc_RuntimeError);
  py::register_exception<franka::NetworkException>(m, "NetworkException", PyExc_RuntimeError);
  py::register_exception<franka::ControlException>(m, "ControlException", PyExc_RuntimeError);
  py::register_exception<franka::InvalidOperationException>(m, "InvalidOperationException",
                                                            PyExc_RuntimeError);
  py::register_exception<franka::RealtimeException>(m, "RealtimeException", PyExc_RuntimeError);

  // Bind Duration
  py::class_<franka::Duration>(m, "Duration").def("to_sec", &franka::Duration::toSec);

  // Bind enums
  py::enum_<franka::ControllerMode>(m, "ControllerMode")
      .value("JointImpedance", franka::ControllerMode::kJointImpedance)
      .value("CartesianImpedance", franka::ControllerMode::kCartesianImpedance);

  // Bind RealtimeConfig enum
  py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig")
      .value("kEnforce", franka::RealtimeConfig::kEnforce)
      .value("kIgnore", franka::RealtimeConfig::kIgnore);

  // Bind RobotMode enum
  py::enum_<franka::RobotMode>(m, "RobotMode")
      .value("Other", franka::RobotMode::kOther)
      .value("Idle", franka::RobotMode::kIdle)
      .value("Move", franka::RobotMode::kMove)
      .value("Guiding", franka::RobotMode::kGuiding)
      .value("Reflex", franka::RobotMode::kReflex)
      .value("UserStopped", franka::RobotMode::kUserStopped)
      .value("AutomaticErrorRecovery", franka::RobotMode::kAutomaticErrorRecovery);

  // Bind Errors struct
  py::class_<franka::Errors>(m, "Errors")
      .def(py::init<>())
      .def("__bool__", &franka::Errors::operator bool)
      .def("__str__", &franka::Errors::operator std::string)
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
  py::class_<franka::RobotState>(m, "RobotState")
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
  py::class_<franka::Model>(m, "Model")
      .def("coriolis",
           [](franka::Model& self, const franka::RobotState& robot_state) {
             return self.coriolis(robot_state);
           })
      .def("gravity",
           [](franka::Model& self, const franka::RobotState& robot_state) {
             return self.gravity(robot_state);
           })
      .def("mass", [](franka::Model& self,
                      const franka::RobotState& robot_state) { return self.mass(robot_state); })
      .def("zero_jacobian",
           [](franka::Model& self, const franka::RobotState& robot_state) {
             return self.zeroJacobian(franka::Frame::kEndEffector, robot_state);
           })
      .def("zero_jacobian", [](franka::Model& self, const franka::Frame& frame,
                               const franka::RobotState& robot_state) {
        return self.zeroJacobian(frame, robot_state);
      });

  // Bind ActiveControlBase
  py::class_<franka::ActiveControlBase>(m, "ActiveControlBase")
      .def("readOnce",
           [](franka::ActiveControlBase& self) {
             auto result = self.readOnce();
             return py::make_tuple(result.first, result.second);
           })
      .def("writeOnce",
           py::overload_cast<const franka::Torques&>(&franka::ActiveControlBase::writeOnce))
      .def("writeOnce",
           py::overload_cast<const franka::JointPositions&>(&franka::ActiveControlBase::writeOnce))
      .def("writeOnce",
           py::overload_cast<const franka::JointVelocities&>(&franka::ActiveControlBase::writeOnce))
      .def("writeOnce",
           py::overload_cast<const franka::CartesianPose&>(&franka::ActiveControlBase::writeOnce))
      .def("writeOnce", py::overload_cast<const franka::CartesianVelocities&>(
                            &franka::ActiveControlBase::writeOnce));

  // Bind control types
  py::class_<franka::Torques>(m, "Torques")
      .def(py::init<const std::array<double, 7>&>())
      .def_readwrite("tau_J", &franka::Torques::tau_J)
      .def_readwrite("motion_finished", &franka::Torques::motion_finished);

  py::class_<franka::JointPositions>(m, "JointPositions")
      .def(py::init<const std::array<double, 7>&>())
      .def_readwrite("q", &franka::JointPositions::q)
      .def_readwrite("motion_finished", &franka::JointPositions::motion_finished);

  py::class_<franka::JointVelocities>(m, "JointVelocities")
      .def(py::init<const std::array<double, 7>&>())
      .def_readwrite("dq", &franka::JointVelocities::dq)
      .def_readwrite("motion_finished", &franka::JointVelocities::motion_finished);

  py::class_<franka::CartesianPose>(m, "CartesianPose")
      .def(py::init<const std::array<double, 16>&>())
      .def_readwrite("O_T_EE", &franka::CartesianPose::O_T_EE)
      .def_readwrite("motion_finished", &franka::CartesianPose::motion_finished);

  py::class_<franka::CartesianVelocities>(m, "CartesianVelocities")
      .def(py::init<const std::array<double, 6>&>())
      .def_readwrite("O_dP_EE", &franka::CartesianVelocities::O_dP_EE)
      .def_readwrite("motion_finished", &franka::CartesianVelocities::motion_finished);
  // Bind PyRobot
  py::class_<PyRobot>(m, "Robot")
      .def(py::init<const std::string&, franka::RealtimeConfig>(), py::arg("robot_ip_address"),
           py::arg("realtime_config") = franka::RealtimeConfig::kEnforce)
      .def("start_torque_control", &PyRobot::startTorqueControl)
      .def("start_joint_position_control", &PyRobot::startJointPositionControl)
      .def("start_joint_velocity_control", &PyRobot::startJointVelocityControl)
      .def("set_collision_behavior", &PyRobot::setCollisionBehavior)
      .def("set_joint_impedance", &PyRobot::setJointImpedance)
      .def("set_cartesian_impedance", &PyRobot::setCartesianImpedance)
      .def("set_K", &PyRobot::setK)
      .def("set_EE", &PyRobot::setEE)
      .def("set_load", &PyRobot::setLoad)
      .def("automatic_error_recovery", &PyRobot::automaticErrorRecovery)
      .def("read_once", &PyRobot::readOnce)
      .def("load_model", &PyRobot::loadModel)
      .def("stop", &PyRobot::stop);

  // Bind franka::GripperState
  py::class_<franka::GripperState>(m, "GripperState")
      .def_readwrite("width", &franka::GripperState::width)
      .def_readwrite("max_width", &franka::GripperState::max_width)
      .def_readwrite("is_grasped", &franka::GripperState::is_grasped)
      .def_readwrite("temperature", &franka::GripperState::temperature)
      .def_readwrite("time", &franka::GripperState::time);

  // Bind PyGripper
  py::class_<PyGripper>(m, "Gripper")
      .def(py::init<const std::string&>())
      .def("homing", &PyGripper::homing)
      .def("grasp", &PyGripper::grasp, py::arg("width"), py::arg("speed"), py::arg("force"),
           py::arg("epsilon_inner") = 0.005,  // Default inner tolerance
           py::arg("epsilon_outer") = 0.005)  // Default outer tolerance
      .def("read_once", &PyGripper::readOnce)
      .def("stop", &PyGripper::stop)
      .def("move", &PyGripper::move)
      .def("server_version", &PyGripper::serverVersion);

  // Add docstring
  m.doc() = "Python bindings for libfranka control";
}

}  // namespace pylibfranka
