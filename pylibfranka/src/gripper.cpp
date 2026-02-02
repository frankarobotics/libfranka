// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka/gripper.h>
#include <pylibfranka/gripper.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace pylibfranka {

void bind_gripper(py::module& m) {
  // Bind franka::GripperState
  py::class_<franka::GripperState>(m, "GripperState", R"pbdoc(
        Current state of the Franka Hand gripper.
    )pbdoc")
      .def_readwrite("width", &franka::GripperState::width, "Current gripper width [m]")
      .def_readwrite("max_width", &franka::GripperState::max_width, "Maximum gripper width [m]")
      .def_readwrite("is_grasped", &franka::GripperState::is_grasped, "True if object is grasped")
      .def_readwrite("temperature", &franka::GripperState::temperature, "Gripper temperature [°C]")
      .def_readwrite("time", &franka::GripperState::time, "Timestamp");

  // Bind franka::Gripper
  py::class_<franka::Gripper>(m, "Gripper", R"pbdoc(
        Interface for controlling a Franka Hand gripper.
    )pbdoc")
      .def(py::init<const std::string&>(), R"pbdoc(
        Connect to gripper.

        Establishes connection to the Franka Hand gripper at the specified address.

        @param franka_address IP address or hostname of the robot
        :raises NetworkException: if the connection cannot be established
    )pbdoc")
      .def("homing", &franka::Gripper::homing, R"pbdoc(
        Perform homing to find maximum width.

        Homing moves the gripper fingers to the maximum width to calibrate the position.
        This should be performed after powering on the gripper or when the gripper state is uncertain.

        @return True if successful
        :raises CommandException: if the command fails
        :raises NetworkException: if the connection is lost
    )pbdoc")
      .def("grasp", &franka::Gripper::grasp, py::arg("width"), py::arg("speed"), py::arg("force"),
           py::arg("epsilon_inner") = 0.005, py::arg("epsilon_outer") = 0.005, R"pbdoc(
        Grasp an object.

        The gripper closes at the specified speed and force until an object is detected
        or the target width is reached. The grasp is considered successfulsetCurrentThreadToHighestSchedulerPriority if the final
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
      .def("read_once", &franka::Gripper::readOnce, R"pbdoc(
        Read current gripper state once.

        Queries the gripper for its current state including width, temperature,
        and grasp status.

        @return Current gripper state
        :raises NetworkException: if the connection is lost
    )pbdoc")
      .def("stop", &franka::Gripper::stop, R"pbdoc(
        Stop current gripper motion.

        Stops any ongoing gripper movement immediately.

        @return True if successful
        :raises CommandException: if the command fails
        :raises NetworkException: if the connection is lost
    )pbdoc")
      .def("move", &franka::Gripper::move, R"pbdoc(
        Move gripper fingers to a specific width.

        Moves the gripper to the specified width at the given speed. Use this for
        opening and closing the gripper without force control.

        @param width Target width [m]
        @param speed Movement speed [m/s]
        @return True if successful
        :raises CommandException: if the command fails
        :raises NetworkException: if the connection is lost
    )pbdoc")
      .def("server_version", &franka::Gripper::serverVersion, R"pbdoc(
        Get gripper server version.

        @return Server version information
    )pbdoc");
}

}  // namespace pylibfranka