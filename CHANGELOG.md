# CHANGELOG

All notable changes to libfranka and pylibfranka will be documented in this file.

## [0.18.2]
Requires Franka Research 3 System Version >= 5.9.0

### libfranka - C++
#### Documentation
- Enhanced documentation with expanded installation methods, and reorganized getting started guide

#### Added
- upload debian packages automatically to releases

#### Fixed
- Fixed a compile issue with TinyXML2 dependency (see [github](https://github.com/frankarobotics/libfranka/issues/215))


### pylibfranka - Python
#### Added
- Added Joint and Cartesian Velocity controller examples.

## [0.18.1]
Requires Franka Research 3 System Version >= 5.9.0

### libfranka - C++
#### Changed
- The libfranka library is built with static links - removing the need for dynamic dependencies at runtime
- Fix the acceleration link addition bug and robot model link parsing [github](https://github.com/frankarobotics/libfranka/issues/213)
- Update the test urdf with the acceleration links and joint

### pylibfranka - Python
#### Added
- Added API documentation section to the official docs: [API References](https://frankarobotics.github.io/docs/libfranka/docs/api_references.html)

## [0.18.0]
Requires Franka Research 3 System Version >= 5.9.0

### libfranka - C++
#### Added
- Forward kinematics computation using Pinocchio library
- Jacobian computation using Pinocchio library
- Accelerometer data exposed in robot state
- Joint position-based velocity limits parsed from URDF to compute upper and lower velocity bounds
- Documentation under libfranka repository
- Feat: Async Position Control Handler added to libfranka to support asynchronous position control (**Experimental and subject to change**)

#### Changed
- Updated libfranka-common (common-libfranka) to use float-based RobotState struct
- Upgraded Pinocchio dependency to version 3.4.0
- Upgraded minimum Clang version from 6 to 14
- Removed dynamic library loading from Model constructor

#### Fixed
- Corrected elbow limit rating which was using incorrect values

#### Deprecated
- Marked `franka::computeUpperLimitsJointVelocity(const std::array<double, 7UL> &q)` and `franka::computeLowerLimitsJointVelocity(const std::array<double, 7UL> &q)`as deprecated. It will be removed in future versions. Use `Robot::getUpperJointVelocityLimits(const std::array<double, 7UL> &joint_positions)` and `Robot::getLowerJointVelocityLimits(const std::array<double, 7UL> &joint_positions)` instead.
- With system image >= 5.9.0, the joint position limits of the system with libfranka are increased to match the datasheet. The methods `franka::computeUpperLimitsJointVelocity(const std::array<double, 7UL> &q)` and `franka::computeLowerLimitsJointVelocity(const std::array<double, 7UL> &q)` now impose stricter constraints compared to the new parametrized limits.

### pylibfranka - Python
- Deployment of the pylibfranka to the github hosted web-page
- Pylibfranka and libfranka versions are synchronized


## [0.17.0] - 2025-10-15
### pylibfranka - Python
#### Added
- Cartesian pose control interface

## 0.16.1 - 2025-10-10

* Fix: fix cmake and package versioning

## 0.16.0 - 2025-09-15

* Feat: Added pylibfranka - a python binding for libfranka

## 0.15.3 - 2025-07-31

* Fix outdated Emika links

## 0.15.2 - 2025-07-24

* Optimize robot model computation

## 0.15.1 - 2025-06-23

* docs starts a new docs build

## 0.15.0 - 2025-01-16

Requires Franka Research 3 System Version >= 5.7.2

* Improved error minimization for joint/cartesian pose velocities and accelerations.
* Updated joint position and cartesian pose examples to start from the initial `q` or `O_T_EE`.
* Fixed joint position and cartesian pose low-pass filters to use the same command in the first package.
* Added a general logger, allowing users to implement custom loggers for receiving log messages.
* Fixed the bug that prevented exit in case of network connection interruption.

**Important**
* The flags `fci_joint_motion_generator_position_limits_violation_flag` and `fci_cartesian_motion_generator_joint_position_limits_violation_flag` are now always set to `false` and will be deprecated in future releases. They will be replaced by `joint_position_limits_violation`.

**BREAKING**
* The previous CSV robot state logger has been renamed to `robot_state_logger` and relocated.


## 0.14.1 - 2024-09-06

* Added doxygen generation

## 0.14.0 - 2024-03-26

Requires Franka Research 3 system version >= 5.7.0

* Compute dynamic robot parameters with Pinocchio library.

## 0.13.5 - 2024-09-06

* Updated doxygen generation

## 0.13.4 - 2024-09-05
Requires Franka Research 3 system version >= 5.5.0

* Added package.xml for ros humble release

## 0.13.3 - 2024-01-18

Requires Franka Research 3 system version >= 5.5.0

* Bump libfranka-common version compatible with 5.5.0
* Delete the temporary-workaround max-path-pose deviation. Fixed in the system image.

## 0.13.2 - 2023-12-04

* Hotfix: (temporary-workaround) for max-path-pose-deviation in ExternalMode for active control.


## 0.13.1 - 2023-11-30

* Hotfix: (temporary-workaround) for max-path-pose-deviation in ExternalMode for callback based control.

## 0.13.0 - 2023-11-16

* Add abstract ActiveControlBase class for the ActiveControllers to implement.
* **BREAKING** Fix function naming errors in robot class.
* **BREAKING** ActiveController initializer functions return ActiveControlBase

## 0.12.1 - 2023-09-20

* Fix install common/include typo

## 0.12.0 - 2023-09-14

Requires Franka Research 3 system version >= 5.2.0

* **BREAKING** ActiveControl serves now as base class for ActiveTorqueControl, that implements the torque control functionality
* ActiveMotionGenerator class introduced to support external control loops for JointPositions, JointVelocities, CartesianPose and CartesianVelocities

## 0.11.0 - 2023-08-16

Requires Franka Research 3 system version >= 5.2.0

* **BREAKING** Instead of using constants, velocity limits in rate-limiter are computed based on the current desired joint position.
* Allow external control loops by the introduction of the ActiveControl read-write-interface for torque-control

## 0.10.0 - 2022-09-02

Requires Franka Research 3 system version >= 5.2.0

* **BREAKING** Remove getVirtualWall and setFilters commands
* **BREAKING** Disable rate-limiter by default
* **BREAKING** Adapt rate limiting values in `rate_limiting.h` for FR3 robot

## 0.9.2 - 2022-08-31

Requires Panda system version >= 4.2.1

* Revert usage of `set_version_from_git` in CMakeLists.txt, using hardcoded values instead

## 0.9.1 - 2022-08-26

Requires Panda system version >= 4.2.1

* Use orthonormalized rotations in `cartesianLowpassFilter`, `limitRate` and `cartesian_impedance_control` example
* Support building libfranka from outside the library, so the debian package name can be set externally.
* Check if GitHub remote is always in sync
* Bug fixes in `rate_limiting_tests.cpp`
* Make column names in `logToCSV` unique, separated them by state & cmd
* Improve error messages including current `RobotMode` information

## 0.9.0 - 2022-03-25

Requires Panda system version >= 4.2.1

* **BREAKING** Add `O_ddP_O` base acceleration to robot state, hardcoded to `{0, 0, -9.81}`.
* **BREAKING** New `base_acceleration_initialization_timeout`, `base_acceleration_invalid_reading`
                `cartesian_spline_motion_generator_violation` and
                `joint_via_motion_generator_planning_joint_limit_violation` reflexes.
* Adjust network error messages. Distinguish between problems resulting from:
    - a wrong network configuration. A message is shown after a timeout of 60 seconds.
    - a missing FCI feature or a blocked port due to Single Point of Control. An immediate error
      response is shown.
* Changed signature of `franka::Model::gravity` to use `O_ddP_O` in the robot state.

## 0.8.0 - 2020-04-29

Requires Panda system version >= 4.0.0

* **BREAKING** Change behavior of `franka::Robot::setEE`. Previously, this method would set the
   flange-to-end-effector transformation `F_T_EE`. This has been split up into two transformations:
   `F_T_NE`, only settable in Desk, and `NE_T_EE`, which can be set in `libfranka` with `setEE` and
   defaults to the identity transformation.
* Add `F_T_NE` and `NE_T_EE` to `franka::RobotState`.
* Add support for the cobot pump with `franka::VacuumGripper`.
* CPack: Add conflict with `ros-melodic-libfranka`.
* Fix missing include for Windows (#55).

## 0.7.1 - 2019-07-08

Requires Panda system version >= 3.0.0

### Changed

* Fix compilation error on ARM.

## 0.7.0 - 2019-07-05

Requires Panda system version >= 3.0.0

### Added

* Add support for using `franka::Model` on Linux ARM and ARM64
   (requires Panda system version >= 3.1.0).
* Add Cartesian low-pass filter in `lowpass_filter.h`.
* Add `control_tools.h` with helper functions for writing control loops.

### Changed

* **BREAKING** Rename `franka::CartesianPose::hasValidElbow()`
   to `franka::CartesianPose::hasElbow()`.
* **BREAKING** Throw `std::invalid_argument` in `franka::Robot::control` if
   NaN or infinity values are returned from a control callback.
* **BREAKING** Throw `std::invalid_argument` in low-pass filter and rate limiting
   functions if invalid values are given as function parameters.
* **BREAKING** Do not throw exceptions in constructors of control types anymore.
* Take arguments by value in `franka::MotionFinished`.
* Bug fixes in `communication_test.cpp`.
* Export requirements for C++ features as CMake target compile features.

## 0.6.0 - 2019-02-06

Requires Panda system version >= 3.0.0

* Added support for Ubuntu 18.04 (Bionic).
* **EXPERIMENTAL** Added support for Windows.
* Added support for using `franka::Model` on Linux and Windows x86 and x64.
* Bugfix for aborting gripper commands with `franka::Gripper::stop()`.
* Improved documentation for `franka::Robot::setCollisionBehavior`.

## 0.5.0 - 2018-08-08

Requires Panda system version >= 1.3.0

### Motion and control interfaces

* **BREAKING** Added low-pass filter for all realtime interfaces with default cutoff frequency
   of 100 Hz
* **DEPRECATED** `franka::Robot::setFilters` in favor of `franka::lowpassFilter`
* Fixed description of log record entries

### Library

* Added `lowpass_filter.h` to public interface

### Examples

* Added `communication_test.cpp` to evaluate network performance.

## 0.4.0 - 2018-06-21

Requires Panda system version >= 1.3.0

### Motion and control interfaces

* **BREAKING** Added rate limiting of commanded values as an option in the control loop and as
    a helper function. Rate limiting is activated by default, this could change the behavior of
    existing programs. Only works when filters (`franka::Robot::setFilters`) are deactivated.
* Added `joint_move_in_wrong_direction` error to `franka::RobotState`
* Added first and second derivatives of last commanded signals to `franka::RobotState`

### Library

* Added `rate_limiting.h` to public interface
* Removed unnecessary CMake script from installation

## 0.3.0 - 2018-02-22

Requires Panda system version >= 1.2.0

### Motion and control interfaces

* Added optional `epsilon` parameters to `franka::Gripper::grasp`

### Examples

* Set default collision behavior, impedances, and filter frequencies
* Removed command line parameters to simplify usage
* Fixed locking bug in `joint_impedance_control`

### Library

* **BREAKING** Changed signatures and added overloads in `franka::Model`
* Added additional variables to installed CMake config
* Updated `SOVERSION` to include minor version number
* Added conflict with `ros-kinetic-libfranka` to Debian packaging
* Minor fixes and improvements for API documentation
* Fixed build errors on Clang 5.0
* Fixed test errors for Poco >= 1.8.0

## 0.2.0 - 2018-01-29

Requires Panda system version >= 1.1.0

### Motion and control interfaces

* Improved external torque tracking behavior.
* Fixed discontinuities in commanding orientation changes via the cartesian
    pose interface.
* Added `joint_p2p_insufficient_torque_for_planning`, `tau_j_range_violation`, and
    `instability_detected` errors to `franka::RobotState`
* Added `tau_J_d`, `m_ee`, `F_x_Cee`, `I_ee`, `m_total`, `F_x_Ctotal`, `I_total`,
    `theta` and `dtheta` to `franka::RobotState`
* Added `setFilter` command to `franka::Robot`
* Added support for commanding elbow positions for Cartesian motions.
* Added stiffness frame `K` to `franka::Model`
* **BREAKING** Replaced `p_min` and `p_max` of `franka::VirtualWallCuboid` with `object_world_size`

### Error handling

* **WARNING** Not all robot errors can be recovered using the guiding button
    on the robot anymore. To manually recover from such errors, please use the
    Franka DESK web interface.
* Added logs to `ControlException` and improved exception messages.
* Fail earlier (by throwing exception) if any of the commanded values are
    `NaN` or `infinity`.

### Examples

* Added saturation to joint velocity, joint position, and joint impedance
    examples.
* Changed examples to read initial states inside control loops (after
    controller switching).
* Examples first move to an initial joint position.
* Added new cartesian impedance and force control examples.
* Lowered grasping force in `grasp_object` example

### Library

* **BREAKING** New build-time dependency on Eigen3.
* Changed thread priority to the maximum allowed value.
* Prepare for the removal of the socket-init in the default constructor in
    POCO releases >= 1.8.0.
* Removed unnecessary public dependencies for libfranka.
* CI: Run linter on examples
* Docu: Use SVG instead of MathML for math rendering in API documentation to support Chrome.

## 0.1.0 - 2017-09-15

* Initial release
