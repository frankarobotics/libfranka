# pylibfranka Installation and Usage Guide

This document provides comprehensive instructions for installing and using pylibfranka, a Python binding for libfranka that enables control of Franka Robotics robots.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Installation from Source](#installation-from-source)
  - [Installation from Artifactory](#installation-from-artifactory)
- [Examples](#examples)
  - [Joint Position Example](#joint-position-example)
  - [Print Robot State Example](#print-robot-state-example)
  - [Joint Impedance Control Example](#joint-impedance-control-example)
  - [Gripper Control Example](#gripper-control-example)
- [Troubleshooting](#troubleshooting)

## Prerequisites

Before installing pylibfranka, ensure you have the following prerequisites:

- Python 3.8 or newer
- CMake 3.16 or newer
- C++ compiler with C++17 support
- Eigen3 development headers
- Poco development headers

### Installing Prerequisites on Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev libpoco-dev python3-dev
```
## Installing Pinocchio

```bash
 sudo apt install -qqy lsb-release curl
```

```bash
 sudo mkdir -p /etc/apt/keyrings
 curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
     | sudo tee /etc/apt/keyrings/robotpkg.asc
```

```bash
 echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
     | sudo tee /etc/apt/sources.list.d/robotpkg.list
```
```bash
 sudo apt update
 sudo apt install -qqy robotpkg-py3*-pinocchio
```

### Installation from Source

1. Clone the repository with the `--recursive` flag to also clone the libfranka submodule:

```bash
git clone --recursive git@github.com:frankaemika/pylibfranka.git
cd pylibfranka
```

2. Create and activate a virtual environment (optional but recommended):

```bash
python3 -m venv venv
source venv/bin/activate
```

3. Install the required build dependencies:

```bash
pip install -r requirements.txt
```

4. Install the package in development mode:

```bash
pip install .
```

5. Install and set up pre-commit hooks (recommended for development):

```bash
pip install pre-commit
pre-commit install
```

This will install pre-commit and set up the git hooks to automatically run checks before each commit. The hooks will help maintain code quality by running various checks like code formatting, linting, and other validations.

To manually run the pre-commit checks on all files:

```bash
pre-commit run --all-files
```

This will build the C++ extension and install the Python package.

## Examples

pylibfranka comes with three example scripts that demonstrate how to use the library to control a Franka robot.

### Joint Position Example

This example demonstrates how to use an external control loop with pylibfranka to move the robot joints.

To run the example:

```bash
python3 examples/joint_position_example.py --ip <robot_ip>
```

Where `<robot_ip>` is the IP address of your Franka robot. If not specified, it defaults to "localhost".

The active control example:
- Sets collision behavior parameters
- Starts joint position control with CartesianImpedance controller mode
- Moves the robot using an external control loop
- Performs a simple motion of selected joints

### Print Robot State Example

This example shows how to read and display the complete state of the robot.

To run the example:

```bash
python3 examples/print_robot_state.py --ip <robot_ip> [--rate <rate>] [--count <count>]
```

Where:
- `--ip` is the robot's IP address (defaults to "localhost")
- `--rate` is the frequency at which to print the state in Hz (defaults to 0.5)
- `--count` is the number of state readings to print (defaults to 1, use 0 for continuous)

The print robot state example:
- Connects to the robot
- Reads the complete robot state
- Prints detailed information about:
  - Joint positions, velocities, and torques
  - End effector pose and velocities
  - External forces and torques
  - Robot mode and error states
  - Mass and inertia properties

### Joint Impedance Control Example

This example demonstrates how to implement a joint impedance controller that renders a spring-damper system to move the robot through a sequence of target joint configurations.

To run the example:

```bash
python3 examples/joint_impedance_example.py --ip <robot_ip>
```

Where `--ip` is the robot's IP address (defaults to "localhost").

The joint impedance example:
- Implements a minimum jerk trajectory generator for smooth joint motion
- Uses a spring-damper system for compliant control
- Moves through a sequence of predefined joint configurations:
  - Home position (slightly bent arm)
  - Extended arm pointing forward
  - Arm pointing to the right
  - Arm pointing to the left
  - Return to home position
- Includes position holding with dwell time between movements
- Compensates for Coriolis effects

### Gripper Control Example

This example demonstrates how to control the Franka gripper, including homing, grasping, and reading gripper state.

To run the example:

```bash
python3 examples/move_gripper.py --robot_ip <robot_ip> [--width <width>] [--homing <0|1>] [--speed <speed>] [--force <force>]
```

Where:
- `--robot_ip` is the robot's IP address (required)
- `--width` is the object width to grasp in meters (defaults to 0.005)
- `--homing` enables/disables gripper homing (0 or 1, defaults to 1)
- `--speed` is the gripper speed (defaults to 0.1)
- `--force` is the gripper force in N (defaults to 60)

The gripper example:
- Connects to the gripper
- Performs homing to estimate maximum grasping width
- Reads and displays gripper state including:
  - Current width
  - Maximum width
  - Grasp status
  - Temperature
  - Timestamp
- Attempts to grasp an object with specified parameters
- Verifies successful grasping
- Releases the object

## Troubleshooting

More information about troubleshooting you can find in the FCI documentation https://frankaemika.github.io/docs/troubleshooting.html
