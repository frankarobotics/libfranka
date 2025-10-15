# pylibfranka Installation and Usage Guide

This document provides comprehensive instructions for installing and using pylibfranka, a Python binding for libfranka that enables control of Franka Robotics robots.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Installation from Source](#installation-from-source)
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

**Disclaimer: If you are using the provided devcontainer, you can skip the prerequisites installation as they are already included in the container.**

### Installing Prerequisites on Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev libpoco-dev python3-dev
```

#### Installing pylibfranka via PIP

From the root folder, you can install `pylibfranka` (therefore, NOT in the build folder) using pip:

```bash
pip install .
```

or

```bash
pip3 install .
```

This will install pylibfranka in your current Python environment.

## Examples

pylibfranka comes with three example scripts that demonstrate how to use the library to control a Franka robot.

### Joint Position Example

This example demonstrates how to use an external control loop with pylibfranka to move the robot joints.

To run the example:

```bash
cd examples
python3 joint_position_example.py --ip <robot_ip>
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
cd examples
python3 print_robot_state.py --ip <robot_ip> [--rate <rate>] [--count <count>]
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
cd examples
python3 joint_impedance_example.py --ip <robot_ip>
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

### And more control examples

There are more control examples to discover. All of them can be executed in a similar way:

```bash
cd examples
python3 other_example.py --ip <robot_ip>
```

### Gripper Control Example

This example demonstrates how to control the Franka gripper, including homing, grasping, and reading gripper state.

To run the example:

```bash
cd examples
python3 move_gripper.py --robot_ip <robot_ip> [--width <width>] [--homing <0|1>] [--speed <speed>] [--force <force>]
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
