# libfranka: C++ Library for Franka Robotics Research Robots

[![codecov][codecov-status]][codecov]

**libfranka** is a C++ library that provides low-level control of Franka Robotics research robots. The [generated API documentation][api-docs] offers an overview of its capabilities, while the [Franka Control Interface (FCI) documentation][fci-docs] provides more information on setting up the robot and utilizing its features and functionalities.

To find the appropriate version to use, please refer to the [Compatibility Matrix][compatibility-matrix].

## Key Features

- **Low-level control**: Access precise motion control for research robots.
- **Real-time communication**: Interact with the robot in real-time.

# Getting Started

There are three main ways to set up and install **libfranka**: using the provided `libfranka*.deb` debian package, using the devcontainer setup or manually installing the library from source.

## 1 Debian Package Installation

Each release will provide a pre-built debian package that can be installed directly on your system:
```bash
sudo dpkg -i libfranka_<version>_<architecture>.deb
```

This is the recommended way to install **libfranka** if you do not need to modify the source code.

The artifacts can be found on [github](https://github.com/frankarobotics/libfranka).

## 2 Devcontainer Setup

For an easy start, we recommend using the devcontainer setup. This setup includes all necessary dependencies and configurations to get you up and running quickly. If you start Visual Code, it will automatically prompt you to open the folder in a container.

To build and install **libfranka** using the devcontainer, follow section [Building and Installation from Source](#4-building-and-installation-from-source) after opening the devcontainer.

## 3 Manual Installation

The manual installation is more complex and should only be used if you have specific requirements that the devcontainer setup does not meet.

### 3.1 System Requirements

Before using **libfranka**, ensure your system meets the following requirements:

- **Operating System**: [Linux with PREEMPT_RT patched kernel][real-time-kernel]  (Ubuntu 16.04 or later, Ubuntu 22.04 recommended)
- **Compiler**: GCC 7 or later
- **CMake**: Version 3.10 or later
- **Robot**: Franka Robotics robot with FCI feature installed

### 3.2 Installing dependencies

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev
```

To use libfranka version `0.14.0` or later, you will need to install [pinocchio][stack-of-tasks] and some more dependencies:

```bash
sudo apt-get install -y lsb-release curl
sudo mkdir -p /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
```

```bash
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
```

```bash
sudo apt-get update
sudo apt-get install -y robotpkg-pinocchio
```

## 4 Building and Installation from Source

Before building and installing from source, please uninstall existing installations of libfranka to avoid conflicts:

```bash
sudo apt-get remove "*libfranka*"
```

#### Clone the Repository

You can clone the repository and choose the version you need by selecting a specific tag:

```bash
git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git
cd libfranka
```

List available tags

```bash
git tag -l
```

Checkout a specific tag (e.g., 0.15.0)

```bash
git checkout 0.15.0
```

Update submodules

```bash
git submodule update
```

Create a build directory and navigate to it

```bash
mkdir build
cd build
```

Configure the project and build

```bash
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
make
```

#### Installing libfranka as a Debian Package (Optional but recommended)

Building a Debian package is optional but recommended for easier installation and management. In the build folder, execute:

```bash
cpack -G DEB
```

This command creates a Debian package named libfranka-<version>-<architecture>.deb. You can then install it with:

```bash
sudo dpkg -i libfranka*.deb
```

Installing via a Debian package simplifies the process compared to building from source every time. Additionally the package integrates better with system tools and package managers, which can help manage updates and dependencies more effectively.

# Usage

After installation, check the [Minimum System and Network Requirements][requirements] for network settings, the [Operating System and PC Configuration][real-time-kernel] for system setup, and the [Getting Started Manual][getting-started] for initial steps. Once configured, you can control the robot using the example applications provided in the examples folder.

To run a sample program, navigate to the build folder and execute the following command:

```bash
./examples/communication_test <robot-ip>
```

# Pylibfranka

Pylibfranka is a Python binding for libfranka, allowing you to control Franka robots using Python. It is included in the libfranka repository and can be built alongside libfranka. For more details, see `pylibfranka` and its [README](pylibfranka/README.md). The [generated API documentation][pylibfranka-api-docs] offers an overview of its capabilities.

# Development Information

If you actively contribute to this repository, you should install and set up pre-commit hooks:

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

# License

`libfranka` is licensed under the [Apache 2.0 license][apache-2.0].

[stack-of-tasks]: https://stack-of-tasks.github.io/pinocchio/download.html
[real-time-kernel]: https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel
[requirements]: https://frankarobotics.github.io/docs/requirements.html
[getting-started]: https://frankarobotics.github.io/docs/getting_started.html#
[compatibility-matrix]: https://frankarobotics.github.io/docs/compatibility.html
[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[api-docs]: https://frankarobotics.github.io/libfranka
[pylibfranka-api-docs]: https://frankarobotics.github.io/libfranka/pylibfranka/latest
[fci-docs]: https://frankarobotics.github.io/docs
[codecov-status]: https://codecov.io/gh/frankarobotics/libfranka/branch/master/graph/badge.svg
[codecov]: https://codecov.io/gh/frankarobotics/libfranka
