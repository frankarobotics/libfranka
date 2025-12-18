libfranka: C++ Library for Franka Robotics Research Robots
===========================================================

.. image:: https://codecov.io/gh/frankarobotics/libfranka/branch/master/graph/badge.svg
   :target: https://codecov.io/gh/frankarobotics/libfranka
   :alt: codecov

**libfranka** is a C++ library that provides low-level control of Franka Robotics research robots.
The `API References <https://frankarobotics.github.io/docs/libfranka/docs/api_references.html>`_ offers an overview of its capabilities,
while the `Franka Control Interface (FCI) documentation <https://frankarobotics.github.io/docs>`_ provides more information on setting up the robot and utilizing its features and functionalities.

To find the appropriate version to use, please refer to the `Robot System Version Compatibility <https://frankarobotics.github.io/docs/libfranka/docs/compatibility_with_images.html>`_.

Key Features
------------

- **Low-level control**: Access precise motion control for research robots.
- **Real-time communication**: Interact with the robot in real-time.
- **Multi-platform support**: Ubuntu 20.04, 22.04, and 24.04 LTS


1. System Requirements
~~~~~~~~~~~~~~~~~~~~~~

Before using **libfranka**, ensure your system meets the following requirements:

**Operating System**:
   - Ubuntu 20.04 LTS (Focal Fossa)
   - Ubuntu 22.04 LTS (Jammy Jellyfish)
   - Ubuntu 24.04 LTS (Noble Numbat)
   - `Linux with PREEMPT_RT patched kernel <https://frankarobotics.github.io/docs/libfranka/docs/real_time_kernel.html>`_ recommended for real-time control

**Build Tools** (for building from source):
   - GCC 9 or later
   - CMake 3.22 or later
   - Git

**Hardware**:
   - Franka Robotics robot with FCI feature installed
   - Network connection to robot (1000BASE-T Ethernet recommended)

.. _installation-debian-package:

2. Installation from Debian Package (Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The easiest way to install **libfranka** is by using the pre-built Debian packages
published on GitHub.

Supported Platforms
^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 28 18 18 26

   * - Ubuntu Version
     - Codename
     - Architecture
     - Status
   * - 20.04 LTS
     - focal
     - amd64
     - Supported
   * - 22.04 LTS
     - jammy
     - amd64
     - Supported
   * - 24.04 LTS
     - noble
     - amd64
     - Supported

Quick Install
^^^^^^^^^^^^^

Use the follwing scripts to set the desired **libfranka** version. The script automatically detects your
Ubuntu codename, downloads the matching Debian package, verifies its checksum,
and installs it.

.. code-block:: bash

   VERSION=0.19.0
   CODENAME=$(lsb_release -cs)

   # Download package
   wget https://github.com/frankarobotics/libfranka/releases/download/${VERSION}/libfranka_${VERSION}_${CODENAME}_amd64.deb

   # Download checksum
   wget https://github.com/frankarobotics/libfranka/releases/download/${VERSION}/libfranka_${VERSION}_${CODENAME}_amd64.deb.sha256

   # Verify package integrity
   sha256sum -c libfranka_${VERSION}_${CODENAME}_amd64.deb.sha256

   # Install package
   sudo dpkg -i libfranka_${VERSION}_${CODENAME}_amd64.deb

Example
^^^^^^^

The following example installs **libfranka 0.19.0** on **Ubuntu 22.04 (Jammy)**:

.. code-block:: bash

   VERSION=0.19.0

   wget https://github.com/frankarobotics/libfranka/releases/download/${VERSION}/libfranka_${VERSION}_jammy_amd64.deb
   wget https://github.com/frankarobotics/libfranka/releases/download/${VERSION}/libfranka_${VERSION}_jammy_amd64.deb.sha256
   sha256sum -c libfranka_${VERSION}_jammy_amd64.deb.sha256
   sudo dpkg -i libfranka_${VERSION}_jammy_amd64.deb

.. tip::

   All released versions, packages, and checksums are available on the
   `GitHub Releases page <https://github.com/frankarobotics/libfranka/releases>`_.


.. _building-with-docker:

3. Building with Docker (Development Environment)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Docker provides a consistent, isolated build environment. This method is ideal for:

- Development and testing
- Building for multiple Ubuntu versions
- Avoiding dependency conflicts on your host system

Prerequisites
^^^^^^^^^^^^^

- Docker installed on your system
- Visual Studio Code with Dev Containers extension (optional, for VS Code users)

Clone the Repository
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git
   cd libfranka
   git checkout 0.19.0  # or your desired version

Method A: Using Visual Studio Code (Recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. **Install Visual Studio Code**

   Download from https://code.visualstudio.com/

2. **Install the Dev Containers extension**

   - Open VS Code
   - Go to Extensions (Ctrl+Shift+X)
   - Search for "Dev Containers" and install it

3. **Configure Ubuntu version** (optional, defaults to 20.04)

   Edit ``devcontainer_distro`` file in the project root and specify the desired Ubuntu version:

   .. code-block:: bash

     22.04  # Ubuntu 22.04 (default)


4. **Open in container**

   - Open the project in VS Code: ``code .``
   - Press ``Ctrl+Shift+P``
   - Select "Dev Containers: Reopen in Container"
   - Wait for the container to build

5. **Build libfranka**

   Open a terminal in VS Code and run:

   .. code-block:: bash

      mkdir build && cd build
      cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
      cmake --build . -- -j$(nproc)

6. **Create Debian package** (optional)

   .. code-block:: bash

      cd build
      cpack -G DEB

   The package will be in the ``build/`` directory. To install on your host system:

   .. code-block:: bash

      # On host system (outside container)
      cd libfranka/build
      sudo dpkg -i libfranka*.deb

Method B: Using Docker Command Line
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. **Build the Docker image**

   .. code-block:: bash

      # For Ubuntu 20.04
      docker build --build-arg UBUNTU_VERSION=20.04 -t libfranka-build:20.04 .ci/

      # For Ubuntu 22.04
      docker build --build-arg UBUNTU_VERSION=22.04 -t libfranka-build:22.04 .ci/

      # For Ubuntu 24.04
      docker build --build-arg UBUNTU_VERSION=24.04 -t libfranka-build:24.04 .ci/

2. **Run the container and build**

   .. code-block:: bash

      docker run --rm -it -v $(pwd):/workspaces -w /workspaces libfranka-build:20.04

      # Inside container:
      mkdir build && cd build
      cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
      cmake --build . -- -j$(nproc)
      cpack -G DEB
      exit

3. **Install on host system**

   .. code-block:: bash

      cd build
      sudo dpkg -i libfranka*.deb

.. _building-from-source:

4. Building from Source (Advanced)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Prerequisites
^^^^^^^^^^^^^

**System packages:**

.. code-block:: bash

   sudo apt-get update
   sudo apt-get install -y \
       build-essential \
       cmake \
       git \
       wget \
       libeigen3-dev \
       libpoco-dev \
       libfmt-dev \
       pybind11-dev

**Ubuntu 20.04:** ensure CMake >= 3.22:

.. code-block:: bash

   wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg
   echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main" | sudo tee /etc/apt/sources.list.d/kitware.list
   sudo apt-get update
   sudo apt-get install -y cmake

Remove Existing Installations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt-get remove -y "*libfranka*"

Build Dependencies from Source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Follow these steps **in order**. All dependencies are built with static linking.

**1. Boost 1.77.0**

.. code-block:: bash

   git clone --depth 1 --recurse-submodules --shallow-submodules \
       --branch boost-1.77.0 https://github.com/boostorg/boost.git
   cd boost
   ./bootstrap.sh --prefix=/usr/local
   sudo ./b2 install -j$(nproc)
   cd .. && rm -rf boost

**2. TinyXML2**

.. code-block:: bash

   git clone --depth 1 --branch 10.0.0 https://github.com/leethomason/tinyxml2.git
   cd tinyxml2
   mkdir build && cd build

   cmake .. \
     -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
     -DBUILD_SHARED_LIBS=OFF \
     -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_INSTALL_PREFIX=/usr/local

   make -j$(nproc)
   sudo make install

   cd ../.. && rm -rf tinyxml2

**3. console_bridge**

.. code-block:: bash

   git clone --depth 1 --branch 1.0.2 https://github.com/ros/console_bridge.git
   cd console_bridge
   mkdir build && cd build

   cmake .. \
     -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
     -DBUILD_SHARED_LIBS=OFF \
     -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_INSTALL_PREFIX=/usr/local

   make -j$(nproc)
   sudo make install

   cd ../..
   rm -rf console_bridge


**4. urdfdom_headers**

.. code-block:: bash

   git clone --depth 1 --branch 1.0.5 https://github.com/ros/urdfdom_headers.git
   cd urdfdom_headers
   mkdir build && cd build

   cmake .. \
     -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_INSTALL_PREFIX=/usr/local

   make -j$(nproc)
   sudo make install

   cd ../..
   rm -rf urdfdom_headers


**5. urdfdom (with patch)**

.. code-block:: bash

   wget https://raw.githubusercontent.com/frankarobotics/libfranka/main/.ci/urdfdom.patch

   git clone --depth 1 --branch 4.0.0 https://github.com/ros/urdfdom.git
   cd urdfdom

   git apply ../urdfdom.patch

   mkdir build && cd build
   cmake .. \
     -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
     -DBUILD_SHARED_LIBS=OFF \
     -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_INSTALL_PREFIX=/usr/local

   make -j$(nproc)
   sudo make install
   cd ../..
   rm -rf urdfdom


**6. Assimp**

.. code-block:: bash

   git clone --depth 1 --recurse-submodules --shallow-submodules \
       --branch v5.4.3 https://github.com/assimp/assimp.git
   cd assimp && mkdir build && cd build
   cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
       -DBUILD_SHARED_LIBS=OFF -DASSIMP_BUILD_TESTS=OFF \
       -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
   make -j$(nproc) && sudo make install
   cd ../.. && rm -rf assimp

**7. Pinocchio (with patch)**

.. code-block:: bash

   wget https://raw.githubusercontent.com/frankarobotics/libfranka/main/.ci/pinocchio.patch
   git clone --depth 1 --recurse-submodules --shallow-submodules \
       --branch v3.4.0 https://github.com/stack-of-tasks/pinocchio.git
   cd pinocchio && git apply ../pinocchio.patch
   mkdir build && cd build
   cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
       -DBUILD_SHARED_LIBS=OFF -DBUILD_PYTHON_INTERFACE=OFF \
       -DBUILD_DOCUMENTATION=OFF -DBUILD_TESTING=OFF \
       -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
   make -j$(nproc) && sudo make install
   cd ../.. && rm -rf pinocchio

Build libfranka
^^^^^^^^^^^^^^^

.. code-block:: bash

   git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git
   cd libfranka
   git checkout 0.19.0
   git submodule update --init --recursive

   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_TESTS=OFF \
         -DCMAKE_INSTALL_PREFIX=/usr/local ..
   cmake --build . -- -j$(nproc)

Create Debian Package (Optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Inside the build folder, run:

.. code-block:: bash

   cpack -G DEB
   sudo dpkg -i libfranka*.deb

.. _verifying-installation:

5. Verifying Installation
~~~~~~~~~~~~~~~~~~~~~~~~~~

After installation (any method), verify that libfranka is properly installed:

**Check library file:**

.. code-block:: bash

   ls -l /usr/lib/libfranka.so

Expected output:

.. code-block:: text

   /usr/lib/libfranka.so -> libfranka.so.0.19.0

**Check header files:**

.. code-block:: bash

   ls /usr/include/franka/

Expected output:

.. code-block:: text

   active_control_base.h      active_torque_control.h  control_tools.h
   gripper_state.h            lowpass_filter.h         robot.h
   ...

**Check installed package version:**

.. code-block:: bash

   dpkg -l | grep libfranka

Expected output:

.. code-block:: text

   ii  libfranka  0.19.0  amd64  libfranka - Franka Robotics C++ library

.. _usage:

6. Usage
~~~~~~~~

After installation, configure your system for real-time control and run example programs:

System Setup
^^^^^^^^^^^^

1. **Network configuration**: Follow `Minimum System and Network Requirements <https://frankarobotics.github.io/docs/libfranka/docs/system_requirements.html>`_
2. **Real-time kernel**: See `Setting up the Real-Time Kernel <https://frankarobotics.github.io/docs/libfranka/docs/real_time_kernel.html>`_
3. **Getting started**: Read the `Getting Started Manual <https://frankarobotics.github.io/docs/libfranka/docs/getting_started.html>`_

Running Examples
^^^^^^^^^^^^^^^^

If you built from source, example programs are in the ``build/examples/`` directory:

.. code-block:: bash

   cd build/examples
   ./communication_test <robot-ip>

Replace ``<robot-ip>`` with your robot's IP address (e.g., ``192.168.1.1``).

For more examples, see the `Usage Examples documentation <https://frankarobotics.github.io/docs/libfranka/docs/usage_examples.html>`_.

.. _pylibfranka:

7. Pylibfranka (Python Bindings)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Pylibfranka** provides Python bindings for libfranka, allowing robot control with Python.

Installation
^^^^^^^^^^^^

Pylibfranka is included in the libfranka Debian packages. For manual installation:

.. code-block:: bash

   # Build with Python bindings enabled
   cd libfranka/build
   cmake -DGENERATE_PYLIBFRANKA=ON ..
   cmake --build . -- -j$(nproc)
   sudo cmake --install .

Documentation
^^^^^^^^^^^^^

- `Pylibfranka README <pylibfranka/README.md>`_
- `API Documentation <https://frankarobotics.github.io/libfranka/pylibfranka/latest>`_

.. _development-information:

8. Development Information
~~~~~~~~~~~~~~~~~~~~~~~~~~

Contributing to libfranka
^^^^^^^^^^^^^^^^^^^^^^^^^^

If you're contributing to libfranka development:

**Install pre-commit hooks:**

.. code-block:: bash

   pip install pre-commit
   pre-commit install

This automatically runs code formatting and linting checks before each commit.

**Run checks manually:**

.. code-block:: bash

   pre-commit run --all-files

Build Options
^^^^^^^^^^^^^

Customize your build with CMake options:

.. list-table::
   :header-rows: 1
   :widths: 35 50 15

   * - Option
     - Description
     - Default
   * - ``CMAKE_BUILD_TYPE``
     - Build type (Release/Debug)
     - Release
   * - ``BUILD_TESTS``
     - Build unit tests
     - ON
   * - ``BUILD_EXAMPLES``
     - Build example programs
     - ON
   * - ``GENERATE_PYLIBFRANKA``
     - Build Python bindings
     - OFF
   * - ``CMAKE_INSTALL_PREFIX``
     - Installation directory
     - /usr/local

Example:

.. code-block:: bash

   cmake -DCMAKE_BUILD_TYPE=Debug \
         -DBUILD_TESTS=ON \
         -DGENERATE_PYLIBFRANKA=ON \
         ..

Troubleshooting
~~~~~~~~~~~~~~~

**Network connection issues**

See the `Troubleshooting Network <https://frankarobotics.github.io/docs/libfranka/docs/troubleshooting.html>`_ guide.

License
-------

``libfranka`` is licensed under the `Apache 2.0 license <https://www.apache.org/licenses/LICENSE-2.0.html>`_.
