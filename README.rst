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

Getting Started
---------------

.. _system-requirements:

1. System Requirements
~~~~~~~~~~~~~~~~~~~~~~

Before using **libfranka**, ensure your system meets the following requirements:

- **Operating System**: `Linux with PREEMPT_RT patched kernel <https://frankarobotics.github.io/docs/libfranka/docs/real_time_kernel.html>`_  (Ubuntu 16.04 or later, Ubuntu 22.04 recommended)
- **Compiler**: GCC 7 or later
- **CMake**: Version 3.10 or later
- **Robot**: Franka Robotics robot with FCI feature installed

.. _installing-dependencies:

2. Installing dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   sudo apt-get update
   sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev

3. Install from Debian Package - Generic Pattern
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Check your architecture:**

.. code-block:: bash

   dpkg --print-architecture

**Download and install:**

.. code-block:: bash

   wget https://github.com/frankarobotics/libfranka/releases/download/<version>/libfranka_<version>_<architecture>.deb
   sudo dpkg -i libfranka_<version>_<architecture>.deb

Replace ``<version>`` with the desired release version (e.g., ``0.18.1``) and ``<architecture>`` with your system architecture (e.g., ``amd64`` or ``arm64``).

**Example for version 0.18.1 on amd64:**

.. code-block:: bash

   wget https://github.com/frankarobotics/libfranka/releases/download/0.18.1/libfranka_0.18.1_amd64.deb
   sudo dpkg -i libfranka_0.18.1_amd64.deb

.. _building-in-docker:
4. Building libfranka Inside Docker
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you prefer to build **libfranka** inside a Docker container, you can use the provided Docker setup. This ensures a consistent build environment and avoids dependency conflicts on your host system.

Docker creates a self-contained environment, which is helpful if:

- Your system doesn't meet the requirements
- You want to avoid installing dependencies on your main system
- You prefer a clean, reproducible setup

If you haven't already, clone the **libfranka** repository:

   .. code-block:: bash

      git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git
      cd libfranka
      git checkout <desired-tag-or-branch>

Using Docker command line
^^^^^^^^^^^^^^^^^^^^^^^^^

1. **Build the Docker image**:

   .. code-block:: bash

      cd .ci
      docker build -t libfranka:latest -f Dockerfile.focal .
      cd ..

2. **Run the Docker container**:

   .. code-block:: bash

      docker run --rm -it -v $(pwd):/workspace libfranka:latest

   If you already have a build folder, you must remove it first to avoid issues:

   .. code-block:: bash

      rm -rf /workspace/build
      mkdir -p /workspace/build
      cd /workspace/build
      cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF /workspace
      make

   To generate a Debian package:

   .. code-block:: bash

      sudo cpack -G DEB

   Exit the Docker container by typing ``exit`` in the terminal.

3. **Install libfranka on your host system**:

   Inside the libfranka build folder ``cd build`` on your host system, run:

   .. code-block:: bash

      sudo dpkg -i libfranka*.deb


Using Visual Studio Code
^^^^^^^^^^^^^^^^^^^^^^^^

You can also build **libfranka** inside Docker using **VS Code** with the **Dev Containers** extension. This provides an integrated development environment inside the container.

1. **Install Visual Studio Code**:

   - Download and install **Visual Studio Code** from the official website: https://code.visualstudio.com/.
   - Follow the installation instructions for your operating system.

2. **Open the Project in VS Code**:

   Inside the libfranka folder, open a new terminal and run:

   .. code-block:: bash

      code .

   This will open the project in VS Code.

3. **Install the Dev Containers Extension**:

   Install the "Dev Containers" extension in VS Code from the Extensions Marketplace.

4. **Open the Project in a Dev Container**:

   - Open the **Command Palette** (``Ctrl+Shift+P``).
   - Select **Dev Containers: Reopen in Container**.
   - VS Code will build the Docker image and start a container based on the provided ``.ci/Dockerfile``.

5. **Build libfranka**:

   - Open a terminal in VS Code.
   - Run the following commands:

   .. code-block:: bash

      mkdir build && cd build
      cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
      make

6. **Install libfranka**:

   If you want to install **libfranka** inside the container, you can run:

   .. code-block:: bash

      sudo make install

   If you want to install **libfranka** on your host system, you can run:

   .. code-block:: bash

      sudo cpack -G DEB

   Then in a new terminal on your host system, navigate to the libfranka ``build`` folder and run:

   .. code-block:: bash

      sudo dpkg -i libfranka*.deb

.. _verifying-installation:
Verify the installation on your local system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To verify its installation, you can run:

.. code-block:: bash

   ls /usr/lib/libfranka.so

Expected output:

.. code-block:: text

   /usr/lib/libfranka.so

Check the installed headers:

.. code-block:: bash

   ls /usr/include/franka/

Expected output:

.. code-block:: text

   active_control_base.h      active_torque_control.h  control_tools.h      errors.h
   gripper_state.h            lowpass_filter.h         robot.h              robot_state.h
   active_control.h           async_control            control_types.h      exception.h
   joint_velocity_limits.h    model.h                  robot_model_base.h   vacuum_gripper.h
   active_motion_generator.h  commands                 duration.h           gripper.h
   logging                    rate_limiting.h          robot_model.h        vacuum_gripper_state.h

You can check the version of the installed library:

.. code-block:: bash

   dpkg -l | grep libfranka

Expected output:

.. code-block:: text

   ii  libfranka  0.18.1-9-g722bf63  amd64  libfranka built using CMake


.. _building-from-source:

5. Building and Installation from Source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before building and installing from source, please uninstall existing installations of libfranka to avoid conflicts:

.. code-block:: bash

   sudo apt-get remove "*libfranka*"

Clone the Repository
^^^^^^^^^^^^^^^^^^^^

You can clone the repository and choose the version you need by selecting a specific tag:

.. code-block:: bash

   git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git
   cd libfranka

List available tags

.. code-block:: bash

   git tag -l

Checkout a specific tag (e.g., 0.15.0)

.. code-block:: bash

   git checkout 0.15.0

Update submodules

.. code-block:: bash

   git submodule update

Create a build directory and navigate to it

.. code-block:: bash

   mkdir build
   cd build

Configure the project and build

.. code-block:: bash

   cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
   make

.. _installing-debian-package:

Installing libfranka as a Debian Package (Optional but recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Building a Debian package is optional but recommended for easier installation and management. In the build folder, execute:

.. code-block:: bash

   cpack -G DEB

This command creates a Debian package named libfranka-<version>-<architecture>.deb. You can then install it with:

.. code-block:: bash

   sudo dpkg -i libfranka*.deb

Installing via a Debian package simplifies the process compared to building from source every time. Additionally the package integrates better with
system tools and package managers, which can help manage updates and dependencies more effectively.

.. _usage:

6. Usage
~~~~~~~~

After installation, check the `Minimum System and Network Requirements <https://frankarobotics.github.io/docs/libfranka/docs/system_requirements.html>`_ for network settings,
the `Setting up the Real-Time Kernel <https://frankarobotics.github.io/docs/libfranka/docs/real_time_kernel.html>`_ for system setup,
and the `Getting Started Manual <https://frankarobotics.github.io/docs/libfranka/docs/getting_started.html>`_ for initial steps. Once configured,
you can control the robot using the example applications provided in the examples folder (`Usage Examples <https://frankarobotics.github.io/docs/libfranka/docs/usage_examples.html>`_).

To run a sample program, navigate to the build folder and execute the following command:

.. code-block:: bash

   ./examples/communication_test <robot-ip>

.. _pylibfranka:

7. Pylibfranka
~~~~~~~~~~~~~~

Pylibfranka is a Python binding for libfranka, allowing you to control Franka robots using Python. It is included in the libfranka repository and
can be built alongside libfranka. For more details, see ``pylibfranka`` and its `README <pylibfranka/README.md>`_.
The `generated API documentation <https://frankarobotics.github.io/libfranka/pylibfranka/latest>`_ offers an overview of its capabilities.

.. _development-information:

8. Development Information
~~~~~~~~~~~~~~~~~~~~~~~~~~

If you actively contribute to this repository, you should install and set up pre-commit hooks:

.. code-block:: bash

   pip install pre-commit
   pre-commit install

This will install pre-commit and set up the git hooks to automatically run checks before each commit.
The hooks will help maintain code quality by running various checks like code formatting, linting, and other validations.

To manually run the pre-commit checks on all files:

.. code-block:: bash

   pre-commit run --all-files

This will build the C++ extension and install the Python package.

License
-------

``libfranka`` is licensed under the `Apache 2.0 license <https://www.apache.org/licenses/LICENSE-2.0.html>`_.
