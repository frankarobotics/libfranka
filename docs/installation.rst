.. _libfranka_installation:

Build / Installation
--------------------

.. note::

   **libfranka** supports Ubuntu 20.04 (focal), 22.04 (jammy), and 24.04 (noble) LTS versions. Pre-built **amd64** Debian packages are available for all supported versions.

.. _libfranka_installation_debian_package:

Debian Package
~~~~~~~~~~~~~~

**libfranka** releases are provided as pre-built Debian packages for multiple Ubuntu LTS versions.
You can find the packaged artifacts on the `libfranka releases <https://github.com/frankarobotics/libfranka/releases/>`_ page on GitHub.


Installation Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^

**Step 1: Check your Ubuntu version**

.. code-block:: bash

   lsb_release -a

**Step 2: Download and install the appropriate package**


For the supported ubuntu versions, use the following pattern:

.. code-block:: bash

   # Replace with your desired version and Ubuntu codename
   VERSION=0.19.0
   CODENAME=focal  # or jammy, noble

   wget https://github.com/frankarobotics/libfranka/releases/download/${VERSION}/libfranka_${VERSION}_${CODENAME}_amd64.deb
   sudo dpkg -i libfranka_${VERSION}_${CODENAME}_amd64.deb

.. tip::

   This is the recommended installation method for **libfranka** if you do not need to modify the source code.

Verify Installation
^^^^^^^^^^^^^^^^^^^

After installation, verify that libfranka is correctly installed:

.. code-block:: bash

   dpkg -l | grep libfranka

.. _libfranka_installation_docker:
Inside docker container
~~~~~~~~~~~~~~~~~~~~~~~
If you prefer to build **libfranka** inside a Docker container, you can use the
provided Docker setup. Follow :ref:`building-in-docker` for detailed building
and installation instructions.

.. _libfranka_installation_plain_cmake:
Plain CMake
~~~~~~~~~~~

Before you begin, check the :ref:`system-requirements` and install the necessary dependencies listed in :ref:`installing-dependencies`.

After installing the dependencies, proceed to :ref:`building-from-source`.

Once the build is complete, you can optionally create and install a Debian package as described in :ref:`installing-debian-package`.

.. _libfranka_installation_pip:
Python Package (pip)
~~~~~~~~~~~~~~~~~~~~

**pylibfranka** is a Python binding for libfranka that enables control of Franka Robotics robots from Python.
Pre-built wheels are available on PyPI and include all necessary dependencies.

Installation
^^^^^^^^^^^^

The easiest way to install pylibfranka is via pip:

.. code-block:: bash

   pip install pylibfranka

Platform Compatibility
^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Ubuntu Version
     - Supported Python Versions
   * - 20.04 (Focal)
     - Python 3.9 only
   * - 22.04 (Jammy)
     - Python 3.9, 3.10, 3.11, 3.12
   * - 24.04 (Noble)
     - Python 3.9, 3.10, 3.11, 3.12

.. note::

   Ubuntu 20.04 users must use Python 3.9 due to glibc compatibility requirements.

Verify Installation
^^^^^^^^^^^^^^^^^^^

After installation, verify that pylibfranka is correctly installed:

.. code-block:: python

   import pylibfranka
   print(f"pylibfranka version: {pylibfranka.__version__}")

   # Verify key classes are available
   from pylibfranka import Robot, Gripper, Model, RobotState
   print("All core classes imported successfully")

Quick Start Example
^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import pylibfranka

   # Connect to the robot
   robot = pylibfranka.Robot("172.16.0.2")
   state = robot.read_once()

   # Print joint positions
   print(f"Joint positions: {state.q}")

For more examples, see the `pylibfranka examples <https://github.com/frankarobotics/libfranka/tree/main/pylibfranka/examples>`_ on GitHub.

.. _libfranka_installation_python_bindings:
Building Python Bindings from Source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you need to build from source (e.g., for development or unsupported platforms), follow these instructions.

Prerequisites
^^^^^^^^^^^^^

- Python 3.9 or newer
- CMake 3.16 or newer
- C++ compiler with C++17 support
- Eigen3 development headers
- Poco development headers

.. tip::

   If you are using the provided devcontainer, you can skip the prerequisites installation as they are already included in the container.

Installing Prerequisites on Ubuntu/Debian
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt-get update
   sudo apt-get install -y build-essential cmake libeigen3-dev libpoco-dev python3-dev

Build and Install
^^^^^^^^^^^^^^^^^

From the libfranka root folder, install pylibfranka using pip:

.. code-block:: bash

   pip install .

This will build the C++ extension and install pylibfranka in your current Python environment.

For development (editable install):

.. code-block:: bash

   pip install -e .

Inside a ROS / ROS 2 workspace
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2
^^^^^

TODO: Add ROS 2 installation instructions

ROS Noetic
^^^^^^^^^^

.. note::

   While ``libfranka`` and the ``franka_ros`` packages should work on different Linux distributions,
   official support is currently only provided for:

   * Ubuntu 18.04 LTS `Bionic Beaver` and ROS `Melodic Morenia` (requires at least ``libfranka`` 0.6.0)
   * Ubuntu 20.04 LTS `Focal Fossa` and ROS `Noetic Ninjemys` (requires at least ``libfranka`` 0.8.0)

   The following instructions are exemplary for Ubuntu 20.04 LTS system and ROS `Noetic Ninjemys`.
   They only work in the supported environments.

**Building the ROS packages**

After `setting up ROS Noetic <https://wiki.ros.org/noetic/Installation/Ubuntu>`__, create a Catkin workspace in a directory of your choice:

.. code-block:: shell

    cd /path/to/desired/folder
    mkdir -p catkin_ws/src
    cd catkin_ws
    source /opt/ros/noetic/setup.sh
    catkin_init_workspace src

Then clone the ``franka_ros`` repository from `GitHub <https://github.com/frankarobotics/franka_ros>`__:

.. code-block:: shell

    git clone --recursive https://github.com/frankarobotics/franka_ros src/franka_ros

By default, this will check out the newest release of ``franka_ros``. If you want to build a particular version of ``franka_ros`` instead, check out the corresponding Git tag:

.. code-block:: shell

    git checkout <version>

Install any missing dependencies and build the packages:

.. code-block:: shell

    rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
    catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
    source devel/setup.sh

.. warning::
    If you also installed ``ros-noetic-libfranka``, ``libfranka`` might be picked up from ``/opt/ros/noetic`` instead of from your custom ``libfranka`` build!

Use this Library in Other Projects
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To get started with **libfranka** in your own CMake projects, you need a simple `CMakeLists.txt` and a corresponding `main.cpp` file.

At a minimum, your project must:

- Use ``find_package(Franka REQUIRED)`` to locate the library
- Link against ``Franka::Franka`` inside your ``target_link_libraries(...)`` command

Below is an example `CMakeLists.txt` file:

.. literalinclude:: assets/example_cmake_lists.txt
   :language: cmake

This example uses ``find_package`` to detect **libfranka** and sets up an executable target that links to the library.

A matching `example_main.cpp` file could look like this:

.. literalinclude:: assets/example_main.cpp
   :language: cpp

To build the project:

.. code-block:: bash

   mkdir build && cd build
   cmake ..
   cmake --build .

Once built, run your example:

.. code-block:: bash

   ./example_main
