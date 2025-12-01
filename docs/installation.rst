.. _libfranka_installation:

Build / Installation
--------------------

.. _libfranka_installation_debian_package:
Debian Package
~~~~~~~~~~~~~~

Starting with version 0.18.1, **libfranka** releases are provided as pre-built Debian packages.
You can find the packaged artifacts on the `libfranka releases <https://github.com/frankarobotics/libfranka/releases/>`_ page on GitHub.

Download the Debian package:

.. code-block:: bash

   wget https://github.com/frankarobotics/libfranka/releases/download/0.18.1/libfranka_0.18.1_amd64.deb

Install the package on your system:

.. code-block:: bash

   sudo dpkg -i libfranka_0.18.1_amd64.deb

For other versions or architectures, use the following pattern:

.. code-block:: bash

   wget https://github.com/frankarobotics/libfranka/releases/download/<version>/libfranka_<version>_<architecture>.deb
   sudo dpkg -i libfranka_<version>_<architecture>.deb

This is the recommended installation method for **libfranka** if you do not need to modify the source code.

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

TODO: Add pip installation instructions

.. _libfranka_installation_python_bindings:
Building Python Bindings from Source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

TODO: Add instructions for building and installing pylibfranka from source

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
