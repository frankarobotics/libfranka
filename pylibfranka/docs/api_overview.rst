API Overview
============

.. currentmodule:: pylibfranka

This page provides a complete overview of all classes, types, and functions in pylibfranka.

Complete Class Reference
-------------------------

Control and Robot Classes
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 5 30 65
   :header-rows: 1

   * - Type
     - Name
     - Description
   * - **C**
     - :class:`ActiveControlBase`
     - Allows the user to read the state of a Robot and to send new control commands after starting a control process of a Robot
   * - **C**
     - :class:`Model`
     - Calculates poses of joints and dynamic properties of the robot
   * - **C**
     - :class:`Robot`
     - Maintains a network connection to the robot, provides the current robot state, gives access to the model library and allows to control the robot

Gripper Classes
~~~~~~~~~~~~~~~

.. list-table::
   :widths: 5 30 65
   :header-rows: 1

   * - Type
     - Name
     - Description
   * - **C**
     - :class:`Gripper`
     - Maintains a network connection to the gripper, provides the current gripper state, and allows the execution of commands
   * - **C**
     - :class:`GripperState`
     - Describes the gripper state

State and Error Classes
~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 5 30 65
   :header-rows: 1

   * - Type
     - Name
     - Description
   * - **C**
     - :class:`Errors`
     - Enumerates errors that can occur while controlling a Robot
   * - **C**
     - :class:`RobotState`
     - Describes the robot state including joint positions, velocities, torques, and transformation matrices

Motion Command Types
~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 5 30 65
   :header-rows: 1

   * - Type
     - Name
     - Description
   * - **C**
     - :class:`CartesianPose`
     - Stores values for Cartesian pose motion generation
   * - **C**
     - :class:`CartesianVelocities`
     - Stores values for Cartesian velocity motion generation
   * - **C**
     - :class:`JointPositions`
     - Stores values for joint position motion generation
   * - **C**
     - :class:`JointVelocities`
     - Stores values for joint velocity motion generation
   * - **C**
     - :class:`Torques`
     - Stores values for torque control

Enumeration Types
~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 5 30 65
   :header-rows: 1

   * - Type
     - Name
     - Description
   * - **E**
     - :class:`ControllerMode`
     - Enumeration for controller mode selection (JointImpedance, CartesianImpedance)
   * - **E**
     - :class:`RealtimeConfig`
     - Enumeration for real-time scheduling requirements (kEnforce, kIgnore)
   * - **E**
     - :class:`RobotMode`
     - Enumerates different robot operation modes

Utility Types
~~~~~~~~~~~~~

.. list-table::
   :widths: 5 30 65
   :header-rows: 1

   * - Type
     - Name
     - Description
   * - **C**
     - :class:`Duration`
     - Represents a duration with millisecond resolution

Exception Classes
~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 5 30 65
   :header-rows: 1

   * - Type
     - Name
     - Description
   * - **C**
     - :class:`CommandException`
     - Thrown if an error occurs during command execution
   * - **C**
     - :class:`ControlException`
     - Thrown if an error occurs during motion generation or torque control
   * - **C**
     - :class:`FrankaException`
     - Base class for all exceptions used by libfranka
   * - **C**
     - :class:`InvalidOperationException`
     - Thrown if an operation cannot be performed
   * - **C**
     - :class:`NetworkException`
     - Thrown if a connection to the robot cannot be established, or when a timeout occurs
   * - **C**
     - :class:`RealtimeException`
     - Thrown if real-time scheduling requirements cannot be met

Legend
------

* **C** = Class/Type
* **E** = Enumeration

Quick Links by Category
------------------------

Robot Control
~~~~~~~~~~~~~

* :class:`Robot` - Main robot interface
* :class:`ActiveControlBase` - Active control loop interface
* :class:`Model` - Dynamics computations

Gripper Control
~~~~~~~~~~~~~~~

* :class:`Gripper` - Gripper interface
* :class:`GripperState` - Gripper state

State Access
~~~~~~~~~~~~

* :class:`RobotState` - Complete robot state
* :class:`RobotMode` - Robot operation mode
* :class:`Errors` - Error flags

Motion Commands
~~~~~~~~~~~~~~~

* :class:`Torques` - Torque commands
* :class:`JointPositions` - Joint position commands
* :class:`JointVelocities` - Joint velocity commands
* :class:`CartesianPose` - Cartesian pose commands
* :class:`CartesianVelocities` - Cartesian velocity commands

Error Handling
~~~~~~~~~~~~~~

* :class:`FrankaException` - Base exception
* :class:`CommandException` - Command errors
* :class:`ControlException` - Control errors
* :class:`NetworkException` - Network errors
* :class:`InvalidOperationException` - Invalid operations
* :class:`RealtimeException` - Real-time errors

Configuration
~~~~~~~~~~~~~

* :class:`RealtimeConfig` - Real-time enforcement
* :class:`ControllerMode` - Controller selection
* :class:`Duration` - Time representation

See Also
--------

* :doc:`class_hierarchy` - View inheritance relationships
* :doc:`modules` - Browse by module organization
* :ref:`genindex` - Alphabetical index

