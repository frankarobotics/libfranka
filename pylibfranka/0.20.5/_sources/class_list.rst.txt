Class List
==========

.. currentmodule:: pylibfranka

Here are all the classes, structs, enums, and types with brief descriptions:

Core Robot Control
------------------

.. list-table::
   :widths: 30 70
   :header-rows: 0

   * - :class:`Robot`
     - Maintains a network connection to the robot, provides the current robot state, gives access to the model library and allows to control the robot
   * - :class:`ActiveControlBase`
     - Allows the user to read the state of a Robot and to send new control commands after starting a control process
   * - :class:`Model`
     - Calculates poses of joints and dynamic properties of the robot

Gripper Control
---------------

.. list-table::
   :widths: 30 70
   :header-rows: 0

   * - :class:`Gripper`
     - Maintains a network connection to the gripper, provides the current gripper state, and allows the execution of commands
   * - :class:`GripperState`
     - Describes the gripper state

Robot State
-----------

.. list-table::
   :widths: 30 70
   :header-rows: 0

   * - :class:`RobotState`
     - Describes the robot state including joint positions, velocities, torques, and errors
   * - :class:`RobotMode`
     - Enumerates different robot operation modes
   * - :class:`Errors`
     - Enumerates errors that can occur while controlling a Robot

Motion Types
------------

.. list-table::
   :widths: 30 70
   :header-rows: 0

   * - :class:`JointPositions`
     - Stores values for joint position motion generation
   * - :class:`JointVelocities`
     - Stores values for joint velocity motion generation
   * - :class:`CartesianPose`
     - Stores values for Cartesian pose motion generation
   * - :class:`CartesianVelocities`
     - Stores values for Cartesian velocity motion generation
   * - :class:`Torques`
     - Stores values for torque control

Configuration Types
-------------------

.. list-table::
   :widths: 30 70
   :header-rows: 0

   * - :class:`Duration`
     - Represents a duration with millisecond resolution
   * - :class:`RealtimeConfig`
     - Enumeration for real-time scheduling requirements
   * - :class:`ControllerMode`
     - Enumeration for controller mode selection (joint or Cartesian impedance)

Exceptions
----------

.. list-table::
   :widths: 30 70
   :header-rows: 0

   * - :class:`FrankaException`
     - Base class for all exceptions used by libfranka
   * - :class:`CommandException`
     - Thrown if an error occurs during command execution
   * - :class:`ControlException`
     - Thrown if an error occurs during motion generation or torque control
   * - :class:`NetworkException`
     - Thrown if a connection to the robot cannot be established, or when a timeout occurs
   * - :class:`InvalidOperationException`
     - Thrown if an operation cannot be performed
   * - :class:`RealtimeException`
     - Thrown if real-time scheduling requirements cannot be met

See Also
--------

* :doc:`class_hierarchy` - View the inheritance structure
* :doc:`modules` - Browse by module

