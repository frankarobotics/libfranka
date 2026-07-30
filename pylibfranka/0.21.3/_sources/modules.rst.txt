Module Reference
================

.. currentmodule:: pylibfranka

This page provides an overview of the module structure in pylibfranka.

Main Module
-----------

All classes and types are available directly from the top-level :mod:`pylibfranka` module:

.. code-block:: python

    import pylibfranka
    
    robot = pylibfranka.Robot("172.16.0.2")
    gripper = pylibfranka.Gripper("172.16.0.2")

Module Contents
~~~~~~~~~~~~~~~

.. autosummary::
   :nosignatures:

   Robot
   ActiveControlBase
   Model
   Gripper
   GripperState
   RobotState
   RobotMode
   Errors
   JointPositions
   JointVelocities
   CartesianPose
   CartesianVelocities
   Torques
   Duration
   RealtimeConfig
   ControllerMode
   FrankaException
   CommandException
   ControlException
   NetworkException
   InvalidOperationException
   RealtimeException

Detailed API Reference by Category
-----------------------------------

For detailed documentation organized by functionality, see:

.. toctree::
   :maxdepth: 1

   api/robot
   api/control
   api/model
   api/gripper
   api/state
   api/exceptions

Organization by Functionality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Robot Control
^^^^^^^^^^^^^

Classes for establishing connection and controlling the robot:

* :class:`Robot` - Main robot interface
* :class:`ActiveControlBase` - Active control session
* :class:`RealtimeConfig` - Real-time configuration
* :class:`ControllerMode` - Controller mode selection

Motion Commands
^^^^^^^^^^^^^^^

Data structures for sending motion commands:

* :class:`JointPositions` - Joint position commands
* :class:`JointVelocities` - Joint velocity commands
* :class:`CartesianPose` - Cartesian pose commands
* :class:`CartesianVelocities` - Cartesian velocity commands
* :class:`Torques` - Torque commands

Robot State and Model
^^^^^^^^^^^^^^^^^^^^^

Classes for reading robot state and computing dynamics:

* :class:`RobotState` - Complete robot state
* :class:`Model` - Dynamics model (mass matrix, Coriolis, gravity, etc.)
* :class:`RobotMode` - Operation mode
* :class:`Errors` - Error state

Gripper
^^^^^^^

Classes for gripper control:

* :class:`Gripper` - Gripper interface
* :class:`GripperState` - Gripper state

Exceptions
^^^^^^^^^^

Exception hierarchy for error handling:

* :class:`FrankaException` - Base exception
* :class:`CommandException` - Command errors
* :class:`ControlException` - Control/motion errors
* :class:`NetworkException` - Network errors
* :class:`InvalidOperationException` - Invalid operation errors
* :class:`RealtimeException` - Real-time scheduling errors

See Also
--------

* :doc:`class_list` - Complete class list with descriptions
* :doc:`class_hierarchy` - Class inheritance structure
* :ref:`genindex` - Complete index

