Class Hierarchy
===============

.. currentmodule:: pylibfranka

This page shows the class hierarchy for all classes in pylibfranka.

Exception Hierarchy
-------------------

All exceptions inherit from the base :class:`FrankaException`:

.. inheritance-diagram:: FrankaException CommandException ControlException NetworkException InvalidOperationException RealtimeException
   :parts: 1
   :top-classes: Exception

Text Representation
~~~~~~~~~~~~~~~~~~~

.. code-block:: text

    Exception (Python built-in)
    └── FrankaException
        ├── CommandException
        ├── ControlException
        ├── NetworkException
        ├── InvalidOperationException
        └── RealtimeException

Control and State Classes
--------------------------

The main classes for robot control and state access:

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Class
     - Description
   * - :class:`Robot`
     - Main robot control interface
   * - :class:`ActiveControlBase`
     - Active control session for reading state and writing commands
   * - :class:`RobotState`
     - Complete robot state information
   * - :class:`Model`
     - Robot dynamics model

Gripper Classes
---------------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Class
     - Description
   * - :class:`Gripper`
     - Gripper control interface
   * - :class:`GripperState`
     - Gripper state information

Motion Command Types
---------------------

All motion command types are independent data structures:

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Class
     - Description
   * - :class:`JointPositions`
     - Joint space position commands
   * - :class:`JointVelocities`
     - Joint space velocity commands
   * - :class:`CartesianPose`
     - Cartesian space pose commands
   * - :class:`CartesianVelocities`
     - Cartesian space velocity commands
   * - :class:`Torques`
     - Joint torque commands

Enumeration Types
-----------------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Enum
     - Description
   * - :class:`RealtimeConfig`
     - Real-time enforcement mode (kEnforce, kIgnore)
   * - :class:`ControllerMode`
     - Controller type (JointImpedance, CartesianImpedance)
   * - :class:`RobotMode`
     - Current robot operation mode

Other Types
-----------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Class
     - Description
   * - :class:`Duration`
     - Time duration representation
   * - :class:`Errors`
     - Robot error state flags

See Also
--------

* :doc:`class_list` - Alphabetical class list
* :doc:`api/exceptions` - Detailed exception documentation

