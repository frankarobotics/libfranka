Gripper
=======

.. currentmodule:: pylibfranka

Interface for controlling the Franka Hand gripper.

Gripper Class
-------------

.. autoclass:: Gripper
   :members:
   :undoc-members:
   :show-inheritance:

The :class:`Gripper` class provides control of the Franka Hand parallel-jaw gripper.

Methods
-------

.. automethod:: Gripper.__init__
   :noindex:

.. automethod:: Gripper.homing
   :noindex:

.. automethod:: Gripper.move
   :noindex:

.. automethod:: Gripper.grasp
   :noindex:

.. automethod:: Gripper.stop
   :noindex:

.. automethod:: Gripper.read_once
   :noindex:


GripperState Class
------------------

.. autoclass:: GripperState
   :members:
   :undoc-members:
   :show-inheritance:

Current state of the Franka Hand gripper.

See Also
--------

* :class:`Robot` -- Main robot control interface
* :class:`GripperState` -- Gripper state information
