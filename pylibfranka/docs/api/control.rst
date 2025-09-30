Control Interface
=================

.. currentmodule:: pylibfranka

The control interface provides real-time command and feedback for robot control.
It is returned by the ``start_*`` methods of the :class:`Robot` class.

ActiveControlBase Class
-----------------------

.. autoclass:: ActiveControlBase
   :members:
   :undoc-members:
   :show-inheritance:

Methods
~~~~~~~

readOnce
^^^^^^^^

.. py:method:: ActiveControlBase.readOnce()
   :noindex:

   Read robot state once from the control loop.

   :return: Tuple of (RobotState, Duration) containing the robot state and time since last read operation
   :rtype: tuple(RobotState, Duration)

writeOnce (Torques)
^^^^^^^^^^^^^^^^^^^

.. py:method:: ActiveControlBase.writeOnce(command: Torques)
   :noindex:

   Write torque command to the robot.

   :param command: Torque command to send
   :type command: Torques


writeOnce (JointPositions)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. py:method:: ActiveControlBase.writeOnce(command: JointPositions)
   :noindex:

   Write joint position command to the robot.

   :param command: Joint position command to send
   :type command: JointPositions


writeOnce (JointVelocities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. py:method:: ActiveControlBase.writeOnce(command: JointVelocities)
   :noindex:

   Write joint velocity command to the robot.

   :param command: Joint velocity command to send
   :type command: JointVelocities


writeOnce (CartesianPose)
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. py:method:: ActiveControlBase.writeOnce(command: CartesianPose)
   :noindex:

   Write Cartesian pose command to the robot.

   :param command: Cartesian pose command to send
   :type command: CartesianPose

writeOnce (CartesianVelocities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. py:method:: ActiveControlBase.writeOnce(command: CartesianVelocities)
   :noindex:

   Write Cartesian velocity command to the robot.

   :param command: Cartesian velocity command to send
   :type command: CartesianVelocities

Command Types
-------------

Torques
~~~~~~~

.. autoclass:: Torques
   :members:
   :undoc-members:

   Torque control command for joint-level torque control.

   **Attributes:**

   .. py:attribute:: tau_J
      :type: array_like
      :noindex:

      Joint torques [Nm] (7,)

   .. py:attribute:: motion_finished
      :type: bool
      :noindex:

      Set to True to finish motion and exit control loop

   **Constructor:**

   .. py:method:: Torques(tau_J)

      Create torque command.

      :param tau_J: Joint torques [Nm] (7,)
      :type tau_J: array_like


JointPositions
~~~~~~~~~~~~~~

.. autoclass:: JointPositions
   :members:
   :undoc-members:

   Joint position control command.

   **Attributes:**

   .. py:attribute:: q
      :type: array_like
      :noindex:

      Joint positions [rad] (7,)

   .. py:attribute:: motion_finished
      :type: bool
      :noindex:

      Set to True to finish motion and exit control loop

   **Constructor:**

   .. py:method:: JointPositions(q)

      Create joint position command.

      :param q: Joint positions [rad] (7,)
      :type q: array_like


JointVelocities
~~~~~~~~~~~~~~~

.. autoclass:: JointVelocities
   :members:
   :undoc-members:

   Joint velocity control command.

   **Attributes:**

   .. py:attribute:: dq
      :type: array_like
      :noindex:

      Joint velocities [rad/s] (7,)

   .. py:attribute:: motion_finished
      :type: bool
      :noindex:

      Set to True to finish motion and exit control loop

   **Constructor:**

   .. py:method:: JointVelocities(dq)

      Create joint velocity command.

      :param dq: Joint velocities [rad/s] (7,)
      :type dq: array_like


CartesianPose
~~~~~~~~~~~~~

.. autoclass:: CartesianPose
   :members:
   :undoc-members:

   Cartesian pose control command.

   **Attributes:**

   .. py:attribute:: O_T_EE
      :type: array_like
      :noindex:

      End effector pose as homogeneous transformation matrix (16,) in column-major order

   .. py:attribute:: motion_finished
      :type: bool
      :noindex:

      Set to True to finish motion and exit control loop

   **Constructor:**

   .. py:method:: CartesianPose(O_T_EE)

      Create Cartesian pose command.

      :param O_T_EE: Homogeneous transformation matrix (16,) in column-major order
      :type O_T_EE: array_like


CartesianVelocities
~~~~~~~~~~~~~~~~~~~

.. autoclass:: CartesianVelocities
   :members:
   :undoc-members:

   Cartesian velocity control command (twist).

   **Attributes:**

   .. py:attribute:: O_dP_EE
      :type: array_like
      :noindex:

      End effector twist [vx, vy, vz, wx, wy, wz] in [m/s, rad/s] (6,)

   .. py:attribute:: motion_finished
      :type: bool
      :noindex:

      Set to True to finish motion and exit control loop

   **Constructor:**

   .. py:method:: CartesianVelocities(O_dP_EE)

      Create Cartesian velocity command.

      :param O_dP_EE: End effector twist [vx, vy, vz, wx, wy, wz] in [m/s, rad/s] (6,)
      :type O_dP_EE: array_like


See Also
--------

* :class:`Robot` -- Main robot control interface
* :class:`RobotState` -- Robot state information
* :class:`Model` -- Dynamics model
