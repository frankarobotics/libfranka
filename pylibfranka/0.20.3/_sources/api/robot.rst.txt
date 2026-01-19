Robot
=====

.. currentmodule:: pylibfranka

The :class:`Robot` class maintains a network connection to the robot, provides the current robot state, gives access to the model library and allows to control the robot.

Class Definition
----------------

.. autoclass:: Robot
   :members:
   :undoc-members:
   :show-inheritance:

Constructor
-----------

.. py:method:: Robot(robot_ip_address, realtime_config=RealtimeConfig.kEnforce)
   :noindex:

   Establishes a connection with the robot.

   :param robot_ip_address: IP address or hostname of the robot controller
   :type robot_ip_address: str
   :param realtime_config: Real-time scheduling requirements. Default: ``RealtimeConfig.kEnforce``
   :type realtime_config: RealtimeConfig
   :raises NetworkException: if the connection cannot be established
   :raises IncompatibleVersionException: if the robot's software version is incompatible

   **Example:**

   .. code-block:: python

       import pylibfranka

       # Connect with real-time enforcement
       robot = pylibfranka.Robot("172.16.0.2")

       # Connect without real-time enforcement (for testing)
       robot_test = pylibfranka.Robot(
           "172.16.0.2",
           pylibfranka.RealtimeConfig.kIgnore
       )

Control Methods
---------------

start_torque_control
~~~~~~~~~~~~~~~~~~~~

.. py:method:: Robot.start_torque_control()
   :noindex:

   Starts a new torque controller.

   :return: Active control interface for sending torque commands
   :rtype: ActiveControlBase
   :raises ControlException: if an error related to torque control occurred
   :raises InvalidOperationException: if a conflicting operation is already running
   :raises NetworkException: if the connection is lost, e.g. after a timeout

   **Example:**

   .. code-block:: python

       robot = pylibfranka.Robot("172.16.0.2")

       # Start torque control mode (gravity is automatically compensated)
       control = robot.start_torque_control()

       # Control loop at 1 kHz
       for i in range(5000):
           state, duration = control.readOnce()

           # Send zero torque for compliant mode
           tau = pylibfranka.Torques([0, 0, 0, 0, 0, 0, 0])

           # Send torque command
           control.writeOnce(tau)

   .. seealso::
      :meth:`ActiveControlBase.writeOnce` for sending torque commands

start_joint_position_control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. py:method:: Robot.start_joint_position_control(control_type)
   :noindex:

   Starts a new joint position motion generator.

   :param control_type: Controller mode for the operation
   :type control_type: ControllerMode
   :return: Active control interface for sending position commands
   :rtype: ActiveControlBase
   :raises ControlException: if an error related to motion generation occurred
   :raises InvalidOperationException: if a conflicting operation is already running
   :raises NetworkException: if the connection is lost, e.g. after a timeout

start_joint_velocity_control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. py:method:: Robot.start_joint_velocity_control(control_type)
   :noindex:

   Starts a new joint velocity motion generator.

   :param control_type: Controller mode for the operation
   :type control_type: ControllerMode
   :return: Active control interface for sending velocity commands
   :rtype: ActiveControlBase
   :raises ControlException: if an error related to motion generation occurred
   :raises InvalidOperationException: if a conflicting operation is already running
   :raises NetworkException: if the connection is lost, e.g. after a timeout

start_cartesian_pose_control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. py:method:: Robot.start_cartesian_pose_control(control_type)
   :noindex:

   Starts a new cartesian pose motion generator.

   :param control_type: Controller mode for the operation
   :type control_type: ControllerMode
   :return: Active control interface for sending position commands
   :rtype: ActiveControlBase
   :raises ControlException: if an error related to motion generation occurred
   :raises InvalidOperationException: if a conflicting operation is already running
   :raises NetworkException: if the connection is lost, e.g. after a timeout

start_cartesian_velocity_control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. py:method:: Robot.start_cartesian_velocity_control(control_type)
   :noindex:

   Starts a new cartesian velocity motion generator.

   :param control_type: Controller mode for the operation
   :type control_type: ControllerMode
   :return: Active control interface for sending velocity commands
   :rtype: ActiveControlBase
   :raises ControlException: if an error related to motion generation occurred
   :raises InvalidOperationException: if a conflicting operation is already running
   :raises NetworkException: if the connection is lost, e.g. after a timeout

State and Model
---------------

read_once
~~~~~~~~~

.. py:method:: Robot.read_once()
   :noindex:

   Read current robot state once.

   :return: Current robot state containing all state information
   :rtype: RobotState
   :raises NetworkException: if the connection is lost

load_model
~~~~~~~~~~

.. py:method:: Robot.load_model()
   :noindex:

   Load robot dynamics model.

   :return: Model object for computing dynamics quantities
   :rtype: Model


Configuration Methods
---------------------

set_collision_behavior
~~~~~~~~~~~~~~~~~~~~~~~

.. py:method:: Robot.set_collision_behavior(lower_torque_thresholds, upper_torque_thresholds, lower_force_thresholds, upper_force_thresholds)
   :noindex:

   Configure collision detection thresholds.

   :param lower_torque_thresholds: Lower torque thresholds [Nm] for each joint (7,)
   :type lower_torque_thresholds: array_like
   :param upper_torque_thresholds: Upper torque thresholds [Nm] for each joint (7,)
   :type upper_torque_thresholds: array_like
   :param lower_force_thresholds: Lower Cartesian force thresholds [N, Nm] (6,)
   :type lower_force_thresholds: array_like
   :param upper_force_thresholds: Upper Cartesian force thresholds [N, Nm] (6,)
   :type upper_force_thresholds: array_like
   :raises CommandException: if the Control reports an error
   :raises NetworkException: if the connection is lost

set_joint_impedance
~~~~~~~~~~~~~~~~~~~

.. py:method:: Robot.set_joint_impedance(K_theta)
   :noindex:

   Sets the impedance for each joint in the internal controller.

   User-provided torques are not affected by this setting.

   :param K_theta: Joint impedance values :math:`K_{\theta_{1-7}} \in [0, 14250]` [Nm/rad] (7,)
   :type K_theta: array_like
   :raises CommandException: if the Control reports an error
   :raises NetworkException: if the connection is lost

set_cartesian_impedance
~~~~~~~~~~~~~~~~~~~~~~~~

.. py:method:: Robot.set_cartesian_impedance(K_x)
   :noindex:

   Sets the Cartesian impedance for the internal controller.

   User-provided torques are not affected by this setting.

   :param K_x: Cartesian impedance values [N/m, Nm/rad] for (x, y, z, roll, pitch, yaw) (6,)
   :type K_x: array_like
   :raises CommandException: if the Control reports an error
   :raises NetworkException: if the connection is lost


set_K
~~~~~

.. py:method:: Robot.set_K(EE_T_K)
   :noindex:

   Sets the transformation :math:`^{EE}T_K` from end effector frame to stiffness frame.

   The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.

   :param EE_T_K: Vectorized EE-to-K transformation matrix :math:`^{EE}T_K`, column-major (16,)
   :type EE_T_K: array_like
   :raises CommandException: if the Control reports an error
   :raises NetworkException: if the connection is lost


set_EE
~~~~~~

.. py:method:: Robot.set_EE(NE_T_EE)
   :noindex:

   Sets the transformation :math:`^{NE}T_{EE}` from nominal end effector to end effector frame.

   The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.

   :param NE_T_EE: Vectorized NE-to-EE transformation matrix :math:`^{NE}T_{EE}`, column-major (16,)
   :type NE_T_EE: array_like
   :raises CommandException: if the Control reports an error
   :raises NetworkException: if the connection is lost

   .. seealso::
      * :attr:`RobotState.NE_T_EE` for end effector pose in nominal end effector frame
      * :attr:`RobotState.O_T_EE` for end effector pose in world base frame
      * :attr:`RobotState.F_T_EE` for end effector pose in flange frame


set_load
~~~~~~~~

.. py:method:: Robot.set_load(load_mass, F_x_Cload, load_inertia)
   :noindex:

   Sets dynamic parameters of a payload.

   .. note::
      This is not for setting end effector parameters, which have to be set in the administrator's interface.

   :param load_mass: Mass of the load [kg]
   :type load_mass: float
   :param F_x_Cload: Translation from flange to center of mass of load :math:`^Fx_{C_{load}}` [m] (3,)
   :type F_x_Cload: array_like
   :param load_inertia: Inertia matrix :math:`I_{load}` [kg·m²], column-major (9,)
   :type load_inertia: array_like
   :raises CommandException: if the Control reports an error
   :raises NetworkException: if the connection is lost

Error Handling
--------------

automatic_error_recovery
~~~~~~~~~~~~~~~~~~~~~~~~

.. py:method:: Robot.automatic_error_recovery()
   :noindex:

   Attempt automatic error recovery.

   Executes automatic error recovery if the robot is in an error state. This can clear recoverable errors.

   :raises CommandException: if recovery fails
   :raises NetworkException: if the connection is lost



stop
~~~~

.. py:method:: Robot.stop()
   :noindex:

   Stops all currently running motions.

   If a control or motion generator loop is running in another thread, it will be preempted with a :class:`ControlException`.

   :raises CommandException: if the Control reports an error
   :raises NetworkException: if the connection is lost


Related Classes
---------------

RealtimeConfig
~~~~~~~~~~~~~~

.. autoclass:: RealtimeConfig
   :members:
   :undoc-members:

   Enumeration for real-time configuration options.

   :var kEnforce: Enforce real-time requirements (recommended for actual control)
   :var kIgnore: Ignore real-time requirements (useful for testing/development)

ControllerMode
~~~~~~~~~~~~~~

.. autoclass:: ControllerMode
   :members:
   :undoc-members:

   Enumeration for controller mode selection.

   :var JointImpedance: Joint impedance control mode
   :var CartesianImpedance: Cartesian impedance control mode

See Also
--------

* :class:`ActiveControlBase` -- Control interface returned by start_* methods
* :class:`RobotState` -- Robot state data structure
* :class:`Model` -- Dynamics model for computing kinematics and dynamics
* :class:`Errors` -- Robot error state information
