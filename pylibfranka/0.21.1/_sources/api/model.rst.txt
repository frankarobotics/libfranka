Dynamics Model
==============

.. currentmodule:: pylibfranka

The :class:`Model` class provides robot dynamics model for computing kinematics and dynamics quantities.

Class Definition
----------------

.. autoclass:: Model
   :members:
   :undoc-members:
   :show-inheritance:

Overview
--------

The Model class allows computation of various dynamics quantities including:

* Mass matrix
* Coriolis and centrifugal forces
* Gravity torques
* Forward kinematics (Jacobian)

To obtain a Model instance, use :meth:`Robot.load_model`.

Methods
-------

coriolis
~~~~~~~~

.. py:method:: Model.coriolis(robot_state)
   :noindex:

   Compute Coriolis force vector :math:`c(q, \dot{q})`.

   :param robot_state: Current robot state
   :type robot_state: RobotState
   :return: Coriolis force vector [Nm] as numpy array (7,)
   :rtype: numpy.ndarray

   The Coriolis vector represents centrifugal and Coriolis forces acting on the robot.

   **Example:**

   .. code-block:: python

       import pylibfranka

       robot = pylibfranka.Robot("172.16.0.2")
       model = robot.load_model()
       state = robot.read_once()

       # Compute Coriolis forces
       coriolis = model.coriolis(state)
       print("Coriolis forces [Nm]:", coriolis)

gravity
~~~~~~~

.. py:method:: Model.gravity(robot_state)
   :noindex:

   Compute gravity force vector :math:`g(q)`.

   :param robot_state: Current robot state
   :type robot_state: RobotState
   :return: Gravity force vector [Nm] as numpy array (7,)
   :rtype: numpy.ndarray

   The gravity vector contains the joint torques required to compensate for gravitational forces.
   This is provided for dynamics analysis and computation purposes. When using torque control,
   gravity is automatically compensated by the controller, so you should not add gravity to your commands.

   **Example:**

   .. code-block:: python

       import pylibfranka

       robot = pylibfranka.Robot("172.16.0.2")
       model = robot.load_model()
       state = robot.read_once()

       # Compute gravity for analysis purposes
       gravity = model.gravity(state)
       print("Gravity torques [Nm]:", gravity)

mass
~~~~

.. py:method:: Model.mass(robot_state)
   :noindex:

   Compute mass matrix :math:`M(q)`.

   :param robot_state: Current robot state
   :type robot_state: RobotState
   :return: Mass matrix [kg·m²] as numpy array (7, 7)
   :rtype: numpy.ndarray

   The mass matrix (also called inertia matrix) relates joint accelerations to joint torques.

   **Example:**

   .. code-block:: python

       import pylibfranka
       import numpy as np

       robot = pylibfranka.Robot("172.16.0.2")
       model = robot.load_model()
       state = robot.read_once()

       # Compute mass matrix
       M = model.mass(state)
       print("Mass matrix shape:", M.shape)  # (7, 7)
       print("Mass matrix:\n", M)

       # Example: Compute required torques for desired accelerations
       ddq_desired = np.array([0.1, 0, 0, 0, 0, 0, 0])  # [rad/s²]
       tau = M @ ddq_desired
       print("Required torques:", tau)

zero_jacobian
~~~~~~~~~~~~~

.. py:method:: Model.zero_jacobian(robot_state)
               Model.zero_jacobian(frame, robot_state)
   :noindex:

   Compute zero Jacobian :math:`^0J` for specified frame.

   :param frame: Frame to compute Jacobian for (optional, default: end effector)
   :type frame: Frame
   :param robot_state: Current robot state
   :type robot_state: RobotState
   :return: Jacobian matrix as numpy array (6, 7)
   :rtype: numpy.ndarray

   The Jacobian maps joint velocities to end-effector twist (linear and angular velocities).
   
   The returned Jacobian has shape (6, 7) where:
   
   * Rows 0-2: Linear velocity mapping [m/s per rad/s]
   * Rows 3-5: Angular velocity mapping [rad/s per rad/s]

   **Example:**

   .. code-block:: python

       import pylibfranka
       import numpy as np

       robot = pylibfranka.Robot("172.16.0.2")
       model = robot.load_model()
       state = robot.read_once()

       # Compute end-effector Jacobian
       J = model.zero_jacobian(state)
       print("Jacobian shape:", J.shape)  # (6, 7)

       # Compute end-effector velocity from joint velocities
       dq = np.array(state.dq)  # Joint velocities [rad/s]
       dx = J @ dq  # End-effector twist [m/s, rad/s]
       
       v_linear = dx[:3]   # Linear velocity [m/s]
       w_angular = dx[3:]  # Angular velocity [rad/s]
       
       print("Linear velocity [m/s]:", v_linear)
       print("Angular velocity [rad/s]:", w_angular)

See Also
--------

* :class:`Robot` -- Main robot interface (use :meth:`Robot.load_model` to get Model)
* :class:`RobotState` -- Robot state used as input for dynamics computation
* :class:`ActiveControlBase` -- Control interface for sending computed torques
