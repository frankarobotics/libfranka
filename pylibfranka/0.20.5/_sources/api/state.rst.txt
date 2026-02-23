Robot State
===========

.. currentmodule:: pylibfranka

Robot state information including joint positions, velocities, torques, and Cartesian poses.

RobotState Class
----------------

.. autoclass:: RobotState
   :members:
   :undoc-members:

The :class:`RobotState` contains the complete state of the Franka robot at a given time instant.
All arrays are NumPy arrays for seamless integration with scientific Python libraries.

Obtaining Robot State
~~~~~~~~~~~~~~~~~~~~~~

Robot state can be obtained in two ways:

1. **Outside control loop:** Use :meth:`Robot.read_once`
2. **Inside control loop:** Use :meth:`ActiveControlBase.readOnce`

**Example:**

.. code-block:: python

    import pylibfranka

    robot = pylibfranka.Robot("172.16.0.2")
    
    # Method 1: Read once outside control loop
    state = robot.read_once()
    print("Joint positions:", state.q)
    
    # Method 2: Inside control loop
    control = robot.start_torque_control()
    state, duration = control.readOnce()

State Attributes
----------------

Joint State
~~~~~~~~~~~

.. py:attribute:: RobotState.q
   :type: numpy.ndarray
   :noindex:

   Measured joint positions [rad] (7,)
   
   **Example:**
   
   .. code-block:: python
   
       state = robot.read_once()
       print("Joint 1 position:", state.q[0], "rad")

.. py:attribute:: RobotState.q_d
   :type: numpy.ndarray
   :noindex:

   Desired joint positions [rad] (7,)
   
   Last commanded position from motion generator.

.. py:attribute:: RobotState.dq
   :type: numpy.ndarray
   :noindex:

   Measured joint velocities [rad/s] (7,)

.. py:attribute:: RobotState.dq_d
   :type: numpy.ndarray
   :noindex:

   Desired joint velocities [rad/s] (7,)

.. py:attribute:: RobotState.ddq_d
   :type: numpy.ndarray
   :noindex:

   Desired joint accelerations [rad/s²] (7,)

Joint Torques
~~~~~~~~~~~~~

.. py:attribute:: RobotState.tau_J
   :type: numpy.ndarray
   :noindex:

   Measured link-side joint torques [Nm] (7,)
   
   These are the torques measured at the joints including friction.

.. py:attribute:: RobotState.tau_J_d
   :type: numpy.ndarray
   :noindex:

   Desired link-side joint torques [Nm] (7,)
   
   Last commanded torques from controller.

.. py:attribute:: RobotState.dtau_J
   :type: numpy.ndarray
   :noindex:

   Derivative of measured link-side joint torques [Nm/s] (7,)

.. py:attribute:: RobotState.tau_ext_hat_filtered
   :type: numpy.ndarray
   :noindex:

   External torque estimate (filtered) [Nm] (7,)
   
   Estimated external torques acting on the robot, low-pass filtered.
   Useful for contact detection and force sensing.
   
   **Example:**
   
   .. code-block:: python
   
       state = robot.read_once()
       ext_torque = state.tau_ext_hat_filtered
       if np.linalg.norm(ext_torque) > 5.0:
           print("External contact detected!")

Motor State
~~~~~~~~~~~

.. py:attribute:: RobotState.theta
   :type: numpy.ndarray
   :noindex:

   Motor positions [rad] (7,)

.. py:attribute:: RobotState.dtheta
   :type: numpy.ndarray
   :noindex:

   Motor velocities [rad/s] (7,)

End Effector Poses
~~~~~~~~~~~~~~~~~~

.. py:attribute:: RobotState.O_T_EE
   :type: numpy.ndarray
   :noindex:

   End effector pose in base frame :math:`^OT_{EE}` (16,)
   
   Homogeneous transformation matrix (4×4) in column-major format.
   
   **Example:**
   
   .. code-block:: python
   
       import numpy as np
       
       state = robot.read_once()
       # Convert to 4×4 matrix
       O_T_EE = state.O_T_EE.reshape(4, 4).T
       
       position = O_T_EE[:3, 3]
       rotation = O_T_EE[:3, :3]
       
       print("End-effector position [m]:", position)
       print("End-effector rotation:\n", rotation)

.. py:attribute:: RobotState.O_T_EE_d
   :type: numpy.ndarray
   :noindex:

   Desired end effector pose in base frame :math:`^OT_{EE}^d` (16,)

.. py:attribute:: RobotState.O_T_EE_c
   :type: numpy.ndarray
   :noindex:

   Commanded end effector pose in base frame :math:`^OT_{EE}^c` (16,)

.. py:attribute:: RobotState.F_T_EE
   :type: numpy.ndarray
   :noindex:

   End effector pose in flange frame :math:`^FT_{EE}` (16,)
   
   Set via :meth:`Robot.set_EE`

.. py:attribute:: RobotState.F_T_NE
   :type: numpy.ndarray
   :noindex:

   Nominal end effector pose in flange frame :math:`^FT_{NE}` (16,)

.. py:attribute:: RobotState.NE_T_EE
   :type: numpy.ndarray
   :noindex:

   End effector pose in nominal end effector frame :math:`^{NE}T_{EE}` (16,)

.. py:attribute:: RobotState.EE_T_K
   :type: numpy.ndarray
   :noindex:

   Stiffness frame pose in end effector frame :math:`^{EE}T_K` (16,)
   
   Set via :meth:`Robot.set_K`

Inertial Properties
~~~~~~~~~~~~~~~~~~~

.. py:attribute:: RobotState.m_ee
   :type: float
   :noindex:

   End effector mass [kg]

.. py:attribute:: RobotState.I_ee
   :type: numpy.ndarray
   :noindex:

   End effector inertia tensor [kg·m²] (9,)
   
   Column-major 3×3 inertia matrix.

.. py:attribute:: RobotState.F_x_Cee
   :type: numpy.ndarray
   :noindex:

   Center of mass of end effector in flange frame [m] (3,)

.. py:attribute:: RobotState.m_load
   :type: float
   :noindex:

   External load mass [kg]
   
   Set via :meth:`Robot.set_load`

.. py:attribute:: RobotState.I_load
   :type: numpy.ndarray
   :noindex:

   External load inertia tensor [kg·m²] (9,)

.. py:attribute:: RobotState.F_x_Cload
   :type: numpy.ndarray
   :noindex:

   Center of mass of external load in flange frame [m] (3,)

.. py:attribute:: RobotState.m_total
   :type: float
   :noindex:

   Total mass (end effector + load) [kg]

.. py:attribute:: RobotState.I_total
   :type: numpy.ndarray
   :noindex:

   Total inertia tensor [kg·m²] (9,)

.. py:attribute:: RobotState.F_x_Ctotal
   :type: numpy.ndarray
   :noindex:

   Total center of mass in flange frame [m] (3,)

Elbow Configuration
~~~~~~~~~~~~~~~~~~~

.. py:attribute:: RobotState.elbow
   :type: numpy.ndarray
   :noindex:

   Current elbow configuration (2,)

.. py:attribute:: RobotState.elbow_d
   :type: numpy.ndarray
   :noindex:

   Desired elbow configuration (2,)

.. py:attribute:: RobotState.elbow_c
   :type: numpy.ndarray
   :noindex:

   Commanded elbow configuration (2,)

.. py:attribute:: RobotState.delbow_c
   :type: numpy.ndarray
   :noindex:

   Commanded elbow velocity (2,)

.. py:attribute:: RobotState.ddelbow_c
   :type: numpy.ndarray
   :noindex:

   Commanded elbow acceleration (2,)

Collision and Contact
~~~~~~~~~~~~~~~~~~~~~

.. py:attribute:: RobotState.joint_contact
   :type: numpy.ndarray
   :noindex:

   Joint-level contact detection flags (7,)
   
   True if contact detected at corresponding joint.

.. py:attribute:: RobotState.cartesian_contact
   :type: numpy.ndarray
   :noindex:

   Cartesian contact detection flags (6,)
   
   Contact in (x, y, z, roll, pitch, yaw).

.. py:attribute:: RobotState.joint_collision
   :type: numpy.ndarray
   :noindex:

   Joint-level collision detection flags (7,)

.. py:attribute:: RobotState.cartesian_collision
   :type: numpy.ndarray
   :noindex:

   Cartesian collision detection flags (6,)

External Forces
~~~~~~~~~~~~~~~

.. py:attribute:: RobotState.O_F_ext_hat_K
   :type: numpy.ndarray
   :noindex:

   External wrench in stiffness frame, expressed in base frame [N, Nm] (6,)
   
   Format: [fx, fy, fz, τx, τy, τz]

.. py:attribute:: RobotState.K_F_ext_hat_K
   :type: numpy.ndarray
   :noindex:

   External wrench in stiffness frame [N, Nm] (6,)

End Effector Velocity
~~~~~~~~~~~~~~~~~~~~~

.. py:attribute:: RobotState.O_dP_EE_d
   :type: numpy.ndarray
   :noindex:

   Desired end effector twist in base frame [m/s, rad/s] (6,)
   
   Format: [vx, vy, vz, ωx, ωy, ωz]

.. py:attribute:: RobotState.O_dP_EE_c
   :type: numpy.ndarray
   :noindex:

   Commanded end effector twist in base frame [m/s, rad/s] (6,)

.. py:attribute:: RobotState.O_ddP_EE_c
   :type: numpy.ndarray
   :noindex:

   Commanded end effector acceleration in base frame [m/s², rad/s²] (6,)

.. py:attribute:: RobotState.O_ddP_O
   :type: numpy.ndarray
   :noindex:

   Base acceleration [m/s², rad/s²] (6,)

Error and Mode
~~~~~~~~~~~~~~

.. py:attribute:: RobotState.current_errors
   :type: Errors
   :noindex:

   Current robot errors
   
   **Example:**
   
   .. code-block:: python
   
       state = robot.read_once()
       if state.current_errors:
           print("Active errors:", state.current_errors)

.. py:attribute:: RobotState.last_motion_errors
   :type: Errors
   :noindex:

   Errors from last motion

.. py:attribute:: RobotState.control_command_success_rate
   :type: float
   :noindex:

   Success rate of control commands [0.0, 1.0]
   
   Indicates the percentage of successfully received control commands.

.. py:attribute:: RobotState.robot_mode
   :type: RobotMode
   :noindex:

   Current robot operating mode

.. py:attribute:: RobotState.time
   :type: Duration
   :noindex:

   Timestamp of the state

Errors Class
------------

.. autoclass:: Errors
   :members:
   :undoc-members:

The :class:`Errors` class contains boolean flags for all possible robot errors.

Methods
~~~~~~~

.. py:method:: Errors.__bool__()

   Check if any error is present.
   
   :return: True if any error flag is set
   :rtype: bool
   
   **Example:**
   
   .. code-block:: python
   
       state = robot.read_once()
       if state.current_errors:
           print("Robot has errors!")
           print(state.current_errors)

.. py:method:: Errors.__str__()

   Get string representation of active errors.
   
   :return: Comma-separated list of active error names
   :rtype: str

Error Attributes
~~~~~~~~~~~~~~~~

The Errors class has numerous boolean attributes for different error conditions. Key attributes include:

* ``joint_position_limits_violation``
* ``cartesian_position_limits_violation``
* ``self_collision_avoidance_violation``
* ``joint_velocity_violation``
* ``cartesian_velocity_violation``
* ``force_control_safety_violation``
* ``joint_reflex``
* ``cartesian_reflex``
* ``communication_constraints_violation``
* ``power_limit_violation``
* ``instability_detected``

See the full list in the source code or via autocompletion.

**Example:**

.. code-block:: python

    state = robot.read_once()
    errors = state.current_errors
    
    if errors.joint_position_limits_violation:
        print("Joint position limit violated!")
    
    if errors.cartesian_reflex:
        print("Cartesian reflex triggered!")

RobotMode Enum
--------------

.. autoclass:: RobotMode
   :members:
   :undoc-members:

Operating mode of the robot.

Values
~~~~~~

* ``RobotMode.Other`` - Other/unknown mode
* ``RobotMode.Idle`` - Robot is idle
* ``RobotMode.Move`` - Robot is executing a motion
* ``RobotMode.Guiding`` - Robot is in guiding mode (hand-guided)
* ``RobotMode.Reflex`` - Robot triggered a reflex
* ``RobotMode.UserStopped`` - User stopped the robot
* ``RobotMode.AutomaticErrorRecovery`` - Automatic error recovery in progress

**Example:**

.. code-block:: python

    state = robot.read_once()
    
    if state.robot_mode == pylibfranka.RobotMode.Idle:
        print("Robot is idle")
    elif state.robot_mode == pylibfranka.RobotMode.Move:
        print("Robot is moving")

Duration Class
--------------

.. autoclass:: Duration
   :members:
   :undoc-members:

Time duration representation.

Methods
~~~~~~~

.. py:method:: Duration.to_sec()
   :noindex:

   Convert duration to seconds.
   
   :return: Duration in seconds as float
   :rtype: float

**Example:**

.. code-block:: python

    state = robot.read_once()
    time_sec = state.time.to_sec()
    print(f"Timestamp: {time_sec:.3f} s")

Complete Examples
-----------------

Example 1: Monitoring Robot State
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    import pylibfranka
    import numpy as np

    def main():
        robot = pylibfranka.Robot("172.16.0.2")
        
        for i in range(10):
            state = robot.read_once()
            
            print(f"\n=== Iteration {i+1} ===")
            print(f"Joint positions [rad]: {state.q}")
            print(f"Joint velocities [rad/s]: {state.dq}")
            print(f"Joint torques [Nm]: {state.tau_J}")
            print(f"External torques [Nm]: {state.tau_ext_hat_filtered}")
            
            # End-effector position
            O_T_EE = state.O_T_EE.reshape(4, 4).T
            position = O_T_EE[:3, 3]
            print(f"EE position [m]: {position}")
            
            # Robot mode
            print(f"Robot mode: {state.robot_mode}")
            
            # Errors
            if state.current_errors:
                print(f"Errors: {state.current_errors}")

    if __name__ == "__main__":
        main()


Example 2: Logging State Data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    import pylibfranka
    import numpy as np
    import csv

    def main():
        robot = pylibfranka.Robot("172.16.0.2")
        model = robot.load_model()
        
        # Prepare data logging
        data_log = []
        
        control = robot.start_torque_control()
        
        try:
            for i in range(1000):
                state, duration = control.readOnce()
                
                # Log state
                data_log.append({
                    'time': state.time.to_sec(),
                    'q': state.q.copy(),
                    'dq': state.dq.copy(),
                    'tau_J': state.tau_J.copy(),
                    'tau_ext': state.tau_ext_hat_filtered.copy()
                })
                
                # Control (gravity is automatically compensated)
                tau = pylibfranka.Torques([0, 0, 0, 0, 0, 0, 0])
                control.writeOnce(tau)
                
        finally:
            robot.stop()
        
        # Save to CSV
        with open('robot_data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7',
                           'tau1', 'tau2', 'tau3', 'tau4', 'tau5', 'tau6', 'tau7'])
            
            for data in data_log:
                row = [data['time']] + list(data['q']) + list(data['tau_J'])
                writer.writerow(row)
        
        print(f"Logged {len(data_log)} samples to robot_data.csv")

    if __name__ == "__main__":
        main()

See Also
--------

* :class:`Robot` -- Use :meth:`Robot.read_once` to get state
* :class:`ActiveControlBase` -- Use :meth:`ActiveControlBase.readOnce` in control loops
* :class:`Model` -- Use state for dynamics computations
