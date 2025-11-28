Minimum System and Network Requirements
=======================================

This page specifies the requirements for running the Franka Control Interface (FCI).
Additional requirements may be documented in the materials provided with your robot.

Workstation PC
--------------

+-------------------+--------------------------------------+
| Minimum Requirement| Specification                        |
+===================+======================================+
| Operating System   | Linux with PREEMPT_RT patched kernel |
+-------------------+--------------------------------------+
| Network card       | 100BASE-TX                           |
+-------------------+--------------------------------------+

Since the robot sends data at a 1 kHz frequency, it is important that the workstation PC is configured to minimize latencies. For example, we recommend to :ref:`disable CPU frequency scaling <disable_cpu_frequency_scaling>`. Other optimizations may depend on your particular system.

Network
-------

.. _requirement-network:

If possible, directly connect your workstation PC to the LAN port of **Control**, avoiding any intermediate devices such as switches.

.. important::
   The workstation PC controlling your robot using the FCI **must** be connected to the LAN port of Control (shop floor network) and **not** to the LAN port of the Arm (robot network).

Relays or switches in between may introduce delays, jitter, or packet loss, which can decrease controller performance or render it unusable.

.. hint::
    The best performance is achieved when connecting directly to the LAN port of Control.
    This requires setting up a static IP for the shop floor network in the administrator interface beforehand.
    See :ref:`setting-up-the-network`.

Time Constraints
----------------

To control the robot reliably, the sum of the following time measurements must be **less than 1 ms**:

* Round-trip time (RTT) between the workstation PC and FCI.
* Execution time of your motion generator or control loop.
* Time needed by the robot to process the data and update the internal controller.

.. caution::
    If the **<1 ms constraint** is violated for a cycle, the packet is dropped by FCI. After 20 consecutively dropped packets, the robot `will stop` with the ``communication_constraints_violation`` error.
    Communication quality can be monitored via the ``RobotState::control_command_success_rate`` field.

Packet Handling
---------------

* **Motion generator packet dropped**:
  The robot uses previous waypoints and performs a linear extrapolation (constant acceleration integration) for the missed time step. More than 20 consecutive dropped packets will cause the robot to `stop`.

* **Controller command packet dropped**:
  FCI reuses the torques from the last successfully received packet. Again, more than 20 consecutive dropped packets will cause the robot to `stop`.

.. hint::
    Measure network performance (see :ref:`network-bandwidth-delay-test`) and the control or motion generator loop beforehand to ensure compliance with timing requirements.
