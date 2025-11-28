Minimum system and network requirements
=======================================

This page only specifies the requirements for running the Franka Control Interface (FCI).
Additional requirements are specified in the documents that you have received with your robot.

Workstation PC
--------------

+---------------------------------------------------------+
| Minimum System Requirements                             |
+===================+=====================================+
| Operating System  | Linux with PREEMPT_RT patched kernel|
+-------------------+-------------------------------------+
| Network card      | 100BASE-TX                          |
+-------------------+-------------------------------------+

Since the robot sends data at 1 kHz frequency, it is important that the workstation PC is configured
to minimize latencies. For example, we recommend to
:ref:`disable CPU frequency scaling <disable_cpu_frequency_scaling>`. Other possible optimizations
will depend on your particular system.

.. _requirement-network:

Network
-------
If possible, directly connect your workstation PC to the LAN port of Control, i.e. avoid any
intermediate devices such as switches.

.. important::
   The workstation PC which commands your robot using the FCI must always be connected to the LAN
   port of Control (shop floor network) and **not** to the LAN port of the Arm (robot network).

Having relays in between could lead to delay, jitter or packet loss. This will decrease the
performance of your controller or make it unusable.

.. hint::
    The best performance can be achieved when connecting directly to the LAN port of Control.
    This requires setting up a static IP for the shop floor network in the administrator's interface
    beforehand. See :ref:`setting-up-the-network`.

To control the robot, it must be guaranteed that the sum of the following time measurements is
less than 1 ms:

 * Round trip time (RTT) between the workstation PC and FCI.
 * Execution time of your motion generator or control loop.
 * Time needed by the robot to process your data and step the internal controller.

.. caution::
    If the **<1 ms constraint** is violated for a cycle, the received packet is dropped by
    FCI. After 20 consecutively dropped packets, your robot `will stop` with the
    ``communication_constraints_violation`` error. Current measure of communication quality
    can be read from the  ``RobotState::control_command_success_rate`` field.

If a **motion generator command packet is dropped**, the robot takes the previous waypoints and
performs a linear extrapolation (keep acceleration constant and integrate) for the missed
time step. If more than 20 packets are lost or dropped in a row, your robot `will stop`.

If a **controller command packet is dropped**, FCI will reuse the torques of the last successful
received packet. Again, more than 20 consecutive lost or dropped packets will cause your robot to
`stop`.

.. hint::
    Measure the performance of your network (see :ref:`network-bandwidth-delay-test`) and the
    control or motion generator loop beforehand.
