.. _fci_setup_overview:

Setting Up the Robot System and Network for FCI
===============================================

After completing the :ref:`real-time kernel setup <realtime_kernel_setup>`, ensuring proper network configuration (see :ref:`Network requirements <requirement-network>`), and installing :ref:`libfranka <libfranka_installation>`, you can connect to the robot and verify the setup by using FCI to read the current robot state.

.. _operating_the_robot:

Operating the Robot
------------------

Before proceeding, review the following safety considerations:

1. Ensure the Arm is mounted on a stable base that cannot topple over, even during fast motions or abrupt stops.

.. caution::
   Only tabletop mounting is supported (the Arm must be perpendicular to the ground).
   Other mountings **void the warranty** and **may damage the robot**.

2. Verify that the cable connecting Arm and Control is firmly attached on both sides.
3. Connect the external activation device to the Arm's base and keep it accessible to stop the robot at any time.

.. hint::
   Activating the external activation device will disconnect the Arm from Control.
   The joint motor controllers will hold their current position.
   **The external activation device is not an emergency stop.**

This list is non-exhaustive. The robot manual contains a dedicated chapter on safety and is available via the
`Franka World Hub <https://world.franka.de/resources>`_.

.. important::
   The workstation PC controlling the robot with FCI must be connected to the LAN port of Control
   (shop floor network), **not** the LAN port of the Arm (robot network).

.. _installing_fci_feature:

Installing the FCI Feature
--------------------------

To operate the robot through the Franka Control Interface, the corresponding Feature must be installed.
Check Desk --> Settings --> System --> Installed Features.

If FCI is not installed, synchronize it from your Franka World account to your controller.
You must have an FCI license and a registered controller in your account.
Refer to the `Franka World user manual <https://download.franka.de/franka-world-manual/>`_ for details.

.. _network_setup:

Setting Up the Network
---------------------

Good network performance is critical when using FCI.
A direct connection between the workstation PC and Franka's Control is recommended.

.. figure:: assets/control.png
    :align: center
    :figclass: align-center

    Use Control's LAN port when controlling the robot through FCI.
    Do not connect to the port in the Arm's base.

The Control and workstation must appear on the same network.
For this tutorial, the following static IP addresses are used:

+---------+----------------+------------+
|         | Workstation PC | Control    |
+=========+================+============+
| Address | 172.16.0.1     | 172.16.0.2 |
+---------+----------------+------------+
| Netmask | 24             | 24         |
+---------+----------------+------------+

The Control's address (172.16.0.2) is referred to as ``<fci-ip>`` in the following sections.

.. hint::
    Desk can be accessed via ``https://<fci-ip>``, though your browser may display a certificate warning.

The configuration consists of two steps:

* Configuring Control's network settings.
* Configuring your workstation's network settings.

.. _control_network_config:

Control Network Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before configuring, ensure the robot is installed and tested according to the documents shipped with it.

For system version >= 5.5.0, configure Control's network via the settings interface.
You may temporarily connect through the Arm's base port.
Refer to `Connecting a user interface device` in the robot manual.

.. figure:: assets/FrankaUI_Settings_Bar.png
    :align: center
    :figclass: align-center

    Access the Franka settings interface through Desk.

.. figure:: assets/FrankaUI_Settings_Network.png
    :align: center
    :figclass: align-center

    Set a static IP for the Control's LAN port (Shop Floor network).
    DHCP Client option is deselected.

After applying the settings, connect the workstation LAN port to the Control unit.

.. _linux_network_config:

Linux Workstation Network Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On Ubuntu 16.04 (GUI):

1. Open the Network Connection widget, select the wired connection, and click **Edit**.

.. figure:: assets/edit-connections.png
    :align: center
    :figclass: align-center

    Edit the connection in the Ethernet section.

2. In the IPv4 settings tab, set the method to **Manual** and enter the values:

.. figure:: assets/static-ip-ubuntu.png
    :align: center
    :figclass: align-center

    Set a static IP for the workstation PC.

.. hint::
   This disables DHCP. When no longer using FCI, revert the Method to `Automatic (DHCP)`.

3. Save changes and reconnect. Test connectivity using :ref:`network-bandwidth-delay-test`.
   Desk can now be accessed via the assigned IP in your browser.

.. _preparing_robot_in_desk:

Preparing the Robot for FCI in Desk
-----------------------------------

Ensure the robot's brakes are unlocked and the activation device released.
The robot is ready when the blue LED is active.

.. _activating_fci_mode:

Activating FCI Mode
^^^^^^^^^^^^^^^^^^

In Desk:

1. Release the robot brakes.
2. Expand the top menu and click **Activate FCI**.

.. figure:: assets/FrankaUI_System_ActivateFCI.png
    :align: center
    :figclass: align-center
    :width: 300px

    Activating FCI mode in Desk (system version >= 5.5.0).

.. _fci_mode_fer:

FCI Mode in Franka Emika Robot (FER)
""""""""""""""""""""""""""""""""""""

Once FCI mode is activated, a pop-up confirms the mode is active and Desk interactions are disabled.
The pop-up must remain open while working with FCI.

.. figure:: assets/pop_up_fci.png
    :align: center
    :figclass: align-center

    Pop-up when FCI mode is active.

.. _fci_mode_fr3:

FCI Mode in Franka Research 3
"""""""""""""""""""""""""""""

.. figure:: assets/FrankaUI_System_ActivateFCI_Done.png
    :align: center
    :figclass: align-center
    :scale: 35%

After activation, a pop-up appears requiring user confirmation.

.. _verifying_fci_connection:

Verifying the Connection
-----------------------

The IP of the Control's LAN port is referred to as ``<fci-ip>`` in the following examples.

Ensure :ref:`the robot is prepared for FCI usage in Desk <preparing_robot_in_desk>`.
Run the ``echo_robot_state`` example from ``libfranka``:

.. code-block:: shell

    ./examples/echo_robot_state <fci-ip>

The program prints the current robot state to the console. Refer to :api:`RobotState|structfranka_1_1RobotState.html` for field explanations.

.. code-block:: json

    {
      "O_T_EE": [0.998578,0.0328747,-0.0417381,0,0.0335224,-0.999317,0.0149157,0,-0.04122,-0.016294,
                 -0.999017,0,0.305468,-0.00814133,0.483198,1],
      "O_T_EE_d": [0.998582,0.0329548,-0.041575,0,0.0336027,-0.999313,0.0149824,0,-0.0410535,
                   -0.0163585,-0.999023,0,0.305444,-0.00810967,0.483251,1],
      "F_T_EE": [0.7071,-0.7071,0,0,0.7071,0.7071,0,0,0,0,1,0,0,0,0.1034,1],
      "EE_T_K": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
      "m_ee": 0.73, "F_x_Cee": [-0.01,0,0.03], "I_ee": [0.001,0,0,0,0.0025,0,0,0,0.0017],
      ...
    }

.. hint::

    If an error occurs, perform the :ref:`ping test <troubleshooting_robot_not_reachable>` and ensure
    the fail-safe locking system is open. Refer to the robot manual for additional instructions.
