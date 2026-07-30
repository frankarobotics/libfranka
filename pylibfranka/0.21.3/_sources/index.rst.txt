pylibfranka Documentation
==========================

Overview
--------

**pylibfranka** is a Python library that provides high-level access to Franka Robotics robots,
enabling real-time control with torque, position, and velocity commands.

Key Features
~~~~~~~~~~~~

* **Real-time Control**: Send torque, position, and velocity commands at 1 kHz
* **Multiple Control Modes**: Joint/Cartesian position, velocity, and torque control
* **Robot State Access**: Read all robot state information including joint positions, velocities, torques
* **Dynamics Model**: Compute forward/inverse kinematics, mass matrix, Coriolis, and gravity
* **Gripper Control**: Full control of Franka Hand gripper
* **NumPy Integration**: All arrays are NumPy arrays for seamless integration

Documentation Structure
-----------------------

.. toctree::
   :maxdepth: 1
   :caption: Quick Reference

   api_overview
   class_list
   class_hierarchy
   modules

.. toctree::
   :maxdepth: 2
   :caption: Detailed API Reference
   
   api/robot
   api/control
   api/model
   api/gripper
   api/state
   api/exceptions

Indices and Tables
==================

* :ref:`genindex` - Complete index of all classes and methods
* :ref:`modindex` - Module index
* :ref:`search` - Search the documentation