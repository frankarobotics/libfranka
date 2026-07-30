Exceptions
==========

.. currentmodule:: pylibfranka

Exception hierarchy for error handling in pylibfranka.

Exception Hierarchy
-------------------

All pylibfranka exceptions inherit from :class:`FrankaException`:

.. code-block:: text

    FrankaException (base)
    ├── CommandException
    ├── NetworkException
    ├── ControlException
    ├── InvalidOperationException
    └── RealtimeException

Exception Classes
-----------------

FrankaException
~~~~~~~~~~~~~~~

.. autoexception:: FrankaException
   :show-inheritance:

Base exception for all Franka-related errors.

All exceptions in pylibfranka inherit from this base class, allowing you to catch all library-specific exceptions with a single except clause.

**Example:**

.. code-block:: python

    import pylibfranka

    try:
        robot = pylibfranka.Robot("172.16.0.2")
        control = robot.start_torque_control()
        # ... control code ...
    except pylibfranka.FrankaException as e:
        print(f"Franka error occurred: {e}")

CommandException
~~~~~~~~~~~~~~~~

.. autoexception:: CommandException
   :show-inheritance:

Thrown if an error occurs during command execution.

This exception is raised when:

* A robot command (e.g., :meth:`Robot.set_collision_behavior`) fails
* Configuration changes are rejected by the robot
* Commands contain invalid parameters

NetworkException
~~~~~~~~~~~~~~~~

.. autoexception:: NetworkException
   :show-inheritance:

Thrown if a connection to the robot cannot be established, or when a timeout occurs.

This exception is raised when:

* Initial connection to the robot fails
* Network connection is lost during operation
* Communication timeout occurs
* Robot is not reachable on the network


**Handling in control loop:**

.. code-block:: python

    import pylibfranka

    robot = pylibfranka.Robot("172.16.0.2")
    
    try:
        state = robot.read_once()
    except pylibfranka.NetworkException as e:
        print(f"Lost connection: {e}")
        # Attempt reconnection...

ControlException
~~~~~~~~~~~~~~~~

.. autoexception:: ControlException
   :show-inheritance:

Thrown if an error occurs during motion generation or torque control.

This exception is raised when:

* Motion generator detects discontinuities
* Joint limits are violated
* Cartesian limits are exceeded
* Safety thresholds are triggered
* Commanded values cause instability

**Common causes:**

* Velocity discontinuities in commands
* Acceleration discontinuities
* Torque discontinuities
* Collision detected
* Position limits exceeded


InvalidOperationException
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoexception:: InvalidOperationException
   :show-inheritance:

Thrown if an operation cannot be performed.

This exception is raised when:

* Attempting to start a control loop while one is already running
* Calling methods in an invalid state
* Conflicting operations are requested

RealtimeException
~~~~~~~~~~~~~~~~~

.. autoexception:: RealtimeException
   :show-inheritance:

Thrown if real-time scheduling requirements cannot be met.

This exception is raised when:

* Real-time scheduling cannot be enabled (Linux RT kernel required)
* Real-time priority cannot be set
* RealtimeConfig.kEnforce is used without proper permissions

See Also
--------

* :class:`Robot` -- Robot control interface that may raise exceptions
* :class:`RobotState` -- Use ``current_errors`` and ``last_motion_errors`` attributes
* :class:`Errors` -- Detailed error information
