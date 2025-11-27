#!/usr/bin/env python3

# Copyright (c) 2025 Franka Robotics GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import argparse

import numpy as np
from example_common import MotionGenerator, setDefaultBehaviour

from pylibfranka import CartesianVelocities, ControllerMode, Robot


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="localhost", help="Robot IP address")
    args = parser.parse_args()

    # Connect to robot
    robot = Robot(args.ip)
    robot.automatic_error_recovery()

    print("WARNING: This example will move the robot!")
    print("Please make sure to have the user stop button at hand!")
    input("Press Enter to continue...")

    try:
        setDefaultBehaviour(robot)

        # First move the robot to a suitable joint configuration
        q_goal = [0.0, np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4]

        # Create motion generator
        motion_generator = MotionGenerator(speed_factor=0.5, q_goal=q_goal)
        # First move the robot to a suitable joint configuration

        control = robot.start_joint_position_control(ControllerMode.CartesianImpedance)
        while True:
            state, duration = control.readOnce()  # Properly unpack the tuple
            joint_positions = motion_generator(state, duration.to_sec())

            control.writeOnce(joint_positions)
            if joint_positions.motion_finished:
                robot.stop()
                break

    except KeyboardInterrupt:
        print("\nMotion interrupted by user")

    print("Finished moving to initial joint configuration.")

    # Set additional parameters always before the control loop, NEVER in the control loop!
    # Set the joint impedance.
    robot.set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])  # Nm/rad

    try:
        # Set collision behavior
        lower_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        upper_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        lower_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        upper_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

        robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds,
            lower_force_thresholds,
            upper_force_thresholds,
        )

        # Start joint velocity control with external control loop
        active_control = robot.start_cartesian_velocity_control(ControllerMode.CartesianImpedance)

        time_max = 4.0
        v_max = 0.1
        angle = np.pi / 4.0
        time_elapsed = 0.0
        motion_finished = False

        # External control loop
        while not motion_finished:
            # Read robot state and duration
            robot_state, duration = active_control.readOnce()

            # Update time
            time_elapsed += duration.to_sec()

            # Calculate omega using the same formula as in C++ example
            cycle = np.floor(
                np.power(-1.0, (time_elapsed - np.fmod(time_elapsed, time_max)) / time_max)
            )
            v = cycle * v_max / 2.0 * (1.0 - np.cos(2.0 * np.pi / time_max * time_elapsed))

            v_x = np.cos(angle) * v
            v_z = np.sin(angle) * v

            # Update joint positions
            velocities = [
                v_x,
                0.0,
                v_z,
                0.0,
                0.0,
                0.0,
            ]

            # Set joint positions
            cartesian_velocities = CartesianVelocities(velocities)

            # Set motion_finished flag to True on the last update
            if time_elapsed >= 2.0 * time_max:
                cartesian_velocities.motion_finished = True
                motion_finished = True
                print("Finished motion, shutting down example")

            # Send command to robot
            active_control.writeOnce(cartesian_velocities)

    except KeyboardInterrupt:
        print("\nMotion interrupted by user")

    except Exception as e:
        print(f"Error occurred: {e}")
        if robot is not None:
            robot.stop()
        return -1


if __name__ == "__main__":
    main()
