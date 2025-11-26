#!/usr/bin/env python3

# Copyright (c) 2025 Franka Robotics GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import argparse

import numpy as np

from pylibfranka import ControllerMode, JointVelocities, Robot


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="localhost", help="Robot IP address")
    args = parser.parse_args()

    # Connect to robot
    robot = Robot(args.ip)

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

        # First move the robot to a suitable joint configuration
        print("WARNING: This example will move the robot!")
        print("Please make sure to have the user stop button at hand!")
        input("Press Enter to continue...")

        # Start joint velocity control with external control loop
        active_control = robot.start_joint_velocity_control(ControllerMode.CartesianImpedance)

        time_max = 1.0
        omega_max = 1.0
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
            omega = cycle * omega_max / 2.0 * (1.0 - np.cos(2.0 * np.pi / time_max * time_elapsed))

            # Update joint positions
            velocities = [
                0.0,
                0.0,
                0.0,
                omega,
                omega,
                omega,
                omega,
            ]

            # Set joint positions
            joint_velocities = JointVelocities(velocities)

            # Set motion_finished flag to True on the last update
            if time_elapsed >= 2.0 * time_max:
                joint_velocities.motion_finished = True
                motion_finished = True
                print("Finished motion, shutting down example")

            # Send command to robot
            active_control.writeOnce(joint_velocities)

    except Exception as e:
        print(f"Error occurred: {e}")
        if robot is not None:
            robot.stop()
        return -1


if __name__ == "__main__":
    main()
