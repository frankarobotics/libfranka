#!/usr/bin/env python3

# Copyright (c) 2025 Franka Robotics GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import argparse
import time

import numpy as np

from pylibfranka import Robot


def print_robot_state(state):
    """Print critical properties of the robot state"""
    print("Robot State (Critical Attributes):")

    # Robot mode and status
    try:
        mode = state.robot_mode
        mode_str = str(mode).split(".")[-1]  # Get just the enum value name
        print(f"  Robot Mode: {mode_str}")
    except (AttributeError, ValueError):
        print("  Robot Mode: <not available>")

    # Joint positions and velocities
    try:
        print(f"  Joint Positions (q): {np.round(state.q, 4).tolist()}")
    except (AttributeError, TypeError):
        print("  Joint Positions (q): <not available>")

    try:
        print(f"  Joint Velocities (dq): {np.round(state.dq, 4).tolist()}")
    except (AttributeError, TypeError):
        print("  Joint Velocities (dq): <not available>")

    # End effector position (first 3 elements of the transformation matrix)
    print("  End Effector:")
    try:
        position = [state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]]
        print(f"    Position: {np.round(position, 4).tolist()}")
    except (AttributeError, TypeError, IndexError):
        print("    Position: <not available>")

    # External torques and forces
    try:
        print(f"  External Joint Torques: {np.round(state.tau_ext_hat_filtered, 4).tolist()}")
    except (AttributeError, TypeError):
        print("  External Joint Torques: <not available>")

    try:
        print(f"  External Wrench (base frame): {np.round(state.O_F_ext_hat_K, 4).tolist()}")
    except (AttributeError, TypeError):
        print("  External Wrench (base frame): <not available>")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Print Franka robot state")
    parser.add_argument("--ip", type=str, default="localhost", help="Robot IP address")
    parser.add_argument(
        "--rate", type=float, default=0.5, help="Rate at which to print state (in Hz)"
    )
    parser.add_argument(
        "--count", type=int, default=1, help="Number of state readings to print (0 for continuous)"
    )
    args = parser.parse_args()

    # Connect to robot
    print(f"Connecting to robot at {args.ip}...")
    robot = Robot(args.ip)

    try:
        # Set collision behavior (using default values from active_control_example.py)
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

        print("Connected to robot. Reading state...")

        count = 0
        while args.count == 0 or count < args.count:
            # Read robot state
            state = robot.read_once()

            # Print the robot state
            print("\n" + "=" * 80)
            print(f"Robot State Reading #{count+1}")
            print("=" * 80)
            print_robot_state(state)

            count += 1

            # Sleep if we're continuing
            if args.count == 0 or count < args.count:
                time.sleep(1.0 / args.rate)

    except Exception as e:
        print(f"Error occurred: {e}")


if __name__ == "__main__":
    main()
