#!/usr/bin/env python3

# Copyright (c) 2025 Franka Robotics GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import argparse
import time

from pylibfranka import Gripper


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, required=True, help="Robot IP address")
    parser.add_argument("--width", type=float, default=0.005, help="Object width to grasp")
    parser.add_argument(
        "--homing", type=int, default=1, choices=[0, 1], help="Perform homing (0 or 1)"
    )
    parser.add_argument("--speed", type=float, default=0.1, help="Gripper speed")
    parser.add_argument("--force", type=float, default=60, help="Gripper force")
    args = parser.parse_args()

    try:
        # Connect to gripper
        gripper = Gripper(args.ip)
        grasping_width = args.width

        if args.homing:
            # Do a homing in order to estimate the maximum grasping width with the current fingers
            print("Homing gripper")
            gripper.homing()

        time.sleep(2.0)

        # Check for the maximum grasping width
        gripper_state = gripper.read_once()
        print(f"Gripper width: {gripper_state.width}")
        print(f"Gripper is grasped: {gripper_state.is_grasped}")
        print(f"Gripper temperature: {gripper_state.temperature}")
        print(f"Gripper time: {gripper_state.time.to_sec()}")

        # Grasp the object
        if not gripper.grasp(grasping_width, args.speed, args.force):
            print("Failed to grasp object.")
            return -1

        # Wait 3s and check afterwards, if the object is still grasped
        time.sleep(3.0)

        gripper_state = gripper.read_once()
        if not gripper_state.is_grasped:
            print("Object lost.")
            return -1

        print("Grasped object, will release it now.")
        gripper.stop()

    except Exception as e:
        print(f"Error occurred: {e}")
        return -1

    return 0


if __name__ == "__main__":
    main()
