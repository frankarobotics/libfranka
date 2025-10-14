#!/usr/bin/env python3

# Copyright (c) 2025 Franka Robotics GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import argparse
import time

import numpy as np

from pylibfranka import CartesianPose, ControllerMode, RealtimeConfig, Robot


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="localhost", help="Robot IP address")
    args = parser.parse_args()

    # Connect to robot
    robot = Robot(args.ip, RealtimeConfig.kIgnore)

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

        # Start cartesian pose control with external control loop
        active_control = robot.start_cartesian_pose_control(ControllerMode.JointImpedance)

        time_elapsed = 0.0
        motion_finished = False

        robot_state, duration = active_control.readOnce()
        initial_cartesian_pose = robot_state.O_T_EE

        # External control loop
        while not motion_finished:
            # Read robot state and duration
            robot_state, duration = active_control.readOnce()

            kRadius = 0.3
            angle = np.pi / 4 * (1 - np.cos(np.pi / 5.0 * time_elapsed))
            delta_x = kRadius * np.sin(angle)
            delta_z = kRadius * (np.cos(angle) - 1)

            # Update time
            time_elapsed += duration.to_sec()

            # Update joint positions
            new_cartesian_pose = initial_cartesian_pose.copy()
            new_cartesian_pose[12] += delta_x  # x position
            new_cartesian_pose[14] += delta_z  # z position

            # Set joint positions
            cartesian_pose = CartesianPose(new_cartesian_pose)

            # Set motion_finished flag to True on the last update
            if time_elapsed >= 5.0:
                cartesian_pose.motion_finished = True
                motion_finished = True
                print("Finished motion, shutting down example")

            # Send command to robot
            active_control.writeOnce(cartesian_pose)

    except Exception as e:
        print(f"Error occurred: {e}")
        if robot is not None:
            robot.stop()
        return -1


if __name__ == "__main__":
    main()
