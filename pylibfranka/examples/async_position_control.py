# Copyright (c) 2026 Franka Robotics GmbH
# Apache-2.0

# This example demonstrates asynchronous position control of a Franka robot using the
# pylibfranka library. It connects to the robot, sets up an asynchronous position control
# handler, and continuously updates joint position targets in a loop until interrupted, leveraging
# the latest low-rate control API.

import signal
import sys
import time
import math
import argparse
import threading
from datetime import timedelta

import pylibfranka as franka
from example_common import setDefaultBehaviour

kDefaultMaximumVelocities = [0.655, 0.655, 0.655, 0.655, 1.315, 1.315, 1.315]
kDefaultGoalTolerance = 10.0

motion_finished = False


def signal_handler(sig, frame):
    global motion_finished
    if sig == signal.SIGINT:
        motion_finished = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="localhost", help="Robot IP address")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        robot = franka.Robot(args.ip, franka.RealtimeConfig.kIgnore)
    except Exception as e:
        print(f"Could not connect to robot: {e}")
        sys.exit(-1)

    setDefaultBehaviour(robot)

    initial_position = [0,
                        -math.pi / 4,
                        0,
                        -3 * math.pi / 4,
                        0,
                        math.pi / 2,
                        math.pi / 4]

    time_elapsed = 0.0
    direction = 1.0
    time_since_last_log = 0.0

    def calculate_joint_position_target(period_sec):
        nonlocal time_elapsed, direction, time_since_last_log

        time_elapsed += period_sec

        target_positions = [
            initial_position[i] + direction * 0.25
            for i in range(7)
        ]

        time_since_last_log += period_sec
        if time_since_last_log >= 1.0:
            direction *= -1.0
            time_since_last_log = 0.0

        return franka.AsyncPositionControlHandler.JointPositionTarget(
            joint_positions=target_positions
        )

    joint_position_control_configuration = \
        franka.AsyncPositionControlHandler.Configuration(
            maximum_joint_velocities=kDefaultMaximumVelocities,
            goal_tolerance=kDefaultGoalTolerance
        )

    result = franka.AsyncPositionControlHandler.configure(robot,
                                                   joint_position_control_configuration)

    if result.error_message is not None:
        print(result.error_message)
        sys.exit(-1)

    position_control_handler = result.handler
    target_feedback = position_control_handler.get_target_feedback()

    time_step = 0.020  # 20 ms, 50 Hz

    global motion_finished
    while not motion_finished:
        loop_start = time.monotonic()

        target_feedback = position_control_handler.get_target_feedback()
        if target_feedback.error_message is not None:
            print(target_feedback.error_message)
            sys.exit(-1)

        next_target = calculate_joint_position_target(time_step)
        command_result = position_control_handler.set_joint_position_target(next_target)

        if command_result.error_message is not None:
            print(command_result.error_message)
            sys.exit(-1)

        if time_elapsed > 10.0:
            position_control_handler.stop_control()
            motion_finished = True
            print("Control finished")
            break

        sleep_time = time_step - (time.monotonic() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
