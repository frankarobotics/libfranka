from typing import List, Tuple

import numpy as np

from pylibfranka import JointPositions, Robot, RobotState


def setDefaultBehaviour(robot: Robot):
    # Set collision behavior
    lower_torque_thresholds = [20.0] * 7  # Nm
    upper_torque_thresholds = [40.0] * 7  # Nm
    lower_force_thresholds = [10.0] * 6  # N (linear) and Nm (angular)
    upper_force_thresholds = [20.0] * 6  # N (linear) and Nm (angular)

    robot.set_collision_behavior(
        lower_torque_thresholds,
        upper_torque_thresholds,
        lower_force_thresholds,
        upper_force_thresholds,
    )

    # Set joint impedance
    robot.set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])  # Nm/rad
    robot.set_cartesian_impedance([3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0])  # N/m and Nm/rad])


class MotionGenerator:
    def __init__(self, speed_factor: float, q_goal: List[float]):
        self.q_goal = np.array(q_goal)
        self.time = 0.0

        # Constants from C++ implementation
        self.dq_max = np.array([2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]) * speed_factor
        self.ddq_max_start = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]) * speed_factor
        self.ddq_max_goal = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]) * speed_factor

        # Initialize variables
        self.q_start = np.zeros(7)
        self.delta_q = np.zeros(7)
        self.dq_max_sync = np.zeros(7)
        self.t_1_sync = np.zeros(7)
        self.t_2_sync = np.zeros(7)
        self.t_f_sync = np.zeros(7)
        self.q_1 = np.zeros(7)

        self.delta_q_motion_finished = 1e-12
        self.initialized = False

    def calculate_synchronized_values(self):
        dq_max_reach = self.dq_max.copy()
        t_f = np.zeros(7)
        delta_t_2 = np.zeros(7)
        t_1 = np.zeros(7)
        delta_t_2_sync = np.zeros(7)
        sign_delta_q = np.sign(self.delta_q).astype(int)

        for i in range(7):
            if abs(self.delta_q[i]) > self.delta_q_motion_finished:
                # Calculate maximum reachable velocity
                if abs(self.delta_q[i]) < (
                    3.0 / 4.0 * (self.dq_max[i] ** 2 / self.ddq_max_start[i])
                    + 3.0 / 4.0 * (self.dq_max[i] ** 2 / self.ddq_max_goal[i])
                ):
                    dq_max_reach[i] = np.sqrt(
                        4.0
                        / 3.0
                        * self.delta_q[i]
                        * sign_delta_q[i]
                        * (self.ddq_max_start[i] * self.ddq_max_goal[i])
                        / (self.ddq_max_start[i] + self.ddq_max_goal[i])
                    )

                t_1[i] = 1.5 * dq_max_reach[i] / self.ddq_max_start[i]
                delta_t_2[i] = 1.5 * dq_max_reach[i] / self.ddq_max_goal[i]
                t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + abs(self.delta_q[i]) / dq_max_reach[i]

        max_t_f = np.max(t_f)

        for i in range(7):
            if abs(self.delta_q[i]) > self.delta_q_motion_finished:
                a = 1.5 / 2.0 * (self.ddq_max_goal[i] + self.ddq_max_start[i])
                b = -1.0 * max_t_f * self.ddq_max_goal[i] * self.ddq_max_start[i]
                c = abs(self.delta_q[i]) * self.ddq_max_goal[i] * self.ddq_max_start[i]

                delta = b * b - 4.0 * a * c
                if delta < 0.0:
                    delta = 0.0

                self.dq_max_sync[i] = (-1.0 * b - np.sqrt(delta)) / (2.0 * a)
                self.t_1_sync[i] = 1.5 * self.dq_max_sync[i] / self.ddq_max_start[i]
                delta_t_2_sync[i] = 1.5 * self.dq_max_sync[i] / self.ddq_max_goal[i]
                self.t_f_sync[i] = (
                    self.t_1_sync[i] / 2.0
                    + delta_t_2_sync[i] / 2.0
                    + abs(self.delta_q[i] / self.dq_max_sync[i])
                )
                self.t_2_sync[i] = self.t_f_sync[i] - delta_t_2_sync[i]
                self.q_1[i] = self.dq_max_sync[i] * sign_delta_q[i] * (0.5 * self.t_1_sync[i])

    def calculate_desired_values(self, t: float) -> Tuple[np.ndarray, bool]:
        delta_q_d = np.zeros(7)
        sign_delta_q = np.sign(self.delta_q).astype(int)
        t_d = self.t_2_sync - self.t_1_sync
        delta_t_2_sync = self.t_f_sync - self.t_2_sync
        joint_motion_finished = [False] * 7

        for i in range(7):
            if abs(self.delta_q[i]) < self.delta_q_motion_finished:
                delta_q_d[i] = 0
                joint_motion_finished[i] = True
            else:
                if t < self.t_1_sync[i]:
                    delta_q_d[i] = (
                        -1.0
                        / (self.t_1_sync[i] ** 3)
                        * self.dq_max_sync[i]
                        * sign_delta_q[i]
                        * (0.5 * t - self.t_1_sync[i])
                        * (t**3)
                    )
                elif t >= self.t_1_sync[i] and t < self.t_2_sync[i]:
                    delta_q_d[i] = (
                        self.q_1[i] + (t - self.t_1_sync[i]) * self.dq_max_sync[i] * sign_delta_q[i]
                    )
                elif t >= self.t_2_sync[i] and t < self.t_f_sync[i]:
                    delta_q_d[i] = (
                        self.delta_q[i]
                        + 0.5
                        * (
                            1.0
                            / (delta_t_2_sync[i] ** 3)
                            * (t - self.t_1_sync[i] - 2.0 * delta_t_2_sync[i] - t_d[i])
                            * ((t - self.t_1_sync[i] - t_d[i]) ** 3)
                            + (2.0 * t - 2.0 * self.t_1_sync[i] - delta_t_2_sync[i] - 2.0 * t_d[i])
                        )
                        * self.dq_max_sync[i]
                        * sign_delta_q[i]
                    )
                else:
                    # Hold the final value, but only declare finished if close enough
                    delta_q_d[i] = self.delta_q[i]
                    joint_motion_finished[i] = True

        return delta_q_d, all(joint_motion_finished)

    def __call__(self, robot_state: RobotState, duration_sec: float) -> JointPositions:
        self.time += duration_sec

        if not self.initialized:
            self.q_start = np.array(robot_state.q)
            self.delta_q = self.q_goal - self.q_start
            self.calculate_synchronized_values()
            self.initialized = True

        delta_q_d, motion_finished = self.calculate_desired_values(self.time)
        joint_positions = list(self.q_start + delta_q_d)

        output = JointPositions(joint_positions)
        output.motion_finished = motion_finished
        return output
