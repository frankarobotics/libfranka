// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "examples_common.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>

void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7>& q_goal)
    : q_goal_(q_goal.data()) {
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  dq_max_sync_.setZero();
  q_start_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double time, Vector7d* delta_q_d) const {
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();
  Vector7d t_d = t_2_sync_ - t_1_sync_;
  Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 7> joint_motion_finished{};

  for (Eigen::Index i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (time < t_1_sync_[i]) {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                          (0.5 * time - t_1_sync_[i]) * std::pow(time, 3.0);
      } else if (time >= t_1_sync_[i] && time < t_2_sync_[i]) {
        (*delta_q_d)[i] = q_1_[i] + (time - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      } else if (time >= t_2_sync_[i] && time < t_f_sync_[i]) {
        (*delta_q_d)[i] =
            delta_q_[i] +
            0.5 *
                (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                     (time - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                     std::pow((time - t_1_sync_[i] - t_d[i]), 3.0) +
                 (2.0 * time - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                dq_max_sync_[i] * sign_delta_q[i];
      } else {
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool is_finished) { return is_finished; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector7d dq_max_reach(dq_max_);
  Vector7d t_f = Vector7d::Zero();
  Vector7d delta_t_2 = Vector7d::Zero();
  Vector7d t_1 = Vector7d::Zero();
  Vector7d delta_t_2_sync = Vector7d::Zero();
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  for (Eigen::Index i = 0; i < 7U; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (Eigen::Index i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);            // NOLINT
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];         // NOLINT
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];  // NOLINT
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    q_start_ = Vector7d(robot_state.q.data());
    delta_q_ = q_goal_ - q_start_;
    calculateSynchronizedValues();
  }

  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(joint_positions.data(), 7) = (q_start_ + delta_q_d);
  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}
