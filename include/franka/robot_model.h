// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>

#include "franka/robot_model_base.h"

namespace franka {

/**
 * Optimized implementation of RobotModelBase using Pinocchio.
 */
class RobotModel : public RobotModelBase {
 public:
  RobotModel(const std::string& urdf);

  void coriolis(const std::array<double, 7>& q,
                const std::array<double, 7>& dq,
                const std::array<double, 9>& i_total,
                double m_total,
                const std::array<double, 3>& f_x_ctotal,
                std::array<double, 7>& c_ne) override;

  void coriolis(const std::array<double, 7>& q,
                const std::array<double, 7>& dq,
                const std::array<double, 9>& i_total,
                double m_total,
                const std::array<double, 3>& f_x_ctotal,
                const std::array<double, 3>& g_earth,
                std::array<double, 7>& c_ne) override;

  void gravity(const std::array<double, 7>& q,
               const std::array<double, 3>& g_earth,
               double m_total,
               const std::array<double, 3>& f_x_ctotal,
               std::array<double, 7>& g_ne) override;

  void mass(const std::array<double, 7>& q,
            const std::array<double, 9>& i_total,
            double m_total,
            const std::array<double, 3>& f_x_ctotal,
            std::array<double, 49>& m_ne) override;

 private:
  mutable pinocchio::Data data_;
  mutable pinocchio::Data data_gravity_;

  mutable Eigen::Matrix<double, 7, 1> q_eigen_;
  mutable Eigen::Matrix<double, 7, 1> dq_eigen_;
  mutable Eigen::Matrix<double, 7, 1> tau_eigen_;
  mutable Eigen::Matrix<double, 7, 1> ddq_temp_eigen_;
  mutable Eigen::Vector3d com_eigen_;
  mutable Eigen::Matrix3d inertia_eigen_;

  mutable std::array<double, 9> cached_i_total_;
  mutable double cached_m_total_;
  mutable std::array<double, 3> cached_f_x_ctotal_;
  mutable bool inertia_cache_valid_;

  mutable pinocchio::Model pinocchio_model_;
  pinocchio::Inertia initial_last_link_inertia_;
  pinocchio::FrameIndex last_link_frame_index_;
  pinocchio::JointIndex last_joint_index_;

  void updateInertiaIfNeeded(const std::array<double, 9>& i_total,
                             double m_total,
                             const std::array<double, 3>& f_x_ctotal) const;

  void restoreOriginalInertia() const;

  void computeGravityVector(const std::array<double, 7>& q,
                            const std::array<double, 3>& g_earth,
                            std::array<double, 7>& g_ne) const;

  void copyToEigenQ(const std::array<double, 7>& q) const;
  void copyToEigenDQ(const std::array<double, 7>& dq) const;
  void copyFromEigen(const Eigen::VectorXd& src, std::array<double, 7>& dst) const;
  void copyFromEigenMatrix(const Eigen::MatrixXd& src, std::array<double, 49>& dst) const;
};

}  // namespace franka
