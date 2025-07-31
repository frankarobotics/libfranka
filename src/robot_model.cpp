// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka/robot_model.h"
#include <algorithm>

namespace franka {

RobotModel::RobotModel(const std::string& urdf) : inertia_cache_valid_(false) {
  pinocchio::urdf::buildModelFromXML(urdf, pinocchio_model_);

  data_ = pinocchio::Data(pinocchio_model_);
  data_gravity_ = pinocchio::Data(pinocchio_model_);

  last_joint_index_ = pinocchio_model_.joints.back().id();
  last_link_frame_index_ = pinocchio_model_.getFrameId(pinocchio_model_.frames.back().name);
  initial_last_link_inertia_ = pinocchio_model_.inertias[last_joint_index_];

  cached_i_total_.fill(-1.0);
  cached_f_x_ctotal_.fill(-1.0);
  cached_m_total_ = -1.0;
}

void RobotModel::copyToEigenQ(const std::array<double, 7>& q) const {
  q_eigen_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q.data());
}

void RobotModel::copyToEigenDQ(const std::array<double, 7>& dq) const {
  dq_eigen_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(dq.data());
}

void RobotModel::copyFromEigen(const Eigen::VectorXd& src, std::array<double, 7>& dst) const {
  std::copy(src.data(), src.data() + 7, dst.begin());
}

void RobotModel::copyFromEigenMatrix(const Eigen::MatrixXd& src,
                                     std::array<double, 49>& dst) const {
  std::copy(src.data(), src.data() + 49, dst.begin());
}

void RobotModel::updateInertiaIfNeeded(const std::array<double, 9>& i_total,
                                       double m_total,
                                       const std::array<double, 3>& f_x_ctotal) const {
  if (inertia_cache_valid_ && cached_m_total_ == m_total &&
      std::equal(cached_i_total_.data(), cached_i_total_.data() + 9, i_total.data()) &&
      std::equal(cached_f_x_ctotal_.data(), cached_f_x_ctotal_.data() + 3, f_x_ctotal.data())) {
    return;  // No change needed
  }

  cached_i_total_ = i_total;
  cached_m_total_ = m_total;
  cached_f_x_ctotal_ = f_x_ctotal;

  inertia_eigen_ = Eigen::Map<const Eigen::Matrix3d>(i_total.data(), 3, 3);
  com_eigen_ = Eigen::Map<const Eigen::Vector3d>(f_x_ctotal.data());

  pinocchio::Inertia inertia(m_total, com_eigen_, inertia_eigen_);

  pinocchio_model_.inertias[last_joint_index_] =
      initial_last_link_inertia_ +
      pinocchio_model_.frames[last_link_frame_index_].placement.act(inertia);

  inertia_cache_valid_ = true;
}

void RobotModel::restoreOriginalInertia() const {
  if (inertia_cache_valid_) {
    pinocchio_model_.inertias[last_joint_index_] = initial_last_link_inertia_;
    inertia_cache_valid_ = false;
  }
}

void RobotModel::mass(const std::array<double, 7>& q,
                      const std::array<double, 9>& i_total,
                      double m_total,
                      const std::array<double, 3>& f_x_ctotal,
                      std::array<double, 49>& m_ne) {
  updateInertiaIfNeeded(i_total, m_total, f_x_ctotal);

  copyToEigenQ(q);

  pinocchio::crba(pinocchio_model_, data_, q_eigen_);

  data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.transpose().triangularView<Eigen::StrictlyLower>();

  copyFromEigenMatrix(data_.M, m_ne);
}

void RobotModel::coriolis(const std::array<double, 7>& q,
                          const std::array<double, 7>& dq,
                          const std::array<double, 9>& i_total,
                          double m_total,
                          const std::array<double, 3>& f_x_ctotal,
                          std::array<double, 7>& c_ne) {
  // DEPRECATED: Use coriolis() with gravity parameter for better performance and configurability

  updateInertiaIfNeeded(i_total, m_total, f_x_ctotal);

  std::array<double, 3> earth_gravity = {{0.0, 0.0, -9.81}};
  pinocchio_model_.gravity.linear(Eigen::Map<const Eigen::Vector3d>(earth_gravity.data()));

  copyToEigenQ(q);
  copyToEigenDQ(dq);

  pinocchio::computeCoriolisMatrix(pinocchio_model_, data_, q_eigen_, dq_eigen_);
  tau_eigen_.noalias() = data_.C * dq_eigen_;

  copyFromEigen(tau_eigen_, c_ne);
}

void RobotModel::coriolis(const std::array<double, 7>& q,
                          const std::array<double, 7>& dq,
                          const std::array<double, 9>& i_total,
                          double m_total,
                          const std::array<double, 3>& f_x_ctotal,
                          const std::array<double, 3>& g_earth,
                          std::array<double, 7>& c_ne) {
  updateInertiaIfNeeded(i_total, m_total, f_x_ctotal);

  // CRITICAL: Set gravity vector BEFORE computing RNEA to ensure consistency
  pinocchio_model_.gravity.linear(Eigen::Map<const Eigen::Vector3d>(g_earth.data()));

  copyToEigenQ(q);
  copyToEigenDQ(dq);

  // Compute Coriolis forces directly using RNEA with zero acceleration
  ddq_temp_eigen_.setZero();
  pinocchio::rnea(pinocchio_model_, data_, q_eigen_, dq_eigen_, ddq_temp_eigen_);

  // The result is in data_.tau, but we need to subtract gravity
  // Compute gravity using the same gravity vector (already set above)
  pinocchio::computeGeneralizedGravity(pinocchio_model_, data_gravity_, q_eigen_);

  // Coriolis = RNEA(q, dq, 0) - gravity (both computed with same gravity vector)
  tau_eigen_.noalias() = data_.tau - data_gravity_.g;
  copyFromEigen(tau_eigen_, c_ne);
}

void RobotModel::gravity(const std::array<double, 7>& q,
                         const std::array<double, 3>& g_earth,
                         double m_total,
                         const std::array<double, 3>& f_x_ctotal,
                         std::array<double, 7>& g_ne) {
  if (m_total > 0.0) {
    updateInertiaIfNeeded(std::array<double, 9>{0, 0, 0, 0, 0, 0, 0, 0, 0}, m_total, f_x_ctotal);
  }

  computeGravityVector(q, g_earth, g_ne);
}

void RobotModel::computeGravityVector(const std::array<double, 7>& q,
                                      const std::array<double, 3>& g_earth,
                                      std::array<double, 7>& g_ne) const {
  // Set gravity vector
  pinocchio_model_.gravity.linear(Eigen::Map<const Eigen::Vector3d>(g_earth.data()));

  copyToEigenQ(q);

  pinocchio::computeGeneralizedGravity(pinocchio_model_, data_gravity_, q_eigen_);
  copyFromEigen(data_gravity_.g, g_ne);
}

}  // namespace franka