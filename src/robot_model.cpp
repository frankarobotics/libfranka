// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka/robot_model.h"
#include <algorithm>

namespace franka {

RobotModel::RobotModel(const std::string& urdf) {
  pinocchio::urdf::buildModelFromXML(urdf, pinocchio_model_);

  last_link_frame_index_ = pinocchio_model_.getFrameId(kLastLinkName);
  last_joint_index_ =  // NOLINT(cppcoreguidelines-prefer-member-initializer)
      pinocchio_model_.frames[last_link_frame_index_]
          .parentJoint;  // NOLINT(cppcoreguidelines-prefer-member-initializer)

  initial_last_link_inertia_ = pinocchio_model_.inertias[last_joint_index_];

  cached_f_x_ctotal_.fill(-1.0);
  cached_i_total_.fill(-1.0);

  pinocchio::SE3 identity_transform = pinocchio::SE3::Identity();
  ee_frame_index_ = addFrame("end_effector", last_link_frame_index_, identity_transform);
  stiffness_frame_index_ = addFrame("stiffness", last_link_frame_index_, identity_transform);

  data_ = pinocchio::Data(pinocchio_model_);
  data_gravity_ = pinocchio::Data(pinocchio_model_);
}

pinocchio::FrameIndex RobotModel::addFrame(const std::string& name,
                                           pinocchio::FrameIndex parent_frame_id,
                                           const pinocchio::SE3& placement) {
  // Get parent frame information
  const pinocchio::Frame& parent_frame = pinocchio_model_.frames[parent_frame_id];

  // Add the new frame to the Pinocchio model
  pinocchio::FrameIndex new_frame_index = pinocchio_model_.addFrame(pinocchio::Frame(
      name, parent_frame.parentJoint, parent_frame_id, placement, pinocchio::OP_FRAME));

  return new_frame_index;
}

void RobotModel::copyToEigenQ(
    const std::array<double, 7>& q) const {  // NOLINT(readability-identifier-length)
  q_eigen_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q.data());
}

void RobotModel::copyToEigenDQ(
    const std::array<double, 7>& dq) const {  // NOLINT(readability-identifier-length)
  dq_eigen_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(dq.data());
}

void RobotModel::copyFromEigen(const Eigen::VectorXd& src, std::array<double, 7>& dst) {
  std::copy(src.data(), src.data() + 7, dst.begin());
}

void RobotModel::copyFromEigenMatrix(const Eigen::MatrixXd& src, std::array<double, 49>& dst) {
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

void RobotModel::mass(const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
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

void RobotModel::coriolis(const std::array<double, 7>& q,   // NOLINT(readability-identifier-length)
                          const std::array<double, 7>& dq,  // NOLINT(readability-identifier-length)
                          const std::array<double, 9>& i_total,
                          double m_total,
                          const std::array<double, 3>& f_x_ctotal,
                          std::array<double, 7>& c_ne) {
  // DEPRECATED: Use coriolis() with gravity parameter for better performance

  updateInertiaIfNeeded(i_total, m_total, f_x_ctotal);

  std::array<double, 3> earth_gravity = {{0.0, 0.0, -9.81}};
  pinocchio_model_.gravity.linear(Eigen::Map<const Eigen::Vector3d>(earth_gravity.data()));

  copyToEigenQ(q);
  copyToEigenDQ(dq);

  pinocchio::computeCoriolisMatrix(pinocchio_model_, data_, q_eigen_, dq_eigen_);
  tau_eigen_.noalias() = data_.C * dq_eigen_;

  copyFromEigen(tau_eigen_, c_ne);
}

void RobotModel::coriolis(const std::array<double, 7>& q,   // NOLINT(readability-identifier-length)
                          const std::array<double, 7>& dq,  // NOLINT(readability-identifier-length)
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

void RobotModel::gravity(const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
                         const std::array<double, 3>& g_earth,
                         double m_total,
                         const std::array<double, 3>& f_x_ctotal,
                         std::array<double, 7>& g_ne) {
  if (m_total > 0.0) {
    updateInertiaIfNeeded(std::array<double, 9>{0, 0, 0, 0, 0, 0, 0, 0, 0}, m_total, f_x_ctotal);
  }

  computeGravityVector(q, g_earth, g_ne);
}

void RobotModel::computeGravityVector(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    const std::array<double, 3>& g_earth,
    std::array<double, 7>& g_ne) const {
  // Set gravity vector
  pinocchio_model_.gravity.linear(Eigen::Map<const Eigen::Vector3d>(g_earth.data()));

  copyToEigenQ(q);

  pinocchio::computeGeneralizedGravity(pinocchio_model_, data_gravity_, q_eigen_);
  copyFromEigen(data_gravity_.g, g_ne);
}

// Helper method to perform forward kinematics calculation
pinocchio::Data RobotModel::computeForwardKinematics(
    const std::array<double, 7>& q) const {  // NOLINT(readability-identifier-length)
  // Create Pinocchio data object
  pinocchio::Data data(pinocchio_model_);

  // Convert joint positions array to Eigen vector
  Eigen::VectorXd q_vec =
      Eigen::Map<const Eigen::VectorXd>(q.data(), static_cast<Eigen::Index>(last_joint_index_));

  // Perform forward kinematics
  pinocchio::forwardKinematics(pinocchio_model_, data, q_vec);

  return data;
}

// Helper function to convert Eigen Matrix4d to std::array<double, 16>
std::array<double, 16> RobotModel::eigenToArray(const Eigen::Matrix4d& matrix) {
  std::array<double, 16> result;
  std::copy(matrix.data(), matrix.data() + 16, result.begin());
  return result;
}

// Helper function to convert Eigen Matrix<double, 6, 7> to std::array<double, 42>
std::array<double, 42> RobotModel::eigenToArray(const Eigen::Matrix<double, 6, 7>& matrix) {
  std::array<double, 42> result;
  std::copy(matrix.data(), matrix.data() + 42, result.begin());
  return result;
}

// New helper method to update frame placements based on transforms
void RobotModel::updateFramePlacements(const std::array<double, 16>& f_t_ee,
                                       const std::array<double, 16>& ee_t_k,
                                       bool update_stiffness) {
  // Get the flange frame and its placement
  const pinocchio::Frame& flange_frame = pinocchio_model_.frames[last_link_frame_index_];
  pinocchio::SE3 flange_placement = flange_frame.placement;

  // Extract transform from f_t_ee
  Eigen::Matrix4d transform_ee = Eigen::Map<const Eigen::Matrix4d>(f_t_ee.data());
  Eigen::Matrix3d rotation_ee = transform_ee.block<3, 3>(0, 0);
  Eigen::Vector3d translation_ee = transform_ee.block<3, 1>(0, 3);
  pinocchio::SE3 ee_transform(rotation_ee, translation_ee);

  // Update the EE frame placement
  pinocchio_model_.frames[ee_frame_index_].placement = flange_placement * ee_transform;

  // If we need to update the stiffness frame
  if (update_stiffness) {
    // Extract transform from ee_t_k
    Eigen::Matrix4d transform_k = Eigen::Map<const Eigen::Matrix4d>(ee_t_k.data());
    Eigen::Matrix3d rotation_k = transform_k.block<3, 3>(0, 0);
    Eigen::Vector3d translation_k = transform_k.block<3, 1>(0, 3);
    pinocchio::SE3 k_transform(rotation_k, translation_k);

    // Update the stiffness frame placement
    pinocchio_model_.frames[stiffness_frame_index_].placement =
        flange_placement * ee_transform * k_transform;
  }
}

// Generic helper function to compute Jacobians
std::array<double, 42> RobotModel::computeJacobian(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    int frame_or_joint_index,
    bool is_joint,
    pinocchio::ReferenceFrame reference_frame) {
  // Validate index if it's a joint
  if (is_joint && (frame_or_joint_index < 1 ||
                   static_cast<pinocchio::JointIndex>(frame_or_joint_index) > last_joint_index_)) {
    throw std::runtime_error("Joint index out of bounds: " + std::to_string(frame_or_joint_index) +
                             " (valid range: 1 to " + std::to_string(last_joint_index_) + ")");
  }

  // Create Pinocchio data object
  pinocchio::Data data(pinocchio_model_);

  // Convert joint positions array to Eigen vector
  Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q.data(), 7);

  // Compute joint jacobians
  pinocchio::computeJointJacobians(pinocchio_model_, data, q_vec);

  // If computing for a frame, perform forward kinematics and update frame placement
  if (!is_joint) {
    pinocchio::forwardKinematics(pinocchio_model_, data, q_vec);
    pinocchio::updateFramePlacement(pinocchio_model_, data, frame_or_joint_index);
  }

  // Initialize Jacobian matrix
  Eigen::Matrix<double, 6, 7> full_jacobian(6, pinocchio_model_.nv);
  full_jacobian.setZero();

  // Get the appropriate Jacobian
  if (is_joint) {
    pinocchio::getJointJacobian(pinocchio_model_, data, frame_or_joint_index, reference_frame,
                                full_jacobian);
  } else {
    pinocchio::getFrameJacobian(pinocchio_model_, data, frame_or_joint_index, reference_frame,
                                full_jacobian);
  }

  // Extract the 6x7 Jacobian matrix
  Eigen::Matrix<double, 6, 7> jacobian = full_jacobian.leftCols(7);

  // Convert to array and return
  return eigenToArray(jacobian);
}

// Helper function to compute EE Jacobian
std::array<double, 42> RobotModel::computeEeJacobian(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    const std::array<double, 16>& f_t_ee,
    pinocchio::ReferenceFrame reference_frame) {
  // If there's no end effector transformation, compute for the flange
  if (f_t_ee == std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}) {
    return computeJacobian(q, static_cast<int>(last_link_frame_index_), false, reference_frame);
  }

  // Update the end-effector frame placement
  updateFramePlacements(f_t_ee);

  // Compute the Jacobian for the EE frame
  return computeJacobian(q, static_cast<int>(ee_frame_index_), false, reference_frame);
}

// Helper function to compute Stiffness Jacobian
std::array<double, 42> RobotModel::computeStiffnessJacobian(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    const std::array<double, 16>& f_t_ee,
    const std::array<double, 16>& ee_t_k,
    pinocchio::ReferenceFrame reference_frame) {
  // If there's no stiffness transformation, compute for the EE
  if (ee_t_k == std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}) {
    return computeEeJacobian(q, f_t_ee, reference_frame);
  }

  // If both transforms are identity, compute for the flange
  if (f_t_ee == std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1} &&
      ee_t_k == std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}) {
    return computeJacobian(q, static_cast<int>(last_link_frame_index_), false, reference_frame);
  }

  // Update both the EE and stiffness frame placements
  updateFramePlacements(f_t_ee, ee_t_k, true);

  // Compute the Jacobian for the stiffness frame
  return computeJacobian(q, static_cast<int>(stiffness_frame_index_), false, reference_frame);
}

std::array<double, 16> RobotModel::pose(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    int joint_index) {
  // Validate joint_index to prevent segmentation fault
  if (joint_index < 1 || joint_index > static_cast<int>(last_joint_index_)) {
    throw std::runtime_error("Joint index out of bounds: " + std::to_string(joint_index) +
                             " (valid range: 1 to " + std::to_string(last_joint_index_) + ")");
  }

  // Compute forward kinematics
  pinocchio::Data data = computeForwardKinematics(q);

  // Get the transformation matrix for the specified joint
  Eigen::Matrix4d homogeneous_transform = data.oMi[joint_index].toHomogeneousMatrix();

  // Convert to array and return
  return eigenToArray(homogeneous_transform);
}

std::array<double, 16> RobotModel::poseEe(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    const std::array<double, 16>& f_t_ee) {
  // Get the flange pose
  std::array<double, 16> flange_transform = poseFlange(q);

  // Apply end effector transformation
  Eigen::Matrix4d ee_transform = Eigen::Map<const Eigen::Matrix4d>(flange_transform.data()) *
                                 Eigen::Map<const Eigen::Matrix4d>(f_t_ee.data());

  // Convert to array and return
  return eigenToArray(ee_transform);
}

std::array<double, 16> RobotModel::poseFlange(
    const std::array<double, 7>& q) {  // NOLINT(readability-identifier-length)
  // Compute forward kinematics
  pinocchio::Data data = computeForwardKinematics(q);

  // Update the frame placement for the specific frame we need
  pinocchio::updateFramePlacement(pinocchio_model_, data, last_link_frame_index_);

  // Get the transformation matrix using the frame index
  Eigen::Matrix4d flange_transform = data.oMf[last_link_frame_index_].toHomogeneousMatrix();

  // Convert to array and return
  return eigenToArray(flange_transform);
}

std::array<double, 16> RobotModel::poseStiffness(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    const std::array<double, 16>& f_t_ee,
    const std::array<double, 16>& ee_t_k) {
  // Get the end-effector pose
  std::array<double, 16> ee_pose = poseEe(q, f_t_ee);

  // Apply stiffness frame transformation
  Eigen::Matrix4d stiffness_transform = Eigen::Map<const Eigen::Matrix4d>(ee_pose.data()) *
                                        Eigen::Map<const Eigen::Matrix4d>(ee_t_k.data());

  // Convert to array and return
  return eigenToArray(stiffness_transform);
}

std::array<double, 42> RobotModel::bodyJacobian(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    int joint_index) {
  return computeJacobian(q, joint_index, true, pinocchio::ReferenceFrame::LOCAL);
}

std::array<double, 42> RobotModel::bodyJacobianFlange(
    const std::array<double, 7>& q) {  // NOLINT(readability-identifier-length)
  return computeJacobian(q, static_cast<int>(last_link_frame_index_), false,
                         pinocchio::ReferenceFrame::LOCAL);
}

std::array<double, 42> RobotModel::bodyJacobianEe(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    const std::array<double, 16>& f_t_ee) {
  return computeEeJacobian(q, f_t_ee, pinocchio::ReferenceFrame::LOCAL);
}

std::array<double, 42> RobotModel::bodyJacobianStiffness(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    const std::array<double, 16>& f_t_ee,
    const std::array<double, 16>& ee_t_k) {
  return computeStiffnessJacobian(q, f_t_ee, ee_t_k, pinocchio::ReferenceFrame::LOCAL);
}

std::array<double, 42> RobotModel::zeroJacobian(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    int joint_index) {
  return computeJacobian(q, joint_index, true, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
}

std::array<double, 42> RobotModel::zeroJacobianFlange(
    const std::array<double, 7>& q) {  // NOLINT(readability-identifier-length)
  return computeJacobian(q, static_cast<int>(last_link_frame_index_), false,
                         pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
}

std::array<double, 42> RobotModel::zeroJacobianEe(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    const std::array<double, 16>& f_t_ee) {
  return computeEeJacobian(q, f_t_ee, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
}

std::array<double, 42> RobotModel::zeroJacobianStiffness(
    const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
    const std::array<double, 16>& f_t_ee,
    const std::array<double, 16>& ee_t_k) {
  return computeStiffnessJacobian(q, f_t_ee, ee_t_k,
                                  pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
}

}  // namespace franka
