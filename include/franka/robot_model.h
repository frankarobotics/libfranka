// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <string>

#include "franka/robot_model_base.h"

/**
 * @file robot_model.h
 * Implements RobotModelBase using Pinocchio.
 */

namespace franka {

/**
 * Implements of RobotModelBase using Pinocchio.
 */
class RobotModel : public RobotModelBase {
 public:
  RobotModel(const std::string& urdf);

  void coriolis(const std::array<double, 7>& q,   // NOLINT(readability-identifier-length)
                const std::array<double, 7>& dq,  // NOLINT(readability-identifier-length)
                const std::array<double, 9>& i_total,
                double m_total,
                const std::array<double, 3>& f_x_ctotal,
                std::array<double, 7>& c_ne) override;

  void coriolis(const std::array<double, 7>& q,   // NOLINT(readability-identifier-length)
                const std::array<double, 7>& dq,  // NOLINT(readability-identifier-length)
                const std::array<double, 9>& i_total,
                double m_total,
                const std::array<double, 3>& f_x_ctotal,
                const std::array<double, 3>& g_earth,
                std::array<double, 7>& c_ne) override;

  void gravity(const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
               const std::array<double, 3>& g_earth,
               double m_total,
               const std::array<double, 3>& f_x_ctotal,
               std::array<double, 7>& g_ne) override;

  void mass(const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
            const std::array<double, 9>& i_total,
            double m_total,
            const std::array<double, 3>& f_x_ctotal,
            std::array<double, 49>& m_ne) override;
  std::array<double, 16> pose(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      int joint_index) override;
  std::array<double, 16> poseEe(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee) override;
  std::array<double, 16> poseFlange(
      const std::array<double, 7>& q) override;  // NOLINT(readability-identifier-length)
  std::array<double, 16> poseStiffness(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee,
      const std::array<double, 16>& ee_t_k) override;
  std::array<double, 42> bodyJacobian(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      int joint_index) override;
  std::array<double, 42> bodyJacobianFlange(
      const std::array<double, 7>& q) override;  // NOLINT(readability-identifier-length)
  std::array<double, 42> bodyJacobianEe(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee) override;
  std::array<double, 42> bodyJacobianStiffness(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee,
      const std::array<double, 16>& ee_t_k) override;
  std::array<double, 42> zeroJacobian(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      int joint_index) override;
  std::array<double, 42> zeroJacobianFlange(
      const std::array<double, 7>& q) override;  // NOLINT(readability-identifier-length)
  std::array<double, 42> zeroJacobianEe(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee) override;
  std::array<double, 42> zeroJacobianStiffness(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee,
      const std::array<double, 16>& ee_t_k) override;

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
  mutable double cached_m_total_{-1.0};
  mutable std::array<double, 3> cached_f_x_ctotal_;
  mutable bool inertia_cache_valid_{false};

  mutable pinocchio::Model pinocchio_model_;
  /**
   * @brief Helper method to perform forward kinematics calculation
   * @param q Joint positions
   * @return Pinocchio data object with forward kinematics results
   */
  pinocchio::Data computeForwardKinematics(
      const std::array<double, 7>& q) const;  // NOLINT(readability-identifier-length)

  /**
   * @brief Helper function to convert Eigen Matrix4d to std::array<double, 16>
   * @param matrix The Eigen matrix to convert
   * @return Array representation of the matrix
   */
  static std::array<double, 16> eigenToArray(const Eigen::Matrix4d& matrix);

  /**
   * @brief Helper function to convert Eigen Matrix6x7d to std::array<double, 42>
   * @param matrix The Eigen matrix to convert
   * @return Array representation of the matrix
   */
  static std::array<double, 42> eigenToArray(const Eigen::Matrix<double, 6, 7>& matrix);

  /**
   * @brief Initializes the Pinocchio model from URDF and returns Data object
   * @param urdf URDF string to build model from
   * @return Pinocchio Data object for the initialized model
   */
  pinocchio::Data initializeModelAndReturnData(const std::string& urdf);

  /**
   * @brief Adds a new frame to the Pinocchio model
   * @param name Name of the frame to add
   * @param parent_frame_id ID of the parent frame
   * @param placement Transformation from parent frame to the new frame
   * @return Index of the newly added frame
   */
  pinocchio::FrameIndex addFrame(const std::string& name,
                                 pinocchio::FrameIndex parent_frame_id,
                                 const pinocchio::SE3& placement);

  /**
   * @brief A generic helper function to compute Jacobians for joints or frames
   * @param q Joint positions
   * @param frame_or_joint_index Index of the joint or frame for which to compute the Jacobian
   * @param is_joint Whether the index refers to a joint (true) or frame (false)
   * @param reference_frame The reference frame type (LOCAL or LOCAL_WORLD_ALIGNED)
   * @return The computed Jacobian as a std::array<double, 42>
   */
  std::array<double, 42> computeJacobian(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      int frame_or_joint_index,
      bool is_joint,
      pinocchio::ReferenceFrame reference_frame);

  /**
   * @brief Prepares and computes a Jacobian for the end-effector frame
   * @param q Joint positions
   * @param f_t_ee Transformation from flange to end-effector
   * @param reference_frame The reference frame type (LOCAL or LOCAL_WORLD_ALIGNED)
   * @return The computed Jacobian as a std::array<double, 42>
   */
  std::array<double, 42> computeEeJacobian(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee,
      pinocchio::ReferenceFrame reference_frame);

  /**
   * @brief Prepares and computes a Jacobian for the stiffness frame
   * @param q Joint positions
   * @param f_t_ee Transformation from flange to end-effector
   * @param ee_t_k Transformation from end-effector to stiffness frame
   * @param reference_frame The reference frame type (LOCAL or LOCAL_WORLD_ALIGNED)
   * @return The computed Jacobian as a std::array<double, 42>
   */
  std::array<double, 42> computeStiffnessJacobian(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee,
      const std::array<double, 16>& ee_t_k,
      pinocchio::ReferenceFrame reference_frame);

  /**
   * @brief Updates the placement of frames based on transforms
   * @param f_t_ee Transformation from flange to end-effector
   * @param ee_t_k Transformation from end-effector to stiffness frame (optional)
   * @param update_stiffness Whether to update the stiffness frame (default: false)
   */
  void updateFramePlacements(const std::array<double, 16>& f_t_ee,
                             const std::array<double, 16>& ee_t_k = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                                     1, 0, 0, 0, 0, 1},
                             bool update_stiffness = false);

  /**
   * @brief Updates the inertia if different from the current inertia parameters
   * @param i_total Inertia of the last link
   * @param m_total Total mass of the last link
   * @param f_x_ctotal Center of mass of the last link
   */
  void updateInertiaIfNeeded(const std::array<double, 9>& i_total,
                             double m_total,
                             const std::array<double, 3>& f_x_ctotal) const;

  /**
   * @brief Restores the original inertia
   */
  void restoreOriginalInertia() const;

  /**
   * @brief Computes the gravity vector
   * @param q Joint positions
   * @param g_earth Gravity vector
   * @param g_ne Gravity vector in the end-effector frame
   */
  void computeGravityVector(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 3>& g_earth,
      std::array<double, 7>& g_ne) const;

  static constexpr const char* kLastLinkName = "link8";

  pinocchio::Inertia initial_last_link_inertia_;
  pinocchio::FrameIndex last_link_frame_index_;
  pinocchio::JointIndex last_joint_index_;
  void copyToEigenQ(const std::array<double, 7>& q) const;  // NOLINT(readability-identifier-length)
  void copyToEigenDQ(
      const std::array<double, 7>& dq) const;  // NOLINT(readability-identifier-length)
  static void copyFromEigen(const Eigen::VectorXd& src, std::array<double, 7>& dst);
  static void copyFromEigenMatrix(const Eigen::MatrixXd& src, std::array<double, 49>& dst);
  pinocchio::FrameIndex ee_frame_index_;
  pinocchio::FrameIndex stiffness_frame_index_;
};

}  // namespace franka
