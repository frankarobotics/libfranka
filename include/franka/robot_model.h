// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

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
 * Implements RobotModelBase using Pinocchio.
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
  /**
   * @brief Adds the inertia of the attached total load including end effector to the last link of
   * the robot.
   */
  void addInertiaToLastLink(const std::array<double, 9>& i_total,
                            double m_total,
                            const std::array<double, 3>& f_x_ctotal);
  /**
   * @brief Computes the dynamics of the robot using the given function.
   *        Adds the inertia of the load and restores it after the computation.
   */
  void computeDynamics(
      const std::array<double, 9>& i_total,
      double m_total,
      const std::array<double, 3>& f_x_ctotal,
      pinocchio::Data& data,
      const std::function<void(pinocchio::Model&, pinocchio::Data&)>& compute_func);

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

  pinocchio::Model pinocchio_model_;
  pinocchio::Inertia initial_last_link_inertia_;
  pinocchio::FrameIndex last_link_frame_index_;
  pinocchio::JointIndex last_joint_index_;
  pinocchio::FrameIndex ee_frame_index_;
  pinocchio::FrameIndex stiffness_frame_index_;
};

}  // namespace franka
