// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka/robot_model.h"

namespace franka {

RobotModel::RobotModel(const std::string& urdf) {
  pinocchio::urdf::buildModelFromXML(urdf, pinocchio_model_);

  last_joint_index_ = pinocchio_model_.joints.back().id();
  last_link_frame_index_ = pinocchio_model_.getFrameId(pinocchio_model_.frames.back().name);

  initial_last_link_inertia_ = pinocchio_model_.inertias[last_joint_index_];
}

void RobotModel::computeDynamics(
    const std::array<double, 9>& i_total,
    double m_total,
    const std::array<double, 3>& f_x_ctotal,
    pinocchio::Data& data,
    const std::function<void(pinocchio::Model&, pinocchio::Data&)>& compute_func) {
  addInertiaToLastLink(i_total, m_total, f_x_ctotal);

  compute_func(pinocchio_model_, data);

  pinocchio_model_.inertias[last_joint_index_] = initial_last_link_inertia_;
}

void RobotModel::addInertiaToLastLink(const std::array<double, 9>& i_total,
                                      double m_total,
                                      const std::array<double, 3>& f_x_ctotal) {
  Eigen::Matrix3d inertia_matrix = Eigen::Map<const Eigen::Matrix3d>(i_total.data(), 3, 3);
  Eigen::Vector3d com(f_x_ctotal[0], f_x_ctotal[1], f_x_ctotal[2]);
  pinocchio::Inertia inertia(m_total, com, inertia_matrix);

  // Update the inertia and mass of the end effector in the model
  pinocchio_model_.inertias[last_joint_index_] =
      initial_last_link_inertia_ +
      pinocchio_model_.frames[last_link_frame_index_].placement.act(inertia);
}

void RobotModel::coriolis(const std::array<double, 7>& q,
                          const std::array<double, 7>& dq,
                          const std::array<double, 9>& i_total,
                          double m_total,
                          const std::array<double, 3>& f_x_ctotal,
                          std::array<double, 7>& c_ne) {
  pinocchio::Data data(pinocchio_model_);

  Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q.data(), 7);
  Eigen::VectorXd dq_vec = Eigen::Map<const Eigen::VectorXd>(dq.data(), 7);

  auto lambda_coriolis = [&](pinocchio::Model& model, pinocchio::Data& data) {
    pinocchio::computeCoriolisMatrix(model, data, q_vec, dq_vec);
  };

  computeDynamics(i_total, m_total, f_x_ctotal, data, lambda_coriolis);

  auto coriolis_matrix = data.C;
  Eigen::VectorXd coriolis = coriolis_matrix * dq_vec;

  std::copy(coriolis.data(), coriolis.data() + coriolis.rows(), c_ne.begin());
}

void RobotModel::gravity(const std::array<double, 7>& q,
                         const std::array<double, 3>& g_earth,
                         double m_total,
                         const std::array<double, 3>& f_x_ctotal,
                         std::array<double, 7>& g_ne) {
  pinocchio_model_.gravity.linear(Eigen::Vector3d(g_earth[0], g_earth[1], g_earth[2]));

  pinocchio::Data data(pinocchio_model_);
  Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q.data(), 7);

  std::array<double, 9> zero_inertia = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  auto lambda_gravity = [&](pinocchio::Model& model, pinocchio::Data& data) {
    pinocchio::computeGeneralizedGravity(model, data, q_vec);
  };

  computeDynamics(zero_inertia, m_total, f_x_ctotal, data, lambda_gravity);

  Eigen::VectorXd g_ne_vec = data.g;

  std::copy(g_ne_vec.data(), g_ne_vec.data() + g_ne_vec.size(), g_ne.begin());
}

void RobotModel::mass(const std::array<double, 7>& q,
                      const std::array<double, 9>& i_total,
                      double m_total,
                      const std::array<double, 3>& f_x_ctotal,
                      std::array<double, 49>& m_ne) {
  pinocchio::Data data(pinocchio_model_);
  Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q.data(), 7);

  auto lambda_mass = [&](pinocchio::Model& model, pinocchio::Data& data) {
    pinocchio::crba(model, data, q_vec);
  };

  computeDynamics(i_total, m_total, f_x_ctotal, data, lambda_mass);

  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();

  std::copy(data.M.data(), data.M.data() + data.M.size(), m_ne.begin());
}

// Helper method to perform forward kinematics calculation
pinocchio::Data RobotModel::computeForwardKinematics(const std::array<double, 7>& q) const {
  // Create Pinocchio data object
  pinocchio::Data data(pinocchio_model_);

  // Convert joint positions array to Eigen vector
  Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q.data(), last_joint_index_);

  // Perform forward kinematics
  pinocchio::forwardKinematics(pinocchio_model_, data, q_vec);

  return data;
}

// Helper function to convert Eigen Matrix4d to std::array<double, 16>
std::array<double, 16> RobotModel::eigenToArray(const Eigen::Matrix4d& matrix) const {
  std::array<double, 16> result;
  std::copy(matrix.data(), matrix.data() + 16, result.begin());
  return result;
}

std::array<double, 16> RobotModel::pose(const std::array<double, 7>& q, int joint_index) {
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

std::array<double, 16> RobotModel::poseEe(const std::array<double, 7>& q,
                                          const std::array<double, 16>& f_t_ee) {
  // Get the flange pose
  std::array<double, 16> flange_transform = poseFlange(q);

  // Apply end effector transformation
  Eigen::Matrix4d ee_transform = Eigen::Map<const Eigen::Matrix4d>(flange_transform.data()) *
                                 Eigen::Map<const Eigen::Matrix4d>(f_t_ee.data());

  // Convert to array and return
  return eigenToArray(ee_transform);
}

std::array<double, 16> RobotModel::poseFlange(const std::array<double, 7>& q) {
  // Compute forward kinematics
  pinocchio::Data data = computeForwardKinematics(q);

  // Update the frame placement for the specific frame we need
  pinocchio::updateFramePlacement(pinocchio_model_, data, last_link_frame_index_);

  // Get the transformation matrix using the frame index
  Eigen::Matrix4d flange_transform = data.oMf[last_link_frame_index_].toHomogeneousMatrix();

  // Convert to array and return
  return eigenToArray(flange_transform);
}

std::array<double, 16> RobotModel::poseStiffness(const std::array<double, 7>& q,
                                                 const std::array<double, 16>& f_t_ee,
                                                 const std::array<double, 16>& ee_t_k) {
  std::array<double, 16> ee_pose = poseEe(q, f_t_ee);

  // apply the stiffness frame transform
  Eigen::Matrix4d transform = Eigen::Map<const Eigen::Matrix4d>(ee_pose.data()) *
                              Eigen::Map<const Eigen::Matrix4d>(ee_t_k.data());

  // Convert to array and return
  return eigenToArray(transform);
}

}  // namespace franka
