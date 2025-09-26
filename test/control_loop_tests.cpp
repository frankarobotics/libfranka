// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <exception>
#include <functional>

#include <gmock/gmock.h>

#include <franka/lowpass_filter.h>
#include "control_loop.h"
#include "motion_generator_traits.h"

#include "helpers.h"
#include "mock_robot_control.h"

using namespace ::testing;

using franka::CartesianPose;
using franka::CartesianVelocities;
using franka::ControllerMode;
using franka::Duration;
using franka::JointPositions;
using franka::JointVelocities;
using franka::RobotState;
using franka::Torques;

using research_interface::robot::ControllerCommand;
using research_interface::robot::MotionGeneratorCommand;
using research_interface::robot::Move;
using research_interface::robot::RobotCommand;

using std::placeholders::_1;
using std::placeholders::_2;

constexpr bool kUseNoAsyncMotionGenerator = false;
const std::optional<std::vector<double>> kNoMaximumVelocities = std::nullopt;

// Helper function to setup rate limiter mock expectations
void setupJointVelocityLimitsMock(MockRobotControl& robot) {
  EXPECT_CALL(robot, getUpperJointVelocityLimits(testing::_))
      .WillRepeatedly(testing::Return(std::array<double, 7>{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0}));
  EXPECT_CALL(robot, getLowerJointVelocityLimits(testing::_))
      .WillRepeatedly(
          testing::Return(std::array<double, 7>{-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0}));
}

class MockControlCallback {
 public:
  MOCK_METHOD2(invoke, Torques(const RobotState&, Duration));
};

template <typename T>
class ControlLoop : public franka::ControlLoop<T> {
 public:
  using franka::ControlLoop<T>::ControlLoop;
  using franka::ControlLoop<T>::createMotionCommand;
  using franka::ControlLoop<T>::createControlCommand;
};

template <typename T>
struct MockMotionCallback {
  MOCK_METHOD2_T(invoke, T(const RobotState&, Duration));
};

double getCutoffFreq(bool filter) {
  return (filter) ? franka::kDefaultCutoffFrequency : franka::kMaxCutoffFrequency;
}

template <bool LimitRate, bool Filter>
struct JointPositionMotion {
  using Motion = JointPositions;
  static constexpr bool kLimitRate = LimitRate;
  static constexpr bool kFilter = Filter;
};
template <bool LimitRate, bool Filter>
struct JointVelocityMotion {
  using Motion = JointVelocities;
  static constexpr bool kLimitRate = LimitRate;
  static constexpr bool kFilter = Filter;
};
template <bool LimitRate, bool Filter>
struct CartesianPoseMotion {
  using Motion = CartesianPose;
  static constexpr bool kLimitRate = LimitRate;
  static constexpr bool kFilter = Filter;
};
template <bool LimitRate, bool Filter>
struct CartesianPoseMotionWithElbow {
  using Motion = CartesianPose;
  static constexpr bool kLimitRate = LimitRate;
  static constexpr bool kFilter = Filter;
};
template <bool LimitRate, bool Filter>
struct CartesianVelocityMotion {
  using Motion = CartesianVelocities;
  static constexpr bool kLimitRate = LimitRate;
  static constexpr bool kFilter = Filter;
};
template <bool LimitRate, bool Filter>
struct CartesianVelocityMotionWithElbow {
  using Motion = CartesianVelocities;
  static constexpr bool kLimitRate = LimitRate;
  static constexpr bool kFilter = Filter;
};

template <typename T>
class ControlLoops : public ::testing::Test {
 public:
  using TMotion = typename T::Motion;
  using Loop = ControlLoop<TMotion>;
  using MotionGeneratorCallback = typename Loop::MotionGeneratorCallback;
  using ControlCallback = typename Loop::ControlCallback;

  const research_interface::robot::Move::MotionGeneratorMode kMotionGeneratorMode =
      franka::MotionGeneratorTraits<TMotion>::kMotionGeneratorMode;

  static constexpr bool kLimitRate = T::kLimitRate;
  static constexpr bool kFilter = T::kFilter;
  TMotion createMotion();
  TMotion createInvalidMotion(double invalid_value);
  auto getField(const TMotion& values);
};

template <typename T>
class ControlLoopWithTransformationMatrix : public ::testing::Test {
 public:
  using TMotion = typename T::Motion;
  using Loop = ControlLoop<TMotion>;
  using MotionGeneratorCallback = typename Loop::MotionGeneratorCallback;
  using ControlCallback = typename Loop::ControlCallback;

  const research_interface::robot::Move::MotionGeneratorMode kMotionGeneratorMode =
      franka::MotionGeneratorTraits<TMotion>::kMotionGeneratorMode;

  static constexpr bool kLimitRate = T::kLimitRate;
  static constexpr bool kFilter = T::kFilter;
  TMotion createInvalidTransformationMatrix();
};

template <typename T>
class ControlLoopWithElbow : public ::testing::Test {
 public:
  using TMotion = typename T::Motion;
  using Loop = ControlLoop<TMotion>;
  using MotionGeneratorCallback = typename Loop::MotionGeneratorCallback;
  using ControlCallback = typename Loop::ControlCallback;

  const research_interface::robot::Move::MotionGeneratorMode kMotionGeneratorMode =
      franka::MotionGeneratorTraits<TMotion>::kMotionGeneratorMode;

  static constexpr bool kLimitRate = T::kLimitRate;
  static constexpr bool kFilter = T::kFilter;
  TMotion createElbowConfig(double position, double sign);
};

template <>
JointPositions ControlLoops<JointPositionMotion<true, true>>::createMotion() {
  return JointPositions({0, 1, 2, 3, 4, 5, 6});
}

template <>
JointPositions ControlLoops<JointPositionMotion<false, true>>::createMotion() {
  return JointPositions({0, 1, 2, 3, 4, 5, 6});
}

template <>
JointPositions ControlLoops<JointPositionMotion<true, false>>::createMotion() {
  return JointPositions({0, 1, 2, 3, 4, 5, 6});
}

template <>
JointPositions ControlLoops<JointPositionMotion<false, false>>::createMotion() {
  return JointPositions({0, 1, 2, 3, 4, 5, 6});
}

template <>
JointPositions ControlLoops<JointPositionMotion<true, true>>::createInvalidMotion(
    double invalid_value) {
  return JointPositions({0, 1, invalid_value, 3, 4, 5, 6});
}

template <>
JointPositions ControlLoops<JointPositionMotion<false, true>>::createInvalidMotion(
    double invalid_value) {
  return JointPositions({0, 1, invalid_value, 3, 4, 5, 6});
}

template <>
JointPositions ControlLoops<JointPositionMotion<true, false>>::createInvalidMotion(
    double invalid_value) {
  return JointPositions({0, 1, invalid_value, 3, 4, 5, 6});
}

template <>
JointPositions ControlLoops<JointPositionMotion<false, false>>::createInvalidMotion(
    double invalid_value) {
  return JointPositions({0, 1, invalid_value, 3, 4, 5, 6});
}

template <>
auto ControlLoops<JointPositionMotion<true, true>>::getField(const JointPositions& values) {
  return Field(&research_interface::robot::MotionGeneratorCommand::q_c, Eq(values.q));
}

template <>
auto ControlLoops<JointPositionMotion<false, true>>::getField(const JointPositions& values) {
  return Field(&research_interface::robot::MotionGeneratorCommand::q_c, Eq(values.q));
}

template <>
auto ControlLoops<JointPositionMotion<true, false>>::getField(const JointPositions& values) {
  return Field(&research_interface::robot::MotionGeneratorCommand::q_c, Eq(values.q));
}

template <>
auto ControlLoops<JointPositionMotion<false, false>>::getField(const JointPositions& values) {
  return Field(&research_interface::robot::MotionGeneratorCommand::q_c, Eq(values.q));
}

template <>
JointVelocities ControlLoops<JointVelocityMotion<true, true>>::createMotion() {
  return JointVelocities({0, 1, 2, 3, 4, 5, 6});
}

template <>
JointVelocities ControlLoops<JointVelocityMotion<false, true>>::createMotion() {
  return JointVelocities({0, 1, 2, 3, 4, 5, 6});
}

template <>
JointVelocities ControlLoops<JointVelocityMotion<true, false>>::createMotion() {
  return JointVelocities({0, 1, 2, 3, 4, 5, 6});
}

template <>
JointVelocities ControlLoops<JointVelocityMotion<false, false>>::createMotion() {
  return JointVelocities({0, 1, 2, 3, 4, 5, 6});
}

template <>
JointVelocities ControlLoops<JointVelocityMotion<true, true>>::createInvalidMotion(
    double invalid_value) {
  return JointVelocities({0, 1, invalid_value, 3, 4, 5, 6});
}

template <>
JointVelocities ControlLoops<JointVelocityMotion<false, true>>::createInvalidMotion(
    double invalid_value) {
  return JointVelocities({0, 1, invalid_value, 3, 4, 5, 6});
}

template <>
JointVelocities ControlLoops<JointVelocityMotion<true, false>>::createInvalidMotion(
    double invalid_value) {
  return JointVelocities({0, 1, invalid_value, 3, 4, 5, 6});
}

template <>
JointVelocities ControlLoops<JointVelocityMotion<false, false>>::createInvalidMotion(
    double invalid_value) {
  return JointVelocities({0, 1, invalid_value, 3, 4, 5, 6});
}

template <>
auto ControlLoops<JointVelocityMotion<true, true>>::getField(const JointVelocities& velocities) {
  return Field(&research_interface::robot::MotionGeneratorCommand::dq_c, Lt(velocities.dq));
}

template <>
auto ControlLoops<JointVelocityMotion<false, true>>::getField(const JointVelocities& velocities) {
  return Field(&research_interface::robot::MotionGeneratorCommand::dq_c, Lt(velocities.dq));
}

template <>
auto ControlLoops<JointVelocityMotion<true, false>>::getField(const JointVelocities& velocities) {
  return Field(&research_interface::robot::MotionGeneratorCommand::dq_c, Lt(velocities.dq));
}

template <>
auto ControlLoops<JointVelocityMotion<false, false>>::getField(const JointVelocities& velocities) {
  return Field(&research_interface::robot::MotionGeneratorCommand::dq_c, Eq(velocities.dq));
}

template <>
CartesianPose ControlLoops<CartesianPoseMotion<true, true>>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotion<false, true>>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotion<true, false>>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotion<false, false>>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotion<true, true>>::createInvalidMotion(
    double invalid_value) {
  return CartesianPose(
      {1.0, 0.0, 0.0, invalid_value, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotion<false, true>>::createInvalidMotion(
    double invalid_value) {
  return CartesianPose(
      {1.0, 0.0, 0.0, invalid_value, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotion<true, false>>::createInvalidMotion(
    double invalid_value) {
  return CartesianPose(
      {1.0, 0.0, 0.0, invalid_value, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotion<false, false>>::createInvalidMotion(
    double invalid_value) {
  return CartesianPose(
      {1.0, 0.0, 0.0, invalid_value, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
}

template <>
CartesianPose ControlLoopWithTransformationMatrix<
    CartesianPoseMotion<true, true>>::createInvalidTransformationMatrix() {
  return CartesianPose(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

template <>
CartesianPose ControlLoopWithTransformationMatrix<
    CartesianPoseMotion<false, true>>::createInvalidTransformationMatrix() {
  return CartesianPose(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

template <>
CartesianPose ControlLoopWithTransformationMatrix<
    CartesianPoseMotion<true, false>>::createInvalidTransformationMatrix() {
  return CartesianPose(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

template <>
CartesianPose ControlLoopWithTransformationMatrix<
    CartesianPoseMotion<false, false>>::createInvalidTransformationMatrix() {
  return CartesianPose(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

template <>
auto ControlLoops<CartesianPoseMotion<true, true>>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_c, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
auto ControlLoops<CartesianPoseMotion<false, true>>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_c, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
auto ControlLoops<CartesianPoseMotion<true, false>>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_c, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
auto ControlLoops<CartesianPoseMotion<false, false>>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_c, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
CartesianPose ControlLoops<CartesianPoseMotionWithElbow<true, true>>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotionWithElbow<false, true>>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotionWithElbow<true, false>>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotionWithElbow<false, false>>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotionWithElbow<true, true>>::createInvalidMotion(
    double invalid_value) {
  return CartesianPose(
      {1.0, 0.0, 0.0, invalid_value, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0},
      {0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotionWithElbow<false, true>>::createInvalidMotion(
    double invalid_value) {
  return CartesianPose(
      {1.0, 0.0, 0.0, invalid_value, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0},
      {0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotionWithElbow<true, false>>::createInvalidMotion(
    double invalid_value) {
  return CartesianPose(
      {1.0, 0.0, 0.0, invalid_value, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0},
      {0, 1});
}

template <>
CartesianPose ControlLoops<CartesianPoseMotionWithElbow<false, false>>::createInvalidMotion(
    double invalid_value) {
  return CartesianPose(
      {1.0, 0.0, 0.0, invalid_value, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0},
      {0, 1});
}

template <>
CartesianPose ControlLoopWithTransformationMatrix<
    CartesianPoseMotionWithElbow<true, true>>::createInvalidTransformationMatrix() {
  return CartesianPose(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0, 1});
}

template <>
CartesianPose ControlLoopWithTransformationMatrix<
    CartesianPoseMotionWithElbow<false, true>>::createInvalidTransformationMatrix() {
  return CartesianPose(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0, 1});
}

template <>
CartesianPose ControlLoopWithTransformationMatrix<
    CartesianPoseMotionWithElbow<true, false>>::createInvalidTransformationMatrix() {
  return CartesianPose(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0, 1});
}

template <>
CartesianPose ControlLoopWithTransformationMatrix<
    CartesianPoseMotionWithElbow<false, false>>::createInvalidTransformationMatrix() {
  return CartesianPose(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0, 1});
}

template <>
CartesianPose ControlLoopWithElbow<CartesianPoseMotionWithElbow<true, true>>::createElbowConfig(
    double position,
    double sign) {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {position, sign});
}

template <>
CartesianPose ControlLoopWithElbow<CartesianPoseMotionWithElbow<false, true>>::createElbowConfig(
    double position,
    double sign) {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {position, sign});
}

template <>
CartesianPose ControlLoopWithElbow<CartesianPoseMotionWithElbow<true, false>>::createElbowConfig(
    double position,
    double sign) {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {position, sign});
}

template <>
CartesianPose ControlLoopWithElbow<CartesianPoseMotionWithElbow<false, false>>::createElbowConfig(
    double position,
    double sign) {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {position, sign});
}

template <>
auto ControlLoops<CartesianPoseMotionWithElbow<true, true>>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_c, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c, Eq(pose.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

template <>
auto ControlLoops<CartesianPoseMotionWithElbow<false, true>>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_c, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c, Eq(pose.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

template <>
auto ControlLoops<CartesianPoseMotionWithElbow<true, false>>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_c, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c, Eq(pose.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

template <>
auto ControlLoops<CartesianPoseMotionWithElbow<false, false>>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_c, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c, Eq(pose.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotion<true, true>>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotion<false, true>>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotion<true, false>>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotion<false, false>>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotion<true, true>>::createInvalidMotion(
    double invalid_value) {
  return CartesianVelocities({0, 1, invalid_value, 3, 4, 5});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotion<false, true>>::createInvalidMotion(
    double invalid_value) {
  return CartesianVelocities({0, 1, invalid_value, 3, 4, 5});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotion<true, false>>::createInvalidMotion(
    double invalid_value) {
  return CartesianVelocities({0, 1, invalid_value, 3, 4, 5});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotion<false, false>>::createInvalidMotion(
    double invalid_value) {
  return CartesianVelocities({0, 1, invalid_value, 3, 4, 5});
}

template <>
auto ControlLoops<CartesianVelocityMotion<true, true>>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_c,
                     Lt(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
auto ControlLoops<CartesianVelocityMotion<false, true>>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_c,
                     Lt(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
auto ControlLoops<CartesianVelocityMotion<true, false>>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_c,
                     Lt(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
auto ControlLoops<CartesianVelocityMotion<false, false>>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_c,
                     Eq(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotionWithElbow<true, true>>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5}, {0, -1});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotionWithElbow<false, true>>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5}, {0, -1});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotionWithElbow<true, false>>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5}, {0, -1});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotionWithElbow<false, false>>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5}, {0, -1});
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotionWithElbow<true, true>>::createInvalidMotion(
    double invalid_value) {
  return CartesianVelocities({0, 1, invalid_value, 3, 4, 5}, {0, -1});
}

template <>
CartesianVelocities
ControlLoops<CartesianVelocityMotionWithElbow<false, true>>::createInvalidMotion(
    double invalid_value) {
  return CartesianVelocities({0, 1, invalid_value, 3, 4, 5}, {0, -1});
}

template <>
CartesianVelocities
ControlLoops<CartesianVelocityMotionWithElbow<true, false>>::createInvalidMotion(
    double invalid_value) {
  return CartesianVelocities({0, 1, invalid_value, 3, 4, 5}, {0, -1});
}

template <>
CartesianVelocities
ControlLoops<CartesianVelocityMotionWithElbow<false, false>>::createInvalidMotion(
    double invalid_value) {
  return CartesianVelocities({0, 1, invalid_value, 3, 4, 5}, {0, -1});
}

template <>
CartesianVelocities ControlLoopWithElbow<
    CartesianVelocityMotionWithElbow<true, true>>::createElbowConfig(double position, double sign) {
  return CartesianVelocities({0, 1, 2, 3, 4, 5}, {position, sign});
}

template <>
CartesianVelocities
ControlLoopWithElbow<CartesianVelocityMotionWithElbow<false, true>>::createElbowConfig(
    double position,
    double sign) {
  return CartesianVelocities({0, 1, 2, 3, 4, 5}, {position, sign});
}

template <>
CartesianVelocities
ControlLoopWithElbow<CartesianVelocityMotionWithElbow<true, false>>::createElbowConfig(
    double position,
    double sign) {
  return CartesianVelocities({0, 1, 2, 3, 4, 5}, {position, sign});
}

template <>
CartesianVelocities
ControlLoopWithElbow<CartesianVelocityMotionWithElbow<false, false>>::createElbowConfig(
    double position,
    double sign) {
  return CartesianVelocities({0, 1, 2, 3, 4, 5}, {position, sign});
}

template <>
auto ControlLoops<CartesianVelocityMotionWithElbow<true, true>>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_c,
                     Lt(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(cartesian_velocities.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

template <>
auto ControlLoops<CartesianVelocityMotionWithElbow<false, true>>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_c,
                     Lt(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(cartesian_velocities.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

template <>
auto ControlLoops<CartesianVelocityMotionWithElbow<true, false>>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_c,
                     Lt(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(cartesian_velocities.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

template <>
auto ControlLoops<CartesianVelocityMotionWithElbow<false, false>>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_c,
                     Eq(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_c,
                     Eq(cartesian_velocities.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

using MotionTypes = ::testing::Types<JointPositionMotion<false, true>,
                                     JointVelocityMotion<false, true>,
                                     CartesianPoseMotion<false, true>,
                                     CartesianPoseMotionWithElbow<false, true>,
                                     CartesianVelocityMotion<false, true>,
                                     CartesianVelocityMotionWithElbow<false, true>,
                                     JointPositionMotion<true, true>,
                                     JointVelocityMotion<true, true>,
                                     CartesianPoseMotion<true, true>,
                                     CartesianPoseMotionWithElbow<true, true>,
                                     CartesianVelocityMotion<true, true>,
                                     CartesianVelocityMotionWithElbow<true, true>,
                                     JointPositionMotion<false, false>,
                                     JointVelocityMotion<false, false>,
                                     CartesianPoseMotion<false, false>,
                                     CartesianPoseMotionWithElbow<false, false>,
                                     CartesianVelocityMotion<false, false>,
                                     CartesianVelocityMotionWithElbow<false, false>,
                                     JointPositionMotion<true, false>,
                                     JointVelocityMotion<true, false>,
                                     CartesianPoseMotion<true, false>,
                                     CartesianPoseMotionWithElbow<true, false>,
                                     CartesianVelocityMotion<true, false>,
                                     CartesianVelocityMotionWithElbow<true, false>>;

TYPED_TEST_CASE(ControlLoops, MotionTypes);

TYPED_TEST(ControlLoops, CanNotConstructWithoutMotionCallback) {
  StrictMock<MockRobotControl> robot;
  setupJointVelocityLimitsMock(robot);

  EXPECT_THROW(typename TestFixture::Loop loop(
                   robot,
                   [](const RobotState&, Duration) {
                     return Torques({0, 1, 2, 3, 4, 5, 6});
                   },
                   typename TestFixture::MotionGeneratorCallback(), TestFixture::kLimitRate,
                   getCutoffFreq(TestFixture::kFilter)),
               std::invalid_argument);

  EXPECT_THROW(
      typename TestFixture::Loop loop(robot, ControllerMode::kCartesianImpedance,
                                      typename TestFixture::MotionGeneratorCallback(),
                                      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter)),
      std::invalid_argument);
}

TYPED_TEST(ControlLoops, CanNotConstructWithoutControlCallback) {
  StrictMock<MockRobotControl> robot;
  setupJointVelocityLimitsMock(robot);

  EXPECT_THROW(
      typename TestFixture::Loop loop(robot, typename TestFixture::ControlCallback(),
                                      std::bind(&TestFixture::createMotion, this),
                                      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter)),
      std::invalid_argument);
}

TYPED_TEST(ControlLoops, CanConstructWithMotionAndControllerCallback) {
  MockRobotControl robot;
  EXPECT_CALL(
      robot, startMotion(Move::ControllerMode::kExternalController, this->kMotionGeneratorMode,
                         TestFixture::Loop::kDefaultDeviation, TestFixture::Loop::kDefaultDeviation,
                         kUseNoAsyncMotionGenerator, kNoMaximumVelocities))
      .WillOnce(Return(100));

  EXPECT_NO_THROW(typename TestFixture::Loop(
      robot,
      [](const RobotState&, Duration) {
        return Torques({0, 1, 2, 3, 4, 5, 6});
      },
      std::bind(&TestFixture::createMotion, this), TestFixture::kLimitRate,
      getCutoffFreq(TestFixture::kFilter)));
}

TYPED_TEST(ControlLoops, CanConstructWithMotionCallbackAndControllerMode) {
  MockRobotControl robot;
  EXPECT_CALL(
      robot, startMotion(Move::ControllerMode::kCartesianImpedance, this->kMotionGeneratorMode,
                         TestFixture::Loop::kDefaultDeviation, TestFixture::Loop::kDefaultDeviation,
                         kUseNoAsyncMotionGenerator, kNoMaximumVelocities))
      .WillOnce(Return(200));

  EXPECT_NO_THROW(typename TestFixture::Loop(
      robot, ControllerMode::kCartesianImpedance, std::bind(&TestFixture::createMotion, this),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter)));
}

TYPED_TEST(ControlLoops, CanConstructWithControlCallback) {
  MockRobotControl robot;
  EXPECT_CALL(
      robot,
      startMotion(Move::ControllerMode::kExternalController, Move::MotionGeneratorMode::kNone,
                  TestFixture::Loop::kDefaultDeviation, TestFixture::Loop::kDefaultDeviation,
                  kUseNoAsyncMotionGenerator, kNoMaximumVelocities))
      .WillOnce(Return(200));

  EXPECT_NO_THROW(typename TestFixture::Loop(
      robot,
      [](const RobotState&, Duration) {
        return Torques({0, 1, 2, 3, 4, 5, 6});
      },
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter)));
}

TYPED_TEST(ControlLoops, SpinOnceWithMotionCallbackAndControllerMode) {
  StrictMock<MockRobotControl> robot;
  setupJointVelocityLimitsMock(robot);
  EXPECT_CALL(
      robot, startMotion(Move::ControllerMode::kJointImpedance, this->kMotionGeneratorMode,
                         TestFixture::Loop::kDefaultDeviation, TestFixture::Loop::kDefaultDeviation,
                         kUseNoAsyncMotionGenerator, kNoMaximumVelocities))
      .WillOnce(Return(200));

  MockMotionCallback<typename TestFixture::TMotion> motion_callback;

  auto motion = this->createMotion();

  RobotState robot_state = generateValidRobotState();
  Duration duration(1);
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(motion));

  typename TestFixture::Loop loop(
      robot, ControllerMode::kJointImpedance,
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

  auto [motion_command, control_command] = randomRobotCommand();
  EXPECT_TRUE(loop.createMotionCommand(robot_state, duration, motion_command.value()));
  EXPECT_THAT(motion_command.value(), this->getField(motion));
}

TYPED_TEST(ControlLoops, SpinOnceWithMotionAndControllerCallback) {
  StrictMock<MockRobotControl> robot;
  setupJointVelocityLimitsMock(robot);
  EXPECT_CALL(
      robot, startMotion(Move::ControllerMode::kExternalController, this->kMotionGeneratorMode,
                         TestFixture::Loop::kDefaultDeviation, TestFixture::Loop::kDefaultDeviation,
                         kUseNoAsyncMotionGenerator, kNoMaximumVelocities))
      .WillOnce(Return(200));

  MockControlCallback control_callback;
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;

  Torques torques({0, 1, 2, 3, 4, 5, 6});
  Torques torques_limited({0, 1, 1, 1, 1, 1, 1});
  auto motion = this->createMotion();

  RobotState robot_state = generateValidRobotState();
  Duration duration(2);
  EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(torques));
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(motion));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

  auto [motion_command, control_command] = randomRobotCommand();
  EXPECT_TRUE(loop.createMotionCommand(robot_state, duration, motion_command.value()));
  EXPECT_TRUE(loop.createControlCommand(robot_state, duration, control_command.value()));
  EXPECT_THAT(motion_command.value(), this->getField(motion));
  if (TestFixture::kLimitRate) {
    if (!TestFixture::kFilter) {
      for (size_t i = 0; i < torques_limited.tau_J.size(); i++) {
        EXPECT_NEAR(torques_limited.tau_J[i], control_command->tau_J_d[i], 1e-5);
      }
    } else {
      for (size_t i = 0; i < torques_limited.tau_J.size(); i++) {
        EXPECT_PRED_FORMAT2(testing::DoubleLE, control_command->tau_J_d[i],
                            torques_limited.tau_J[i]);
      }
    }
  } else {
    if (!TestFixture::kFilter) {
      EXPECT_EQ(torques.tau_J, control_command->tau_J_d);
    } else {
      for (size_t i = 0; i < torques.tau_J.size(); i++) {
        EXPECT_PRED_FORMAT2(testing::DoubleLE, control_command->tau_J_d[i], torques.tau_J[i]);
      }
    }
  }
}

TYPED_TEST(ControlLoops, SpinOnceWithInvalidMotionAndControllerCallback) {
  std::array<double, 3> invalid_values = {std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::signaling_NaN(),
                                          std::numeric_limits<double>::infinity()};
  for (size_t i = 0; i < invalid_values.size(); i++) {
    StrictMock<MockRobotControl> robot;
    setupJointVelocityLimitsMock(robot);
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation, kUseNoAsyncMotionGenerator,
                                   kNoMaximumVelocities))
        .WillOnce(Return(200));
    MockControlCallback control_callback;
    MockMotionCallback<typename TestFixture::TMotion> motion_callback;
    Torques torques({0, invalid_values[i], 2, 3, 4, 5, 6});

    RobotState robot_state = generateValidRobotState();
    Duration duration(2);
    EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(torques));
    EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration))
        .WillOnce(Return(this->createInvalidMotion(invalid_values[i])));

    typename TestFixture::Loop loop(
        robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
        std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
        TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

    auto [motion_command, control_command] = randomRobotCommand();

    EXPECT_THROW(loop.createMotionCommand(robot_state, duration, motion_command.value()),
                 std::invalid_argument);
    EXPECT_THROW(loop.createControlCommand(robot_state, duration, control_command.value()),
                 std::invalid_argument);
  }
}

TYPED_TEST(ControlLoops, SpinOnceWithFinishingMotionCallback) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation, kUseNoAsyncMotionGenerator,
                                   kNoMaximumVelocities))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, finishMotion(200, _, _));
  }

  NiceMock<MockControlCallback> control_callback;
  ON_CALL(control_callback, invoke(_, _)).WillByDefault(Return(Torques({0, 1, 2, 3, 4, 5, 6})));

  RobotState robot_state = generateValidRobotState();

  Duration duration(3);
  Duration zero_duration(0);
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration))
      .WillOnce(Return(MotionFinished(this->createMotion())));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));
  auto control_command = ControllerCommand{};
  EXPECT_TRUE(loop.createControlCommand(robot_state, duration, control_command));

  // Use ASSERT to abort on failure because loop() in next line
  // would block otherwise
  auto motion_command = MotionGeneratorCommand{};
  motion_command.motion_generation_finished = false;
  ASSERT_FALSE(loop.createMotionCommand(robot_state, duration, motion_command));

  EXPECT_CALL(robot, updateMotion(_, _)).WillOnce(Return(robot_state));
  EXPECT_CALL(motion_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(MotionFinished(this->createMotion()))));
  loop.loop();
}

TYPED_TEST(ControlLoops, LoopWithThrowingMotionCallback) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation, kUseNoAsyncMotionGenerator,
                                   kNoMaximumVelocities))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, cancelMotion(200));
  }

  NiceMock<MockControlCallback> control_callback;
  ON_CALL(control_callback, invoke(_, _)).WillByDefault(Return(Torques({0, 1, 2, 3, 4, 5, 6})));

  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(_, _)).WillOnce(Throw(std::domain_error("")));

  try {
    typename TestFixture::Loop loop(
        robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
        std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
        TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

    loop.loop();
  } catch (const std::domain_error&) {
  }
}

TYPED_TEST(ControlLoops, SpinOnceWithFinishingMotionCallbackAndControllerMode) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    auto motion_id = 200;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kCartesianImpedance,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation, kUseNoAsyncMotionGenerator,
                                   kNoMaximumVelocities))
        .WillOnce(Return(motion_id));
    EXPECT_CALL(robot, finishMotion(motion_id, _, testing::Eq(std::nullopt)));
  }

  RobotState robot_state = generateValidRobotState();
  Duration duration(4);
  Duration zero_duration(0);
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration))
      .WillOnce(Return(MotionFinished(this->createMotion())));

  typename TestFixture::Loop loop(
      robot, ControllerMode::kCartesianImpedance,
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

  // Use ASSERT to abort on failure because loop() in next line would block otherwise.
  auto motion_command = MotionGeneratorCommand{};
  ASSERT_FALSE(loop.createMotionCommand(robot_state, duration, motion_command));

  EXPECT_CALL(robot, updateMotion(_, _)).WillOnce(Return(robot_state));
  EXPECT_CALL(motion_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(MotionFinished(this->createMotion()))));
  loop.loop();
}

TYPED_TEST(ControlLoops, LoopWithThrowingMotionCallbackAndControllerMode) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kJointImpedance,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation, kUseNoAsyncMotionGenerator,
                                   kNoMaximumVelocities))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, cancelMotion(200));
  }

  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(_, _)).WillOnce(Throw(std::domain_error("")));

  try {
    typename TestFixture::Loop loop(
        robot, ControllerMode::kJointImpedance,
        std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
        TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

    loop.loop();
  } catch (const std::domain_error&) {
  }
}

TYPED_TEST(ControlLoops, SpinOnceWithFinishingControlCallback) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(
        robot,
        startMotion(Move::ControllerMode::kExternalController, Move::MotionGeneratorMode::kNone,
                    TestFixture::Loop::kDefaultDeviation, TestFixture::Loop::kDefaultDeviation,
                    kUseNoAsyncMotionGenerator, kNoMaximumVelocities))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, finishMotion(200, _, _));
  }

  MockControlCallback control_callback;
  RobotState robot_state = generateValidRobotState();
  Duration duration(5);
  Duration zero_duration(0);
  Torques zero_torques{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration))
      .WillOnce(Return(MotionFinished(zero_torques)));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

  auto control_command = ControllerCommand{};
  ASSERT_FALSE(loop.createControlCommand(robot_state, duration, control_command));

  EXPECT_CALL(robot, updateMotion(_, _)).WillOnce(Return(robot_state));
  EXPECT_CALL(control_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(MotionFinished(zero_torques))));
  loop.loop();
}

TYPED_TEST(ControlLoops, LoopWithThrowingControlCallback) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(
        robot,
        startMotion(Move::ControllerMode::kExternalController, Move::MotionGeneratorMode::kNone,
                    TestFixture::Loop::kDefaultDeviation, TestFixture::Loop::kDefaultDeviation,
                    kUseNoAsyncMotionGenerator, kNoMaximumVelocities))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, cancelMotion(200));
  }

  RobotState robot_state = generateValidRobotState();
  ON_CALL(robot, updateMotion(_, _)).WillByDefault(Return(robot_state));

  NiceMock<MockControlCallback> control_callback;
  EXPECT_CALL(control_callback, invoke(_, _)).WillOnce(Throw(std::domain_error("")));

  try {
    typename TestFixture::Loop loop(
        robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
        TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));
    loop.loop();
  } catch (const std::domain_error&) {
  }
}

TYPED_TEST(ControlLoops, GetsCorrectControlTimeStepWithMotionAndControlCallback) {
  NiceMock<MockRobotControl> robot;

  NiceMock<MockMotionCallback<typename TestFixture::TMotion>> motion_callback;
  ON_CALL(motion_callback, invoke(_, _)).WillByDefault(Return(this->createMotion()));

  MockControlCallback control_callback;
  RobotState robot_state = generateValidRobotState();

  robot_state.time = Duration(10);
  Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<uint64_t, 5> ticks{{0, 2, 1, 3, 5}};

  size_t control_count = 0;
  EXPECT_CALL(control_callback, invoke(_, _))
      .Times(ticks.size())
      .WillRepeatedly(Invoke([&](const RobotState&, Duration duration) -> Torques {
        EXPECT_EQ(ticks.at(control_count), duration.toMSec());

        if (++control_count == ticks.size()) {
          return MotionFinished(zero_torques);
        }
        return zero_torques;
      }));
  size_t robot_count = 0;
  EXPECT_CALL(robot, updateMotion(_, _))
      .Times(ticks.size())
      .WillRepeatedly(Invoke([&](const std::optional<MotionGeneratorCommand>&,
                                 const std::optional<ControllerCommand>&) {
        robot_state.time += Duration(ticks.at(robot_count));
        robot_count++;
        return robot_state;
      }));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));
  loop.loop();
}

TYPED_TEST(ControlLoops, GetsCorrectMotionTimeStepWithMotionAndControlCallback) {
  NiceMock<MockRobotControl> robot;

  NiceMock<MockControlCallback> control_callback;
  Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ON_CALL(control_callback, invoke(_, _)).WillByDefault(Return(zero_torques));

  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  RobotState robot_state = generateValidRobotState();
  robot_state.time = Duration(10);
  std::array<uint64_t, 5> ticks{{0, 2, 1, 3, 5}};

  size_t control_count = 0;
  EXPECT_CALL(motion_callback, invoke(_, _))
      .Times(ticks.size())
      .WillRepeatedly(
          Invoke([&](const RobotState&, Duration duration) -> typename TestFixture::TMotion {
            EXPECT_EQ(ticks.at(control_count), duration.toMSec());

            if (++control_count == ticks.size()) {
              return MotionFinished(this->createMotion());
            }
            return this->createMotion();
          }));

  size_t robot_count = 0;
  EXPECT_CALL(robot, updateMotion(_, _))
      .Times(ticks.size())
      .WillRepeatedly(Invoke([&](const std::optional<MotionGeneratorCommand>&,
                                 const std::optional<ControllerCommand>&) {
        robot_state.time += Duration(ticks.at(robot_count));
        robot_count++;
        return robot_state;
      }));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));
  loop.loop();
}

TYPED_TEST(ControlLoops, GetsCorrectTimeStepWithMotionCallback) {
  NiceMock<MockRobotControl> robot;

  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  RobotState robot_state = generateValidRobotState();
  robot_state.time = Duration(10);
  std::array<uint64_t, 5> ticks{{0, 2, 1, 3, 5}};

  size_t control_count = 0;
  EXPECT_CALL(motion_callback, invoke(_, _))
      .Times(ticks.size())
      .WillRepeatedly(
          Invoke([&](const RobotState&, Duration duration) -> typename TestFixture::TMotion {
            EXPECT_EQ(ticks.at(control_count), duration.toMSec());

            if (++control_count == ticks.size()) {
              return MotionFinished(this->createMotion());
            }
            return this->createMotion();
          }));
  size_t robot_count = 0;
  EXPECT_CALL(robot, updateMotion(_, _))
      .Times(ticks.size())
      .WillRepeatedly(Invoke([&](const std::optional<MotionGeneratorCommand>&,
                                 const std::optional<ControllerCommand>&) {
        robot_state.time += Duration(ticks.at(robot_count));
        robot_count++;
        return robot_state;
      }));

  typename TestFixture::Loop loop(
      robot, ControllerMode::kJointImpedance,
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

  loop.loop();
}

using CartesianPoseMotionTypes = ::testing::Types<CartesianPoseMotion<false, true>,
                                                  CartesianPoseMotionWithElbow<false, true>,
                                                  CartesianPoseMotion<true, true>,
                                                  CartesianPoseMotionWithElbow<true, true>,
                                                  CartesianPoseMotion<false, false>,
                                                  CartesianPoseMotionWithElbow<false, false>,
                                                  CartesianPoseMotion<true, false>,
                                                  CartesianPoseMotionWithElbow<true, false>>;

TYPED_TEST_CASE(ControlLoopWithTransformationMatrix, CartesianPoseMotionTypes);

TYPED_TEST(ControlLoopWithTransformationMatrix, SpinOnceWithInvalidTransformationMatrix) {
  StrictMock<MockRobotControl> robot;
  setupJointVelocityLimitsMock(robot);
  EXPECT_CALL(
      robot, startMotion(Move::ControllerMode::kExternalController, this->kMotionGeneratorMode,
                         TestFixture::Loop::kDefaultDeviation, TestFixture::Loop::kDefaultDeviation,
                         kUseNoAsyncMotionGenerator, kNoMaximumVelocities))
      .WillOnce(Return(200));
  MockControlCallback control_callback;
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  Torques torques({0, 1, 2, 3, 4, 5, 6});

  RobotState robot_state = generateValidRobotState();
  Duration duration(2);
  EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(torques));
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration))
      .WillOnce(Return(this->createInvalidTransformationMatrix()));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
      TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

  auto [motion_command, control_command] = randomRobotCommand();

  EXPECT_THROW(loop.createMotionCommand(robot_state, duration, motion_command.value()),
               std::invalid_argument);
  EXPECT_TRUE(loop.createControlCommand(robot_state, duration, control_command.value()));
}

using ElbowMotionTypes = ::testing::Types<CartesianPoseMotionWithElbow<false, true>,
                                          CartesianVelocityMotionWithElbow<false, true>,
                                          CartesianPoseMotionWithElbow<true, true>,
                                          CartesianVelocityMotionWithElbow<true, true>,
                                          CartesianPoseMotionWithElbow<false, false>,
                                          CartesianVelocityMotionWithElbow<false, false>,
                                          CartesianPoseMotionWithElbow<true, false>,
                                          CartesianVelocityMotionWithElbow<true, false>>;

TYPED_TEST_CASE(ControlLoopWithElbow, ElbowMotionTypes);

TYPED_TEST(ControlLoopWithElbow, SpinOnceWithInvalidElbowCallback) {
  std::array<double, 3> invalid_values = {std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::signaling_NaN(),
                                          std::numeric_limits<double>::infinity()};
  for (size_t i = 0; i < invalid_values.size(); i++) {
    StrictMock<MockRobotControl> robot;
    setupJointVelocityLimitsMock(robot);
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation, kUseNoAsyncMotionGenerator,
                                   kNoMaximumVelocities))
        .WillOnce(Return(200));
    MockControlCallback control_callback;
    MockMotionCallback<typename TestFixture::TMotion> motion_callback;
    Torques torques({0, 1, 2, 3, 4, 5, 6});

    RobotState robot_state = generateValidRobotState();
    Duration duration(2);
    EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(torques));
    EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration))
        .WillOnce(Return(this->createElbowConfig(invalid_values[i], invalid_values[i])));

    typename TestFixture::Loop loop(
        robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
        std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2),
        TestFixture::kLimitRate, getCutoffFreq(TestFixture::kFilter));

    auto [motion_command, control_command] = randomRobotCommand();

    EXPECT_THROW(loop.createMotionCommand(robot_state, duration, motion_command.value()),
                 std::invalid_argument);
    EXPECT_TRUE(loop.createControlCommand(robot_state, duration, control_command.value()));
  }
}
