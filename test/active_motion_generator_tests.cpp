// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gmock/gmock.h>

#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <robot_impl.h>

#include "helpers.h"
#include "mock_robot.h"
#include "mock_robot_impl.h"
#include "mock_server.h"

using ::testing::Matcher;

using namespace research_interface;

using namespace franka;

template <typename MotionGeneratorType>
class ActiveMotionGeneratorTest : public ::testing::Test {
 public:
  ActiveMotionGeneratorTest()
      : robot_impl_mock_(std::make_shared<RobotImplMock>(
            std::move(std::make_unique<Network>("127.0.0.1", robot::kCommandPort)),
            0,
            RealtimeConfig::kIgnore)),
        robot_(RobotMock(robot_impl_mock_)){};

  std::unique_ptr<ActiveControlBase> startControl(
      research_interface::robot::Move::ControllerMode controller_mode);

  using CurrentMotionGeneratorType = MotionGeneratorType;

 protected:
  RobotMockServer server_;
  std::shared_ptr<RobotImplMock> robot_impl_mock_;
  RobotMock robot_;

  const Torques default_torques{{0, 0, 0, 0, 0, 0, 0}};
  const JointPositions default_joint_positions{{0, 0, 0, 0, 0, 0, 0}};
  const JointVelocities default_joint_velocities{{0, 0, 0, 0, 0, 0, 0}};
  const CartesianPose default_cartesian_pose{{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
  const CartesianVelocities default_cartesian_velocities{{0, 0, 0, 0, 0, 0}};

  const uint32_t default_motion_id{100};
};

template <>
std::unique_ptr<ActiveControlBase> ActiveMotionGeneratorTest<JointPositions>::startControl(
    research_interface::robot::Move::ControllerMode controller_mode) {
  EXPECT_CALL(*robot_impl_mock_,
              startMotion(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(1)
      .WillOnce(::testing::Return(default_motion_id));

  return robot_.startJointPositionControl(controller_mode);
}

template <>
std::unique_ptr<ActiveControlBase> ActiveMotionGeneratorTest<JointVelocities>::startControl(
    research_interface::robot::Move::ControllerMode controller_mode) {
  EXPECT_CALL(*robot_impl_mock_,
              startMotion(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(1)
      .WillOnce(::testing::Return(default_motion_id));

  return robot_.startJointVelocityControl(controller_mode);
}

template <>
std::unique_ptr<ActiveControlBase> ActiveMotionGeneratorTest<CartesianPose>::startControl(
    research_interface::robot::Move::ControllerMode controller_mode) {
  EXPECT_CALL(*robot_impl_mock_,
              startMotion(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(1)
      .WillOnce(::testing::Return(default_motion_id));

  return robot_.startCartesianPoseControl(controller_mode);
}

template <>
std::unique_ptr<ActiveControlBase> ActiveMotionGeneratorTest<CartesianVelocities>::startControl(
    research_interface::robot::Move::ControllerMode controller_mode) {
  EXPECT_CALL(*robot_impl_mock_,
              startMotion(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(1)
      .WillOnce(::testing::Return(default_motion_id));

  return robot_.startCartesianVelocityControl(controller_mode);
}

using motion_generator_type_list =
    testing::Types<JointPositions, JointVelocities, CartesianPose, CartesianVelocities>;

TYPED_TEST_CASE(ActiveMotionGeneratorTest, motion_generator_type_list);

TYPED_TEST(ActiveMotionGeneratorTest, CanWriteOnceIfControlNotFinished) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;

  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kCartesianImpedance);

  EXPECT_CALL(*(this->robot_impl_mock_), cancelMotion(this->default_motion_id)).Times(1);
  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value ||
      std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0, 0}};
    EXPECT_CALL(*(this->robot_impl_mock_),
                writeOnce(Matcher<const CurrentMotionGeneratorType&>(testing::_)))
        .Times(1);
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianPose>::value) {
    CurrentMotionGeneratorType motion_generator_output{
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    EXPECT_CALL(*(this->robot_impl_mock_),
                writeOnce(Matcher<const CurrentMotionGeneratorType&>(testing::_)))
        .Times(1);
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
  } else {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0}};
    EXPECT_CALL(*(this->robot_impl_mock_),
                writeOnce(Matcher<const CurrentMotionGeneratorType&>(testing::_)))
        .Times(1);
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
  }
}

TYPED_TEST(ActiveMotionGeneratorTest, CanWriteOnceWithExternalControllerIfControlNotFinished) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;

  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kExternalController);

  EXPECT_CALL(*(this->robot_impl_mock_), cancelMotion(this->default_motion_id)).Times(1);
  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value ||
      std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0, 0}};
    EXPECT_CALL(*(this->robot_impl_mock_),
                writeOnce(Matcher<const CurrentMotionGeneratorType&>(testing::_),
                          Matcher<const Torques&>(testing::_)))
        .Times(1);
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianPose>::value) {
    CurrentMotionGeneratorType motion_generator_output{
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    EXPECT_CALL(*(this->robot_impl_mock_),
                writeOnce(Matcher<const CurrentMotionGeneratorType&>(testing::_),
                          Matcher<const Torques&>(testing::_)))
        .Times(1);
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
  } else {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0}};
    EXPECT_CALL(*(this->robot_impl_mock_),
                writeOnce(Matcher<const CurrentMotionGeneratorType&>(testing::_),
                          Matcher<const Torques&>(testing::_)))
        .Times(1);
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
  }
}

TYPED_TEST(ActiveMotionGeneratorTest, CanCallFinishMotionWhenFinished) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;
  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kCartesianImpedance);

  EXPECT_CALL(*(this->robot_impl_mock_),
              finishMotion(this->default_motion_id, testing::_, testing::_))
      .Times(1);

  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value ||
      std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianPose>::value) {
    CurrentMotionGeneratorType motion_generator_output{
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
  } else {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
  }
}

TYPED_TEST(ActiveMotionGeneratorTest, CanCallFinishMotionWithExternalControllerWhenFinished) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;
  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kExternalController);

  EXPECT_CALL(*(this->robot_impl_mock_),
              finishMotion(this->default_motion_id, testing::_, testing::_))
      .Times(1);

  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value ||
      std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianPose>::value) {
    CurrentMotionGeneratorType motion_generator_output{
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
  } else {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
  }
}

TYPED_TEST(ActiveMotionGeneratorTest, CanNotWriteOnceIfControlFinished) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;
  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kCartesianImpedance);

  EXPECT_CALL(*(this->robot_impl_mock_),
              finishMotion(this->default_motion_id, testing::_, testing::_))
      .Times(1);

  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value ||
      std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
    EXPECT_THROW(active_control->writeOnce(motion_generator_output), ControlException);
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianPose>::value) {
    CurrentMotionGeneratorType motion_generator_output{
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
    EXPECT_THROW(active_control->writeOnce(motion_generator_output), ControlException);
  } else {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
    EXPECT_THROW(active_control->writeOnce(motion_generator_output), ControlException);
  }
}

TYPED_TEST(ActiveMotionGeneratorTest, CanNotWriteOnceWithExternalControllerIfControlFinished) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;
  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kExternalController);

  EXPECT_CALL(*(this->robot_impl_mock_),
              finishMotion(this->default_motion_id, testing::_, testing::_))
      .Times(1);

  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value ||
      std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
    EXPECT_THROW(active_control->writeOnce(motion_generator_output, this->default_torques),
                 ControlException);
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianPose>::value) {
    CurrentMotionGeneratorType motion_generator_output{
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
    EXPECT_THROW(active_control->writeOnce(motion_generator_output, this->default_torques),
                 ControlException);
  } else {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
    EXPECT_THROW(active_control->writeOnce(motion_generator_output, this->default_torques),
                 ControlException);
  }
}

TYPED_TEST(ActiveMotionGeneratorTest, ControlTokenReleasedAfterFinishingControl) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;
  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kCartesianImpedance);

  EXPECT_CALL(*(this->robot_impl_mock_),
              finishMotion(this->default_motion_id, testing::_, testing::_))
      .Times(1);

  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value ||
      std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianPose>::value) {
    CurrentMotionGeneratorType motion_generator_output{
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
  } else {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output));
  }

  EXPECT_CALL(*(this->robot_impl_mock_), cancelMotion(this->default_motion_id)).Times(1);
  EXPECT_NO_THROW(
      this->startControl(research_interface::robot::Move::ControllerMode::kCartesianImpedance));
}

TYPED_TEST(ActiveMotionGeneratorTest,
           ControlTokenReleasedWithExternalControllerAfterFinishingControl) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;
  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kExternalController);

  EXPECT_CALL(*(this->robot_impl_mock_),
              finishMotion(this->default_motion_id, testing::_, testing::_))
      .Times(1);

  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value ||
      std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianPose>::value) {
    CurrentMotionGeneratorType motion_generator_output{
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
  } else {
    CurrentMotionGeneratorType motion_generator_output{{0, 0, 0, 0, 0, 0}};
    motion_generator_output.motion_finished = true;
    EXPECT_NO_THROW(active_control->writeOnce(motion_generator_output, this->default_torques));
  }

  EXPECT_CALL(*(this->robot_impl_mock_), cancelMotion(this->default_motion_id)).Times(1);
  EXPECT_NO_THROW(
      this->startControl(research_interface::robot::Move::ControllerMode::kExternalController));
}

TYPED_TEST(ActiveMotionGeneratorTest, UnintendedWriteOnceMethodsThrowException) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;

  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kCartesianImpedance);

  EXPECT_CALL(*(this->robot_impl_mock_), cancelMotion(this->default_motion_id)).Times(1);

  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value) {
    EXPECT_THROW(active_control->writeOnce(this->default_joint_velocities), ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_cartesian_pose), ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_cartesian_velocities), ControlException);
  } else if (std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    EXPECT_THROW(active_control->writeOnce(this->default_joint_positions), ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_cartesian_pose), ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_cartesian_velocities), ControlException);
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianVelocities>::value) {
    EXPECT_THROW(active_control->writeOnce(this->default_joint_positions), ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_joint_velocities), ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_cartesian_pose), ControlException);
  } else {
    EXPECT_THROW(active_control->writeOnce(this->default_joint_positions), ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_joint_velocities), ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_cartesian_velocities), ControlException);
  }

  EXPECT_THROW(active_control->writeOnce(this->default_joint_velocities, this->default_torques),
               ControlException);
  EXPECT_THROW(active_control->writeOnce(this->default_cartesian_pose, this->default_torques),
               ControlException);
  EXPECT_THROW(active_control->writeOnce(this->default_cartesian_velocities, this->default_torques),
               ControlException);
  EXPECT_THROW(active_control->writeOnce(this->default_cartesian_velocities, this->default_torques),
               ControlException);
}

TYPED_TEST(ActiveMotionGeneratorTest,
           UnintendedWriteOnceMethodsWithExternalControllerThrowException) {
  using CurrentMotionGeneratorType = typename TestFixture::CurrentMotionGeneratorType;

  auto active_control =
      this->startControl(research_interface::robot::Move::ControllerMode::kExternalController);

  EXPECT_CALL(*(this->robot_impl_mock_), cancelMotion(this->default_motion_id)).Times(1);

  if (std::is_same<CurrentMotionGeneratorType, JointPositions>::value) {
    EXPECT_THROW(active_control->writeOnce(this->default_joint_velocities, this->default_torques),
                 ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_cartesian_pose, this->default_torques),
                 ControlException);
    EXPECT_THROW(
        active_control->writeOnce(this->default_cartesian_velocities, this->default_torques),
        ControlException);
  } else if (std::is_same<CurrentMotionGeneratorType, JointVelocities>::value) {
    EXPECT_THROW(active_control->writeOnce(this->default_joint_positions, this->default_torques),
                 ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_cartesian_pose, this->default_torques),
                 ControlException);
    EXPECT_THROW(
        active_control->writeOnce(this->default_cartesian_velocities, this->default_torques),
        ControlException);
  } else if (std::is_same<CurrentMotionGeneratorType, CartesianVelocities>::value) {
    EXPECT_THROW(active_control->writeOnce(this->default_joint_positions, this->default_torques),
                 ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_joint_velocities, this->default_torques),
                 ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_cartesian_pose, this->default_torques),
                 ControlException);
  } else {
    EXPECT_THROW(active_control->writeOnce(this->default_joint_positions, this->default_torques),
                 ControlException);
    EXPECT_THROW(active_control->writeOnce(this->default_joint_velocities, this->default_torques),
                 ControlException);
    EXPECT_THROW(
        active_control->writeOnce(this->default_cartesian_velocities, this->default_torques),
        ControlException);
  }

  EXPECT_THROW(active_control->writeOnce(this->default_joint_velocities), ControlException);
  EXPECT_THROW(active_control->writeOnce(this->default_cartesian_pose), ControlException);
  EXPECT_THROW(active_control->writeOnce(this->default_cartesian_velocities), ControlException);
  EXPECT_THROW(active_control->writeOnce(this->default_cartesian_velocities), ControlException);
}
