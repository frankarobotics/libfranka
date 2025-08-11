// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gmock/gmock.h>

#include <franka/active_control.h>
#include <franka/active_torque_control.h>
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

class ActiveTorqueControlTest : public ::testing::Test {
 public:
  ActiveTorqueControlTest()
      : robot_impl_mock(std::make_shared<RobotImplMock>(
            std::move(std::make_unique<Network>("127.0.0.1", robot::kCommandPort)),
            0,
            RealtimeConfig::kIgnore)),
        robot(RobotMock(robot_impl_mock)){};

  std::unique_ptr<ActiveControlBase> startTorqueControl() {
    EXPECT_CALL(*robot_impl_mock,
                startMotion(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
        .Times(1)
        .WillOnce(::testing::Return(100));

    return robot.startTorqueControl();
  }

 protected:
  RobotMockServer server;
  std::shared_ptr<RobotImplMock> robot_impl_mock;
  RobotMock robot;
};

TEST_F(ActiveTorqueControlTest, CanWriteOnceIfControlNotFinished) {
  auto active_control = startTorqueControl();
  Torques default_control_output{{0, 0, 0, 0, 0, 0, 0}};

  EXPECT_CALL(*robot_impl_mock, cancelMotion(100)).Times(1);
  EXPECT_CALL(*robot_impl_mock, writeOnce(Matcher<const Torques&>(testing::_))).Times(1);
  EXPECT_NO_THROW(active_control->writeOnce(default_control_output));
}

TEST_F(ActiveTorqueControlTest, CanCallFinishMotionWhenFinished) {
  auto active_control = startTorqueControl();
  Torques default_control_output{{0, 0, 0, 0, 0, 0, 0}};
  default_control_output.motion_finished = true;

  EXPECT_CALL(*robot_impl_mock, finishMotion(100, testing::_, testing::_)).Times(1);
  EXPECT_NO_THROW(active_control->writeOnce(default_control_output));
}

TEST_F(ActiveTorqueControlTest, CanNotWriteOnceIfControlFinished) {
  auto active_control = startTorqueControl();
  Torques default_control_output{{0, 0, 0, 0, 0, 0, 0}};
  default_control_output.motion_finished = true;

  EXPECT_CALL(*robot_impl_mock, finishMotion(100, testing::_, testing::_)).Times(1);
  EXPECT_NO_THROW(active_control->writeOnce(default_control_output));
  EXPECT_THROW(active_control->writeOnce(default_control_output), ControlException);
}

TEST_F(ActiveTorqueControlTest, ControlTokenReleasedAfterFinishingControl) {
  auto active_control = startTorqueControl();
  Torques default_control_output{{0, 0, 0, 0, 0, 0, 0}};
  default_control_output.motion_finished = true;

  EXPECT_CALL(*robot_impl_mock, finishMotion(100, testing::_, testing::_)).Times(1);
  EXPECT_NO_THROW(active_control->writeOnce(default_control_output));
  EXPECT_CALL(*robot_impl_mock, cancelMotion(100)).Times(1);
  EXPECT_NO_THROW(startTorqueControl());
}

TEST_F(ActiveTorqueControlTest, CanReadOnce) {
  auto active_control = startTorqueControl();
  const Duration time_first_read(1);
  const Duration time_second_read(5);
  const RobotState kFirstExpectedRobotState{.time = time_first_read};
  const RobotState kSecondExpectedRobotState{.time = time_second_read};

  EXPECT_CALL(*robot_impl_mock, readOnce())
      .Times(2)
      .WillOnce(::testing::Return(kFirstExpectedRobotState))
      .WillOnce(::testing::Return(kSecondExpectedRobotState));
  EXPECT_CALL(*robot_impl_mock, cancelMotion(100)).Times(1);
  auto& first_throw_motion_error_call =
      EXPECT_CALL(*robot_impl_mock, throwOnMotionError(kFirstExpectedRobotState, 100)).Times(1);
  EXPECT_CALL(*robot_impl_mock, throwOnMotionError(kSecondExpectedRobotState, 100))
      .Times(1)
      .After(first_throw_motion_error_call);

  auto [received_robot_state, received_period] = active_control->readOnce();
  EXPECT_EQ(received_robot_state, kFirstExpectedRobotState);
  EXPECT_EQ(received_period, Duration(0));

  std::tie(received_robot_state, received_period) = active_control->readOnce();
  EXPECT_EQ(received_robot_state, kSecondExpectedRobotState);
  EXPECT_EQ(received_period, time_second_read - time_first_read);
}
