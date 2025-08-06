// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <limits>

#include <franka/active_control.h>
#include <robot_impl.h>
#include <logging/robot_state_logger.hpp>

#include "helpers.h"
#include "mock_server.h"

using namespace std::chrono_literals;
using namespace research_interface::robot;

using franka::ControlException;
using franka::NetworkException;

constexpr bool kUseNoAsyncMotionGenerator = false;
const std::optional<std::vector<double>> kNoMaximumVelocities = std::nullopt;

struct Robot : public ::franka::Robot {
  struct Impl : public ::franka::Robot::Impl {
    using ::franka::Robot::Impl::controllerRunning;
    using ::franka::Robot::Impl::Impl;
    using ::franka::Robot::Impl::motionGeneratorRunning;
  };
};

class RobotImplTests : public testing::Test {
 public:
  RobotImplTests()
      : default_server(),
        default_robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort),
                      0,
                      franka::RealtimeConfig::kIgnore) {}

 protected:
  // Server needs to be initialized in order for the robot to connect to it.
  RobotMockServer default_server;
  Robot::Impl default_robot;
};

TEST_F(RobotImplTests, CanReceiveRobotState) {
  RobotState sent_robot_state;
  default_server.sendRandomState<RobotState>([](auto s) { randomRobotState(s); }, &sent_robot_state)
      .spinOnce();

  auto received_robot_state = default_robot.updateMotion(std::nullopt, std::nullopt);
  testRobotStatesAreEqual(sent_robot_state, received_robot_state);
}

TEST_F(RobotImplTests, CanReceiveReorderedRobotStatesCorrectly) {
  default_server.onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 2; })
      .spinOnce();

  auto received_robot_state = default_robot.updateMotion(std::nullopt, std::nullopt);
  EXPECT_EQ(2u, received_robot_state.time.toMSec());

  default_server.onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 1; })
      .onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 4; })
      .onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 2; })
      .onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 3; })
      .spinOnce();

  received_robot_state = default_robot.updateMotion(std::nullopt, std::nullopt);
  EXPECT_EQ(4u, received_robot_state.time.toMSec());
}

TEST_F(RobotImplTests, ThrowsTimeoutIfNoRobotStateArrives) {
  EXPECT_THROW(default_robot.updateMotion(std::nullopt, std::nullopt), NetworkException);
}

TEST(RobotImpl, StopsIfServerDiesInActiveControl) {
  std::unique_ptr<franka::Robot> robot;
  std::unique_ptr<franka::ActiveControlBase> rw_interface;

  {
    RobotMockServer server;
    robot.reset(new franka::Robot("127.0.0.1", franka::RealtimeConfig::kIgnore));

    server
        .onSendUDP<RobotState>([=](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kNone;
          robot_state.controller_mode = ControllerMode::kExternalController;
          robot_state.robot_mode = RobotMode::kMove;
        })
        .spinOnce()
        .waitForCommand<Move>(
            [](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); })
        .spinOnce();

    EXPECT_NO_THROW(rw_interface = robot->startTorqueControl());
  }

  EXPECT_THROW(rw_interface->readOnce(), NetworkException);
}

TEST_F(RobotImplTests, CanStartMotion) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([=](const Move::Request& request) {
        EXPECT_EQ(Move::MotionGeneratorMode::kJointPosition, request.motion_generator_mode);
        EXPECT_EQ(Move::ControllerMode::kJointImpedance, request.controller_mode);
        EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
        EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  EXPECT_NO_THROW(default_robot.startMotion(Move::ControllerMode::kJointImpedance,
                                            Move::MotionGeneratorMode::kJointPosition,
                                            maximum_path_deviation, maximum_goal_pose_deviation,
                                            kUseNoAsyncMotionGenerator, kNoMaximumVelocities));
  EXPECT_TRUE(default_robot.motionGeneratorRunning());
  EXPECT_FALSE(default_robot.controllerRunning());

  // Test exceptions if wrong update() overload is called
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce();
  EXPECT_NO_THROW(default_robot.updateMotion(std::nullopt, std::nullopt));

  auto control_command = std::optional<ControllerCommand>{ControllerCommand{}};
  auto motion_command = std::optional<MotionGeneratorCommand>{MotionGeneratorCommand{}};
  EXPECT_THROW(default_robot.updateMotion(std::nullopt, control_command), ControlException);
  EXPECT_THROW(default_robot.updateMotion(motion_command, control_command), ControlException);

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();
  EXPECT_NO_THROW(default_robot.updateMotion(motion_command, std::nullopt));

  // Test exceptions if wrong writeOnce is called
  const franka::Torques control_output{{1, 2, 3, 4, 5, 6, 7}};
  EXPECT_THROW(default_robot.writeOnce(control_output), ControlException);

  // Test exceptions if wrong finishMotion is called
  const uint32_t kDummyMotionID = 1;
  EXPECT_THROW(default_robot.finishMotion(kDummyMotionID, control_output), std::invalid_argument);
}

TEST_F(RobotImplTests, CanStartMotionWithController) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([=](const Move::Request& request) {
        EXPECT_EQ(Move::MotionGeneratorMode::kCartesianPosition, request.motion_generator_mode);
        EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
        EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
        EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  EXPECT_NO_THROW(default_robot.startMotion(Move::ControllerMode::kExternalController,
                                            Move::MotionGeneratorMode::kCartesianPosition,
                                            maximum_path_deviation, maximum_goal_pose_deviation,
                                            kUseNoAsyncMotionGenerator, kNoMaximumVelocities));
  EXPECT_TRUE(default_robot.motionGeneratorRunning());
  EXPECT_TRUE(default_robot.controllerRunning());

  // Test exceptions if wrong update() overload is called
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce();
  EXPECT_NO_THROW(default_robot.updateMotion(std::nullopt, std::nullopt));

  auto control_command = std::optional<ControllerCommand>{ControllerCommand{}};
  auto motion_command = std::optional<MotionGeneratorCommand>{MotionGeneratorCommand{}};
  EXPECT_THROW(default_robot.updateMotion(std::nullopt, control_command), ControlException);
  EXPECT_THROW(default_robot.updateMotion(motion_command, std::nullopt), ControlException);
}

TEST_F(RobotImplTests, CanStartExternalControllerMotion) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kNone;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([=](const Move::Request& request) {
        EXPECT_EQ(Move::MotionGeneratorMode::kNone, request.motion_generator_mode);
        EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
        EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
        EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  EXPECT_NO_THROW(default_robot.startMotion(Move::ControllerMode::kExternalController,
                                            Move::MotionGeneratorMode::kNone,
                                            maximum_path_deviation, maximum_goal_pose_deviation,
                                            kUseNoAsyncMotionGenerator, kNoMaximumVelocities));
  ASSERT_FALSE(default_robot.motionGeneratorRunning());
  ASSERT_TRUE(default_robot.controllerRunning());

  // Test exceptions if wrong update() overload is called
  auto control_command = std::optional<ControllerCommand>{ControllerCommand{}};
  auto motion_command = std::optional<MotionGeneratorCommand>{MotionGeneratorCommand{}};
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kNone;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([=](const RobotCommand& command) {
        testControllerCommandsAreEqual(control_command.value(), command.control);
      })
      .spinOnce();

  // Calling update with motion command should throw
  EXPECT_THROW(default_robot.updateMotion(motion_command, std::nullopt), ControlException);
  EXPECT_NO_THROW(default_robot.updateMotion(std::nullopt, control_command));

  // Test no exception if writeOnce is called
  franka::Torques control_output{{1, 2, 3, 4, 5, 6, 7}};
  auto valid_control_command =
      ControllerCommand{.tau_J_d = control_output.tau_J, .torque_command_finished = false};
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kNone;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([=](const RobotCommand& command) {
        testControllerCommandsAreEqual(valid_control_command, command.control);
      })
      .spinOnce();
  EXPECT_NO_THROW(default_robot.writeOnce(control_output));

  // Test exception if writeOnce with invalid input is called
  EXPECT_THROW(default_robot.writeOnce(franka::Torques{}), std::invalid_argument);

  // Test exception if finishMotion with invalid input is called
  const uint64_t kMessageID = 3;
  EXPECT_THROW(default_robot.finishMotion(kMessageID, franka::Torques{}), std::invalid_argument);
}

TEST_F(RobotImplTests, CanNotStartMultipleMotions) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); })
      .spinOnce();

  EXPECT_NO_THROW(default_robot.startMotion(Move::ControllerMode::kJointImpedance,
                                            Move::MotionGeneratorMode::kJointVelocity,
                                            maximum_path_deviation, maximum_goal_pose_deviation,
                                            kUseNoAsyncMotionGenerator, kNoMaximumVelocities));
  EXPECT_THROW(default_robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                                         Move::MotionGeneratorMode::kJointPosition,
                                         maximum_path_deviation, maximum_goal_pose_deviation,
                                         kUseNoAsyncMotionGenerator, kNoMaximumVelocities),
               ControlException);
  EXPECT_THROW(default_robot.startMotion(Move::ControllerMode::kExternalController,
                                         Move::MotionGeneratorMode::kJointVelocity,
                                         maximum_path_deviation, maximum_goal_pose_deviation,
                                         kUseNoAsyncMotionGenerator, kNoMaximumVelocities),
               ControlException);
}

TEST_F(RobotImplTests, CanSendMotionGeneratorCommand) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
  const uint32_t message_id = 682;

  auto [sent_motion_command, sent_control_command] = randomRobotCommand();
  sent_motion_command->motion_generation_finished = false;

  default_server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); })
      .spinOnce();

  default_robot.startMotion(Move::ControllerMode::kJointImpedance,
                            Move::MotionGeneratorMode::kJointVelocity, maximum_path_deviation,
                            maximum_goal_pose_deviation, kUseNoAsyncMotionGenerator,
                            kNoMaximumVelocities);

  default_server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id + 1;
      })
      .spinOnce()
      .onReceiveRobotCommand([=](const RobotCommand& command) {
        EXPECT_EQ(message_id, command.message_id);
        testMotionGeneratorCommandsAreEqual(sent_motion_command.value(), command.motion);
      })
      .spinOnce();

  EXPECT_NO_THROW(default_robot.updateMotion(sent_motion_command, std::nullopt));
}

TEST_F(RobotImplTests, CanSendControllerCommand) {
  const uint32_t message_id = 684;

  auto [sent_motion_command, sent_control_command] = randomRobotCommand();
  sent_motion_command->motion_generation_finished = false;

  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  default_server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>([=](const Move::Request& request) {
        EXPECT_EQ(Move::MotionGeneratorMode::kJointVelocity, request.motion_generator_mode);
        EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
        EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
        EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  EXPECT_NO_THROW(default_robot.startMotion(Move::ControllerMode::kExternalController,
                                            Move::MotionGeneratorMode::kJointVelocity,
                                            maximum_path_deviation, maximum_goal_pose_deviation,
                                            kUseNoAsyncMotionGenerator, kNoMaximumVelocities));
  EXPECT_TRUE(default_robot.motionGeneratorRunning());
  EXPECT_TRUE(default_robot.controllerRunning());

  default_server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id + 1;
      })
      .spinOnce()
      .onReceiveRobotCommand([=](const RobotCommand& command) {
        EXPECT_EQ(message_id, command.message_id);
        testControllerCommandsAreEqual(sent_control_command.value(), command.control);
        testMotionGeneratorCommandsAreEqual(sent_motion_command.value(), command.motion);
      })
      .spinOnce();

  EXPECT_NO_THROW(default_robot.updateMotion(sent_motion_command, sent_control_command));
}

TEST_F(RobotImplTests, CanSendMotionGeneratorAndControlCommand) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
  const uint32_t message_id = 687;

  auto [sent_motion_command, sent_control_command] = randomRobotCommand();
  sent_motion_command->motion_generation_finished = false;

  default_server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); })
      .spinOnce();

  default_robot.startMotion(Move::ControllerMode::kExternalController,
                            Move::MotionGeneratorMode::kCartesianPosition, maximum_path_deviation,
                            maximum_goal_pose_deviation, kUseNoAsyncMotionGenerator,
                            kNoMaximumVelocities);

  default_server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id + 1;
      })
      .spinOnce()
      .onReceiveRobotCommand([=](const RobotCommand& command) {
        EXPECT_EQ(message_id, command.message_id);
        testMotionGeneratorCommandsAreEqual(sent_motion_command.value(), command.motion);
        testControllerCommandsAreEqual(sent_control_command.value(), command.control);
      })
      .spinOnce();

  EXPECT_NO_THROW(default_robot.updateMotion(sent_motion_command, sent_control_command));
}

TEST(RobotImplTest, CanReceiveMotionRejected) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  uint32_t move_id{};

  std::atomic_flag send = ATOMIC_FLAG_INIT;
  send.test_and_set();

  {
    auto mock_server = RobotMockServer{};
    Robot::Impl robot{std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0,
                      franka::RealtimeConfig::kIgnore};
    mock_server
        .onSendUDP<RobotState>([=](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
          robot_state.controller_mode = ControllerMode::kCartesianImpedance;
          robot_state.robot_mode = RobotMode::kMove;
        })
        .spinOnce()
        .waitForCommand<Move>(
            [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
            &move_id)
        .queueResponse<Move>(
            move_id, []() { return Move::Response(Move::Status::kCommandNotPossibleRejected); })
        .doForever([&]() {
          bool continue_sending = send.test_and_set();
          if (continue_sending) {
            mock_server.onSendUDP<RobotState>([](RobotState& robot_state) {
              robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
              robot_state.controller_mode = ControllerMode::kCartesianImpedance;
              robot_state.robot_mode = RobotMode::kIdle;
            });
            std::this_thread::yield();
            std::this_thread::sleep_for(1ms);
          }
          return continue_sending;
        })
        .spinOnce();

    EXPECT_THROW(robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                                   Move::MotionGeneratorMode::kCartesianVelocity,
                                   maximum_path_deviation, maximum_goal_pose_deviation,
                                   kUseNoAsyncMotionGenerator, kNoMaximumVelocities),
                 ControlException);
    send.clear();
    EXPECT_FALSE(robot.motionGeneratorRunning());
  }
}

TEST_F(RobotImplTests, CanStopMotion) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  auto [sent_motion_command, sent_control_command] = randomRobotCommand();
  sent_motion_command->motion_generation_finished = false;

  uint32_t move_id;

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = default_robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                                      Move::MotionGeneratorMode::kCartesianVelocity,
                                      maximum_path_deviation, maximum_goal_pose_deviation,
                                      kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  default_robot.updateMotion(sent_motion_command, std::nullopt);

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kIdle;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kSuccess); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand& command) {
        EXPECT_TRUE(command.motion.motion_generation_finished);
      })
      .spinOnce();

  default_robot.finishMotion(id, sent_motion_command, std::nullopt);
  EXPECT_FALSE(default_robot.motionGeneratorRunning());
}

TEST_F(RobotImplTests, StopMotionErrorThrowsControlException) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  auto [sent_motion_command, sent_control_command] = randomRobotCommand();
  sent_motion_command->motion_generation_finished = false;

  uint32_t move_id;

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = default_robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                                      Move::MotionGeneratorMode::kCartesianVelocity,
                                      maximum_path_deviation, maximum_goal_pose_deviation,
                                      kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  default_robot.updateMotion(sent_motion_command, std::nullopt);

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kIdle;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kEmergencyAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand& command) {
        EXPECT_TRUE(command.motion.motion_generation_finished);
      })
      .spinOnce();

  EXPECT_THROW(default_robot.finishMotion(id, sent_motion_command, std::nullopt), ControlException);
  EXPECT_FALSE(default_robot.motionGeneratorRunning());
}

TEST_F(RobotImplTests, CanCancelMotion) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  uint32_t move_id{};

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = default_robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                                      Move::MotionGeneratorMode::kCartesianVelocity,
                                      maximum_path_deviation, maximum_goal_pose_deviation,
                                      kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kIdle;
      })
      .spinOnce()
      .waitForCommand<StopMove>([&](const StopMove::Request&) {
        default_server.sendResponse<Move>(
            move_id, []() { return Move::Response(Move::Status::kPreempted); });
        return StopMove::Response(StopMove::Status::kSuccess);
      })
      .spinOnce();

  default_robot.cancelMotion(id);
  EXPECT_FALSE(default_robot.motionGeneratorRunning());
}

TEST(RobotImplTest, CancelMotionErrorThrowsControlException) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  uint32_t move_id{};

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0,
                    franka::RealtimeConfig::kIgnore);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                              Move::MotionGeneratorMode::kCartesianVelocity, maximum_path_deviation,
                              maximum_goal_pose_deviation, kUseNoAsyncMotionGenerator,
                              kNoMaximumVelocities);
  EXPECT_TRUE(robot.motionGeneratorRunning());

  server
      .waitForCommand<StopMove>([&](const StopMove::Request&) {
        server.sendResponse<Move>(move_id,
                                  []() { return Move::Response(Move::Status::kPreempted); });
        return StopMove::Response(StopMove::Status::kCommandNotPossibleRejected);
      })
      .spinOnce();

  EXPECT_THROW(robot.cancelMotion(id), ControlException);
}

TEST_F(RobotImplTests, CanStopMotionWithController) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  auto [sent_motion_command, sent_control_command] = randomRobotCommand();
  sent_motion_command->motion_generation_finished = false;

  uint32_t move_id;

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = default_robot.startMotion(Move::ControllerMode::kExternalController,
                                      Move::MotionGeneratorMode::kCartesianVelocity,
                                      maximum_path_deviation, maximum_goal_pose_deviation,
                                      kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());
  EXPECT_TRUE(default_robot.controllerRunning());

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  default_robot.updateMotion(sent_motion_command, sent_control_command);

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kSuccess); })
      .spinOnce();

  default_robot.finishMotion(id, sent_motion_command, sent_control_command);
  EXPECT_FALSE(default_robot.motionGeneratorRunning());
  EXPECT_FALSE(default_robot.controllerRunning());

  default_server
      .onReceiveRobotCommand([](const RobotCommand& command) {
        EXPECT_TRUE(command.motion.motion_generation_finished);
      })
      .spinOnce();
}

TEST_F(RobotImplTests, ThrowsDuringMotionIfErrorReceived) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  uint32_t move_id;
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = default_robot.startMotion(Move::ControllerMode::kJointImpedance,
                                      Move::MotionGeneratorMode::kJointPosition,
                                      maximum_path_deviation, maximum_goal_pose_deviation,
                                      kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());

  auto motion_command = std::optional<MotionGeneratorCommand>{MotionGeneratorCommand{}};
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto robot_state = default_robot.updateMotion(motion_command, std::nullopt);
  EXPECT_THROW(default_robot.throwOnMotionError(robot_state, id), ControlException);
  EXPECT_FALSE(default_robot.motionGeneratorRunning());
}

TEST_F(RobotImplTests, ThrowsDuringControlIfErrorReceived) {
  const uint32_t message_id = 684;

  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  uint32_t move_id;
  default_server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [=](const Move::Request& request) {
            EXPECT_EQ(Move::MotionGeneratorMode::kJointVelocity, request.motion_generator_mode);
            EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
            EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
            EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
            return Move::Response(Move::Status::kMotionStarted);
          },
          &move_id)
      .spinOnce();

  auto id = default_robot.startMotion(Move::ControllerMode::kExternalController,
                                      Move::MotionGeneratorMode::kJointVelocity,
                                      maximum_path_deviation, maximum_goal_pose_deviation,
                                      kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());
  EXPECT_TRUE(default_robot.controllerRunning());

  auto motion_command = std::optional<MotionGeneratorCommand>{MotionGeneratorCommand{}};
  auto control_command = std::optional<ControllerCommand>{ControllerCommand{}};
  default_server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
        robot_state.message_id = message_id + 1;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto robot_state = default_robot.updateMotion(motion_command, control_command);
  EXPECT_THROW(default_robot.throwOnMotionError(robot_state, id), ControlException);
  EXPECT_FALSE(default_robot.controllerRunning());
}

TEST_F(RobotImplTests, CanStartConsecutiveMotion) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  auto [sent_motion_command, sent_control_command] = randomRobotCommand();
  sent_motion_command->motion_generation_finished = false;

  for (int i = 0; i < 3; i++) {
    uint32_t move_id;
    default_server
        .onSendUDP<RobotState>([](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
          robot_state.controller_mode = ControllerMode::kCartesianImpedance;
          robot_state.robot_mode = RobotMode::kMove;
        })
        .spinOnce()
        .waitForCommand<Move>(
            [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
            &move_id)
        .spinOnce();

    EXPECT_FALSE(default_robot.motionGeneratorRunning());
    auto id = default_robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                                        Move::MotionGeneratorMode::kCartesianVelocity,
                                        maximum_path_deviation, maximum_goal_pose_deviation,
                                        kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
    EXPECT_TRUE(default_robot.motionGeneratorRunning());

    default_server
        .onSendUDP<RobotState>([](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
          robot_state.controller_mode = ControllerMode::kCartesianImpedance;
          robot_state.robot_mode = RobotMode::kMove;
        })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand&) {})
        .spinOnce();

    default_robot.updateMotion(sent_motion_command, std::nullopt);
    EXPECT_TRUE(default_robot.motionGeneratorRunning());

    default_server
        .onSendUDP<RobotState>([](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
          robot_state.controller_mode = ControllerMode::kCartesianImpedance;
          robot_state.robot_mode = RobotMode::kMove;
        })
        .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kSuccess); })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand& command) {
          EXPECT_TRUE(command.motion.motion_generation_finished);
        })
        .spinOnce();

    default_robot.finishMotion(id, sent_motion_command, std::nullopt);
    EXPECT_FALSE(default_robot.motionGeneratorRunning());
  }
}

TEST_F(RobotImplTests, CanStartConsecutiveMotionAfterError) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  uint32_t move_id;
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = default_robot.startMotion(Move::ControllerMode::kJointImpedance,
                                      Move::MotionGeneratorMode::kJointPosition,
                                      maximum_path_deviation, maximum_goal_pose_deviation,
                                      kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());

  auto motion_command = std::optional<MotionGeneratorCommand>{MotionGeneratorCommand{}};
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto robot_state = default_robot.updateMotion(motion_command, std::nullopt);
  EXPECT_THROW(default_robot.throwOnMotionError(robot_state, id), ControlException);
  EXPECT_FALSE(default_robot.motionGeneratorRunning());

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [=](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); })
      .spinOnce();

  default_robot.startMotion(Move::ControllerMode::kJointImpedance,
                            Move::MotionGeneratorMode::kJointPosition, maximum_path_deviation,
                            maximum_goal_pose_deviation, kUseNoAsyncMotionGenerator,
                            kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());
}

TEST_F(RobotImplTests, CanStartConsecutiveControlAfterError) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  uint32_t move_id;
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [](const Move::Request& request) {
            EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
            return Move::Response(Move::Status::kMotionStarted);
          },
          &move_id)
      .spinOnce();

  auto id = default_robot.startMotion(Move::ControllerMode::kExternalController,
                                      Move::MotionGeneratorMode::kCartesianVelocity,
                                      maximum_path_deviation, maximum_goal_pose_deviation,
                                      kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());
  EXPECT_TRUE(default_robot.controllerRunning());

  auto motion_command = std::optional<MotionGeneratorCommand>{MotionGeneratorCommand{}};
  auto control_command = std::optional<ControllerCommand>{ControllerCommand{}};
  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto robot_state = default_robot.updateMotion(motion_command, control_command);
  EXPECT_THROW(default_robot.throwOnMotionError(robot_state, id), ControlException);
  EXPECT_FALSE(default_robot.motionGeneratorRunning());
  EXPECT_FALSE(default_robot.controllerRunning());

  default_server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([](const Move::Request& request) {
        EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);

        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  default_robot.startMotion(Move::ControllerMode::kExternalController,
                            Move::MotionGeneratorMode::kCartesianVelocity, maximum_path_deviation,
                            maximum_goal_pose_deviation, kUseNoAsyncMotionGenerator,
                            kNoMaximumVelocities);
  EXPECT_TRUE(default_robot.motionGeneratorRunning());
  EXPECT_TRUE(default_robot.controllerRunning());
}

TEST(RobotImplConnectionTests, StopsIfControlConnectionClosed) {
  std::unique_ptr<Robot::Impl> robot;
  {
    RobotMockServer server;

    robot.reset(new Robot::Impl(std::make_unique<franka::Network>("127.0.0.1", kCommandPort, 200ms),
                                0, franka::RealtimeConfig::kIgnore));

    RobotState robot_state;
    server.sendRandomState<RobotState>([](auto s) { randomRobotState(s); }, &robot_state)
        .spinOnce();

    testRobotStatesAreEqual(franka::convertRobotState(robot_state),
                            robot->updateMotion(std::nullopt, std::nullopt));
  }

  EXPECT_THROW(robot->updateMotion(std::nullopt, std::nullopt), NetworkException);
  EXPECT_THROW(robot->writeOnce(franka::Torques{{1, 2, 3, 4, 5, 6, 7}}), NetworkException);
}

TEST(RobotImplLoggingTests, LogMadeIfErrorReceived) {
  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
  uint32_t message_id = 682;

  constexpr size_t log_size = 5;
  constexpr size_t commands_sent_in_loop = log_size * 2;
  std::vector<RobotCommand> commands;
  std::vector<RobotState> states;

  Robot::Impl robot_with_logs(std::make_unique<franka::Network>("127.0.0.1", kCommandPort),
                              log_size, franka::RealtimeConfig::kIgnore);

  uint32_t move_id;
  server
      .onSendUDP<RobotState>([=, &states](RobotState& robot_state) {
        randomRobotState(robot_state);
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
        states.push_back(robot_state);
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot_with_logs.startMotion(Move::ControllerMode::kExternalController,
                                        Move::MotionGeneratorMode::kJointPosition,
                                        maximum_path_deviation, maximum_goal_pose_deviation,
                                        kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  ASSERT_TRUE(robot_with_logs.motionGeneratorRunning());

  for (size_t i = 0; i < commands_sent_in_loop; i++) {
    server
        .onSendUDP<RobotState>([&](RobotState& robot_state) {
          randomRobotState(robot_state);
          robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
          robot_state.controller_mode = ControllerMode::kExternalController;
          robot_state.robot_mode = RobotMode::kMove;
          robot_state.message_id = ++message_id;
          states.push_back(robot_state);
        })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand&) {})
        .spinOnce();

    auto [sent_motion_command, sent_control_command] = randomRobotCommand();
    sent_motion_command->motion_generation_finished = false;
    RobotCommand sent_command{.message_id = message_id - 1,
                              .motion = *sent_motion_command,
                              .control = *sent_control_command};
    robot_with_logs.updateMotion(sent_motion_command, sent_control_command);
    commands.push_back(sent_command);
  }

  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        randomRobotState(robot_state);
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
        robot_state.message_id = ++message_id;
        states.push_back(robot_state);
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto [last_motion_command, last_control_command] = randomRobotCommand();
  RobotCommand last_command{.message_id = message_id - 1,
                            .motion = *last_motion_command,
                            .control = *last_control_command};
  auto robot_state = robot_with_logs.updateMotion(last_motion_command, last_control_command);
  commands.push_back(last_command);

  try {
    robot_with_logs.throwOnMotionError(robot_state, id);
    FAIL() << "Expected ControlException";
  } catch (const ControlException& exception) {
    EXPECT_EQ(log_size, exception.log.size());
    for (size_t i = 0; i < log_size - 1; i++) {
      // robot.startMotion calls update(), so there's one state more than there are commands
      size_t start_index = commands_sent_in_loop - log_size + 1;
      testRobotStatesAreEqual(franka::convertRobotState(states[start_index + 1 + i]),
                              exception.log[i].state);
      testRobotCommandsAreEqual(commands[start_index + i], exception.log[i].command);
    }
    franka::Record last = exception.log.back();
    testRobotStatesAreEqual(franka::convertRobotState(states.back()), last.state);
    testRobotCommandsAreEqual(last_command, last.command);
  } catch (...) {
    FAIL() << "Expected ControlException";
  }
}

TEST(RobotImplLoggingTests, LogShowsOnlyTheLastMotion) {
  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
  uint32_t message_id = 682;

  constexpr size_t log_size = 5;
  constexpr size_t commands_sent_first_loop = log_size * 2;
  constexpr size_t commands_sent_second_loop = log_size - 2;
  std::vector<RobotCommand> commands;
  std::vector<RobotState> states;

  Robot::Impl robot_with_logs(std::make_unique<franka::Network>("127.0.0.1", kCommandPort),
                              log_size, franka::RealtimeConfig::kIgnore);

  uint32_t move_id;
  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot_with_logs.startMotion(Move::ControllerMode::kExternalController,
                                        Move::MotionGeneratorMode::kJointPosition,
                                        maximum_path_deviation, maximum_goal_pose_deviation,
                                        kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  ASSERT_TRUE(robot_with_logs.motionGeneratorRunning());
  for (size_t i = 0; i < commands_sent_first_loop; i++) {
    server
        .onSendUDP<RobotState>([&](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
          robot_state.controller_mode = ControllerMode::kExternalController;
          robot_state.robot_mode = RobotMode::kMove;
          robot_state.message_id = ++message_id;
        })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand&) {})
        .spinOnce();
    auto motion_command = std::optional<MotionGeneratorCommand>{MotionGeneratorCommand{}};
    auto control_command = std::optional<ControllerCommand>{ControllerCommand{}};
    robot_with_logs.updateMotion(motion_command, control_command);
  }

  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[8] = true;
        robot_state.robot_mode = RobotMode::kReflex;
        robot_state.message_id = ++message_id;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto robot_state = franka::RobotState{};
  {
    auto last_motion_command = std::optional<MotionGeneratorCommand>{MotionGeneratorCommand{}};
    auto last_control_command = std::optional<ControllerCommand>{ControllerCommand{}};
    robot_state = robot_with_logs.updateMotion(last_motion_command, last_control_command);
    EXPECT_THROW(robot_with_logs.throwOnMotionError(robot_state, id), ControlException);
  }

  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        randomRobotState(robot_state);
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = ++message_id;
        states.push_back(robot_state);
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  id = robot_with_logs.startMotion(Move::ControllerMode::kExternalController,
                                   Move::MotionGeneratorMode::kJointPosition,
                                   maximum_path_deviation, maximum_goal_pose_deviation,
                                   kUseNoAsyncMotionGenerator, kNoMaximumVelocities);
  ASSERT_TRUE(robot_with_logs.motionGeneratorRunning());

  for (size_t i = 0; i < commands_sent_second_loop; i++) {
    server
        .onSendUDP<RobotState>([&](RobotState& robot_state) {
          randomRobotState(robot_state);
          robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
          robot_state.controller_mode = ControllerMode::kExternalController;
          robot_state.robot_mode = RobotMode::kMove;
          robot_state.message_id = ++message_id;
          states.push_back(robot_state);
        })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand&) {})
        .spinOnce();

    auto [sent_motion_command, sent_control_command] = randomRobotCommand();
    sent_motion_command->motion_generation_finished = false;
    RobotCommand sent_command{.message_id = message_id - 1,
                              .motion = *sent_motion_command,
                              .control = *sent_control_command};
    robot_with_logs.updateMotion(sent_motion_command, sent_control_command);
    commands.push_back(sent_command);
  }

  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        randomRobotState(robot_state);
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
        robot_state.message_id = ++message_id;
        states.push_back(robot_state);
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto [last_motion_command, last_control_command] = randomRobotCommand();
  RobotCommand last_command{.message_id = message_id - 1,
                            .motion = *last_motion_command,
                            .control = *last_control_command};
  robot_with_logs.updateMotion(last_motion_command, last_control_command);
  commands.push_back(last_command);

  try {
    robot_with_logs.throwOnMotionError(robot_state, id);
    FAIL() << "Expected ControlException";
  } catch (const ControlException& exception) {
    EXPECT_EQ(commands_sent_second_loop + 1, exception.log.size());
    for (size_t i = 0; i < commands_sent_second_loop; i++) {
      // robot.startMotion calls update(), so there's one state more than there are commands
      testRobotStatesAreEqual(franka::convertRobotState(states[i + 1]), exception.log[i].state);
      testRobotCommandsAreEqual(commands[i], exception.log[i].command);
    }
    franka::Record last = exception.log.back();
    testRobotStatesAreEqual(franka::convertRobotState(states.back()), last.state);
    testRobotCommandsAreEqual(last_command, last.command);
  } catch (...) {
    FAIL() << "Expected ControlException";
  }
}
