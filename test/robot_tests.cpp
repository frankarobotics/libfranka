// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gmock/gmock.h>

#include <atomic>
#include <functional>
#include <thread>
#include <utility>

#include <franka/active_control.h>
#include <franka/active_torque_control.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/robot.h>
#include <research_interface/robot/service_types.h>
#include <robot_impl.h>

#include "helpers.h"
#include "mock_robot.h"
#include "mock_robot_impl.h"
#include "mock_server.h"

using ::testing::_;
using ::testing::Return;

using research_interface::robot::Connect;
using research_interface::robot::Move;
using research_interface::robot::StopMove;
using namespace research_interface;

using namespace franka;
using namespace std::chrono_literals;

constexpr bool kUseNoAsyncMotionGenerator = false;
const std::optional<std::vector<double>> kNoMaximumVelocities = std::nullopt;

class RobotTests : public ::testing::Test {
 public:
  RobotTests() : default_robot("127.0.0.1", RealtimeConfig::kIgnore) {}

 protected:
  RobotMockServer default_server;
  Robot default_robot;
};

TEST_F(RobotTests, CanPerformHandshake) {
  EXPECT_EQ(research_interface::robot::kVersion, default_robot.serverVersion());
}

TEST_F(RobotTests, CanReadRobotState) {
  struct MockCallback {
    MOCK_METHOD1(invoke, bool(const RobotState&));
  };

  default_server.sendEmptyState<research_interface::robot::RobotState>().spinOnce();

  MockCallback callback;
  EXPECT_CALL(callback, invoke(_));

  default_robot.read([&](const RobotState& robot_state) { return callback.invoke(robot_state); });
}

TEST_F(RobotTests, CanReadRobotStateAfterInstanceMove) {
  struct MockCallback {
    MOCK_METHOD1(invoke, bool(const RobotState&));
  };
  MockCallback callback;

  default_server.sendEmptyState<research_interface::robot::RobotState>().spinOnce();
  EXPECT_CALL(callback, invoke(_));
  default_robot.read([&](const RobotState& robot_state) { return callback.invoke(robot_state); });

  // Move constructor
  Robot robot2(std::move(default_robot));
  default_server.sendEmptyState<research_interface::robot::RobotState>().spinOnce();
  EXPECT_CALL(callback, invoke(_));
  robot2.read([&](const RobotState& robot_state) { return callback.invoke(robot_state); });

  // Move assignment
  default_robot = std::move(robot2);
  default_server.sendEmptyState<research_interface::robot::RobotState>().spinOnce();
  EXPECT_CALL(callback, invoke(_));
  default_robot.read([&](const RobotState& robot_state) { return callback.invoke(robot_state); });
}

TEST_F(RobotTests, CanControlRobot) {
  uint32_t move_id;

  std::atomic_flag send = ATOMIC_FLAG_INIT;
  send.test_and_set();

  uint32_t stopped_message_id = 0;
  default_server
      .onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
        robot_state.motion_generator_mode = robot::MotionGeneratorMode::kIdle;
        robot_state.controller_mode = robot::ControllerMode::kOther;
        robot_state.robot_mode = robot::RobotMode::kIdle;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) {
            default_server
                .doForever([&]() {
                  bool continue_sending = send.test_and_set();
                  if (continue_sending) {
                    default_server.onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
                      robot_state.motion_generator_mode =
                          robot::MotionGeneratorMode::kJointPosition;
                      robot_state.controller_mode = robot::ControllerMode::kJointImpedance;
                      robot_state.robot_mode = robot::RobotMode::kMove;
                    });
                    std::this_thread::yield();
                    std::this_thread::sleep_for(1ms);
                  }
                  return continue_sending;
                })
                .onSendUDP<robot::RobotState>([&](robot::RobotState& robot_state) {
                  robot_state.motion_generator_mode = robot::MotionGeneratorMode::kIdle;
                  robot_state.controller_mode = robot::ControllerMode::kJointImpedance;
                  robot_state.robot_mode = robot::RobotMode::kIdle;
                  stopped_message_id = robot_state.message_id;
                })
                .sendResponse<Move>(move_id,
                                    []() { return Move::Response(Move::Status::kSuccess); });
            return Move::Response(Move::Status::kMotionStarted);
          },
          &move_id)
      .spinOnce();

  JointPositions joint_positions{{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}};
  int count = 0;
  default_robot.control(
      [&](const RobotState&, Duration time_step) -> JointPositions {
        if (count == 0) {
          EXPECT_EQ(0u, time_step.toMSec());
        } else {
          EXPECT_GE(time_step.toMSec(), 1u);
        }
        if (++count < 5) {
          return joint_positions;
        }
        send.clear();
        return MotionFinished(joint_positions);
      },
      ControllerMode::kJointImpedance, false, franka::kMaxCutoffFrequency);

  ASSERT_NE(0u, stopped_message_id);
  ASSERT_EQ(5, count);

  // Receive the robot commands sent in the motion loop.
  for (int i = 0; i < count - 1; i++) {
    default_server
        .onReceiveRobotCommand([=](const robot::RobotCommand& robot_command) {
          EXPECT_EQ(joint_positions.q, robot_command.motion.q_c);
          EXPECT_FALSE(robot_command.motion.motion_generation_finished);
          EXPECT_LT(robot_command.message_id, stopped_message_id);
        })
        .spinOnce();
  }

  // Receive the robot commands sent after Stop has been returned from the motion loop.
  // These will be sent at least once and until Robot received the robot state showing the stopped
  // motion.
  default_server
      .onReceiveRobotCommand([=](const robot::RobotCommand& robot_command) {
        EXPECT_TRUE(robot_command.motion.motion_generation_finished);
        EXPECT_LT(robot_command.message_id, stopped_message_id);
      })
      .spinOnce();

  // Ignore remaining RobotCommands that might have been sent to the server.
  default_server.ignoreUdpBuffer();
}

TEST_F(RobotTests, StopAfterControllerChange) {
  uint32_t move_id;

  std::atomic_flag send = ATOMIC_FLAG_INIT;
  send.test_and_set();

  uint32_t stopped_message_id = 0;
  default_server
      .onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
        robot_state.motion_generator_mode = robot::MotionGeneratorMode::kIdle;
        robot_state.controller_mode = robot::ControllerMode::kOther;
        robot_state.robot_mode = robot::RobotMode::kIdle;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) {
            default_server
                .doForever([&]() {
                  bool continue_sending = send.test_and_set();
                  if (continue_sending) {
                    default_server.onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
                      robot_state.motion_generator_mode =
                          robot::MotionGeneratorMode::kJointPosition;
                      robot_state.controller_mode = robot::ControllerMode::kExternalController;
                      robot_state.robot_mode = robot::RobotMode::kMove;
                    });
                    std::this_thread::yield();
                    std::this_thread::sleep_for(1ms);
                  }
                  return continue_sending;
                })
                .onSendUDP<robot::RobotState>([&](robot::RobotState& robot_state) {
                  robot_state.motion_generator_mode = robot::MotionGeneratorMode::kJointPosition;
                  robot_state.controller_mode = robot::ControllerMode::kOther;
                  robot_state.robot_mode = robot::RobotMode::kMove;
                  stopped_message_id = robot_state.message_id;
                })
                .sendResponse<Move>(move_id,
                                    []() { return Move::Response(Move::Status::kReflexAborted); })
                .waitForCommand<StopMove>([](const StopMove::Request&) {
                  return StopMove::Response(StopMove::Status::kSuccess);
                });
            return Move::Response(Move::Status::kMotionStarted);
          },
          &move_id)
      .spinOnce();

  // Ignore remaining RobotCommands that might have been sent to the server.
  default_server.ignoreUdpBuffer();

  int count = 0;
  JointPositions joint_positions{{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}};
  Torques torques{{10, 10, 10, 10, 10, 10, 10}};
  EXPECT_THROW(default_robot.control([&](const franka::RobotState&,
                                         franka::Duration) -> franka::Torques { return torques; },
                                     [&](const RobotState&, Duration time_step) -> JointPositions {
                                       if (count == 0) {
                                         EXPECT_EQ(0u, time_step.toMSec());
                                       } else {
                                         EXPECT_GE(time_step.toMSec(), 1u);
                                       }
                                       if (++count == 5) {
                                         send.clear();
                                       }
                                       return joint_positions;
                                     }),
               ControlException);

  ASSERT_NE(0u, stopped_message_id);
  ASSERT_GE(count, 5);
}

TEST_F(RobotTests, StopAfterMotionAndControllerChange) {
  uint32_t move_id;

  std::atomic_flag send = ATOMIC_FLAG_INIT;
  send.test_and_set();

  uint32_t stopped_message_id = 0;
  default_server
      .onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
        robot_state.motion_generator_mode = robot::MotionGeneratorMode::kIdle;
        robot_state.controller_mode = robot::ControllerMode::kOther;
        robot_state.robot_mode = robot::RobotMode::kIdle;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) {
            default_server
                .doForever([&]() {
                  bool continue_sending = send.test_and_set();
                  if (continue_sending) {
                    default_server.onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
                      robot_state.motion_generator_mode =
                          robot::MotionGeneratorMode::kJointPosition;
                      robot_state.controller_mode = robot::ControllerMode::kExternalController;
                      robot_state.robot_mode = robot::RobotMode::kMove;
                    });
                    std::this_thread::yield();
                    std::this_thread::sleep_for(1ms);
                  }
                  return continue_sending;
                })
                .onSendUDP<robot::RobotState>([&](robot::RobotState& robot_state) {
                  robot_state.motion_generator_mode = robot::MotionGeneratorMode::kIdle;
                  robot_state.controller_mode = robot::ControllerMode::kOther;
                  robot_state.robot_mode = robot::RobotMode::kMove;
                  stopped_message_id = robot_state.message_id;
                })
                .sendResponse<Move>(move_id,
                                    []() { return Move::Response(Move::Status::kReflexAborted); })
                .waitForCommand<StopMove>([](const StopMove::Request&) {
                  return StopMove::Response(StopMove::Status::kSuccess);
                });
            return Move::Response(Move::Status::kMotionStarted);
          },
          &move_id)
      .spinOnce();

  // Ignore remaining RobotCommands that might have been sent to the server.
  default_server.ignoreUdpBuffer();

  int count = 0;
  JointPositions joint_positions{{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}};
  Torques torques{{10, 10, 10, 10, 10, 10, 10}};
  EXPECT_THROW(default_robot.control([&](const franka::RobotState&,
                                         franka::Duration) -> franka::Torques { return torques; },
                                     [&](const RobotState&, Duration time_step) -> JointPositions {
                                       if (count == 0) {
                                         EXPECT_EQ(0u, time_step.toMSec());
                                       } else {
                                         EXPECT_GE(time_step.toMSec(), 1u);
                                       }
                                       if (++count == 5) {
                                         send.clear();
                                       }
                                       return joint_positions;
                                     }),
               ControlException);

  ASSERT_NE(0u, stopped_message_id);
  ASSERT_GE(count, 5);
}

TEST_F(RobotTests, StopAfterMotionGeneratorChange) {
  uint32_t move_id;

  std::atomic_flag send = ATOMIC_FLAG_INIT;
  send.test_and_set();

  uint32_t stopped_message_id = 0;
  default_server
      .onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
        robot_state.motion_generator_mode = robot::MotionGeneratorMode::kIdle;
        robot_state.controller_mode = robot::ControllerMode::kOther;
        robot_state.robot_mode = robot::RobotMode::kIdle;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) {
            default_server
                .doForever([&]() {
                  bool continue_sending = send.test_and_set();
                  if (continue_sending) {
                    default_server.onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
                      robot_state.motion_generator_mode =
                          robot::MotionGeneratorMode::kJointPosition;
                      robot_state.controller_mode = robot::ControllerMode::kExternalController;
                      robot_state.robot_mode = robot::RobotMode::kMove;
                    });
                    std::this_thread::yield();
                    std::this_thread::sleep_for(1ms);
                  }
                  return continue_sending;
                })
                .onSendUDP<robot::RobotState>([&](robot::RobotState& robot_state) {
                  robot_state.motion_generator_mode = robot::MotionGeneratorMode::kIdle;
                  robot_state.controller_mode = robot::ControllerMode::kExternalController;
                  robot_state.robot_mode = robot::RobotMode::kMove;
                  stopped_message_id = robot_state.message_id;
                })
                .sendResponse<Move>(move_id,
                                    []() { return Move::Response(Move::Status::kReflexAborted); })
                .waitForCommand<StopMove>([](const StopMove::Request&) {
                  return StopMove::Response(StopMove::Status::kSuccess);
                });
            return Move::Response(Move::Status::kMotionStarted);
          },
          &move_id)
      .spinOnce();

  // Ignore remaining RobotCommands that might have been sent to the server.
  default_server.ignoreUdpBuffer();

  int count = 0;
  JointPositions joint_positions{{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}};
  Torques torques{{10, 10, 10, 10, 10, 10, 10}};
  EXPECT_THROW(default_robot.control([&](const franka::RobotState&,
                                         franka::Duration) -> franka::Torques { return torques; },
                                     [&](const RobotState&, Duration time_step) -> JointPositions {
                                       if (count == 0) {
                                         EXPECT_EQ(0u, time_step.toMSec());
                                       } else {
                                         EXPECT_GE(time_step.toMSec(), 1u);
                                       }
                                       if (++count == 5) {
                                         send.clear();
                                       }
                                       return joint_positions;
                                     }),
               ControlException);

  ASSERT_NE(0u, stopped_message_id);
  ASSERT_GE(count, 5);
}

TEST_F(RobotTests, ThrowsIfConflictingOperationIsRunning) {
  std::atomic_bool run(true);

  default_server.sendEmptyState<robot::RobotState>().spinOnce();

  std::mutex mutex;
  std::condition_variable cv;
  std::atomic_bool read_started(false);
  auto thread = std::thread([&]() {
    default_robot.read([&](const RobotState&) {
      read_started = true;
      EXPECT_THROW(default_robot.read(std::function<bool(const RobotState&)>()),
                   InvalidOperationException);
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait(lock, [&]() { return !run; });
      return false;
    });
  });

  while (!read_started) {
    std::this_thread::yield();
  }
  EXPECT_THROW(default_robot.control(std::function<Torques(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(default_robot.control(std::function<Torques(const RobotState&, Duration)>(),
                                     std::function<JointPositions(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(default_robot.control(std::function<Torques(const RobotState&, Duration)>(),
                                     std::function<JointVelocities(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(default_robot.control(std::function<Torques(const RobotState&, Duration)>(),
                                     std::function<CartesianPose(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(
      default_robot.control(std::function<Torques(const RobotState&, Duration)>(),
                            std::function<CartesianVelocities(const RobotState&, Duration)>()),
      InvalidOperationException);
  EXPECT_THROW(default_robot.control(std::function<JointPositions(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(default_robot.control(std::function<JointVelocities(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(default_robot.control(std::function<CartesianPose(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(
      default_robot.control(std::function<CartesianVelocities(const RobotState&, Duration)>()),
      InvalidOperationException);
  EXPECT_THROW(default_robot.read(std::function<bool(const RobotState&)>()),
               InvalidOperationException);
  EXPECT_THROW(default_robot.readOnce(), InvalidOperationException);
  EXPECT_THROW(default_robot.startTorqueControl(), InvalidOperationException);

  default_server.ignoreUdpBuffer();

  run = false;
  cv.notify_one();

  if (thread.joinable()) {
    thread.join();
  }
}

TEST(RobotConnectionTests, CannotConnectIfNoServerRunning) {
  EXPECT_THROW(Robot robot("127.0.0.1", franka::RealtimeConfig::kIgnore), NetworkException)
      << "Shut down local robot service to run tests.";
}

TEST(RobotConnectionTests, ThrowsOnIncompatibleLibraryVersion) {
  RobotMockServer server([](const Connect::Request&) {
    return Connect::Response(Connect::Status::kIncompatibleLibraryVersion);
  });

  EXPECT_THROW(Robot robot("127.0.0.1", franka::RealtimeConfig::kIgnore),
               IncompatibleVersionException);
}

TEST_F(RobotTests, JointVelocityLimitsMatchRateLimitingFunctions) {
  std::vector<std::array<double, 7>> test_positions = {
      {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},   {{1.0, 0.5, -0.5, -1.5, 1.2, 2.0, -1.0}},
      {{2.5, 1.5, 2.5, -0.2, 2.5, 4.0, 2.8}},  {{-2.5, -1.5, -2.5, -2.8, -2.5, 1.0, -2.8}},
      {{1.5, -0.8, 1.2, -2.0, 0.5, 3.5, 1.5}},
  };

  const double tolerance = 1e-8;

  for (const auto& joint_positions : test_positions) {
    auto robot_upper_limits = default_robot.getUpperJointVelocityLimits(joint_positions);
    auto rate_limiting_upper_limits = referenceUpperLimitsJointVelocityCalculation(joint_positions);

    for (size_t i = 0; i < 7; ++i) {
      EXPECT_NEAR(robot_upper_limits[i], rate_limiting_upper_limits[i], tolerance)
          << "Upper velocity limit mismatch for joint " << i << " at position "
          << joint_positions[i];
    }

    auto robot_lower_limits = default_robot.getLowerJointVelocityLimits(joint_positions);
    auto rate_limiting_lower_limits = referenceLowerLimitsJointVelocityCalculation(joint_positions);

    for (size_t i = 0; i < 7; ++i) {
      EXPECT_NEAR(robot_lower_limits[i], rate_limiting_lower_limits[i], tolerance)
          << "Lower velocity limit mismatch for joint " << i << " at position "
          << joint_positions[i];
    }
  }
}

TEST_F(RobotTests, JointVelocityLimitsConsistency) {
  std::array<double, 7> joint_positions{{0.5, -0.3, 1.2, -1.8, 0.8, 2.5, -0.5}};

  auto upper_limits = default_robot.getUpperJointVelocityLimits(joint_positions);
  auto lower_limits = default_robot.getLowerJointVelocityLimits(joint_positions);

  for (size_t i = 0; i < 7; ++i) {
    EXPECT_GE(upper_limits[i], lower_limits[i])
        << "Upper limit should be >= lower limit for joint " << i;
  }
}

TEST_F(RobotTests, JointVelocityLimitsAtExtremePositions) {
  std::vector<std::array<double, 7>> extreme_positions = {
      {{2.743, 1.783, 2.900, -0.152, 2.806, 4.516, 3.015}},
      {{-2.743, -1.783, -2.900, -3.041, -2.806, 0.545, -3.015}},
  };

  const double tolerance = 1e-8;

  for (const auto& joint_positions : extreme_positions) {
    EXPECT_NO_THROW({
      auto robot_upper_limits = default_robot.getUpperJointVelocityLimits(joint_positions);
      auto robot_lower_limits = default_robot.getLowerJointVelocityLimits(joint_positions);
      auto rate_limiting_upper_limits =
          referenceUpperLimitsJointVelocityCalculation(joint_positions);
      auto rate_limiting_lower_limits =
          referenceLowerLimitsJointVelocityCalculation(joint_positions);
      for (size_t i = 0; i < 7; ++i) {
        EXPECT_NEAR(robot_upper_limits[i], rate_limiting_upper_limits[i], tolerance);
        EXPECT_NEAR(robot_lower_limits[i], rate_limiting_lower_limits[i], tolerance);
      }
    });
  }
}

TEST(RobotMultipleControlTests, CanStartOnlyOneControl) {
  RobotMockServer server;
  auto network = std::make_unique<Network>("127.0.0.1", robot::kCommandPort);

  auto robot_impl_mock =
      std::make_shared<RobotImplMock>(std::move(network), 0, RealtimeConfig::kIgnore);
  RobotMock robot(robot_impl_mock);

  server.sendEmptyState<robot::RobotState>().spinOnce();

  const robot::Move::Deviation kDefaultDeviation{10.0, 3.12, 2 * M_PI};
  auto motion_generator_mode = Move::MotionGeneratorMode::kNone;
  auto controller_mode = Move::ControllerMode::kExternalController;

  EXPECT_CALL(*robot_impl_mock,
              startMotion(controller_mode, motion_generator_mode, kDefaultDeviation,
                          kDefaultDeviation, kUseNoAsyncMotionGenerator, kNoMaximumVelocities))
      .Times(2)
      .WillRepeatedly(::testing::Return(100));

  EXPECT_CALL(*robot_impl_mock, cancelMotion(100)).Times(2);

  EXPECT_NO_THROW(auto control = robot.startTorqueControl());

  auto control = robot.startTorqueControl();
  EXPECT_THROW(robot.startTorqueControl(), InvalidOperationException);
}
