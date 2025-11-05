#include <franka/async_control/async_position_control_handler.hpp>

#include "../mock_robot.h"
#include "../mock_robot_impl.h"
#include "../mock_server.h"

const std::optional<std::vector<double>> kDefaultMaximumVelocities =
    std::vector<double>{0.655, 0.655, 0.655, 0.655, 1.315, 1.315, 1.315};
constexpr double kDefaultGoalTolerance = 0.01;
constexpr bool kAsyncControlIsActive = true;

namespace franka {

const double k_EPS = 1e-5;

template <typename T>
bool compareWithTolerance(const T& lhs, const T& rhs) {
  return std::equal(lhs.begin(), lhs.end(), rhs.begin(),
                    [](double lhs_element, double rhs_element) {
                      return std::abs(lhs_element - rhs_element) < k_EPS;
                    });
}

bool operator==(const franka::JointPositions& lhs, const franka::JointPositions& rhs) {
  return compareWithTolerance(lhs.q, rhs.q);
}

}  // namespace franka

class AsyncPositionControlTests : public ::testing::Test {
 protected:
  const uint32_t default_motion_id = 42;
  RobotMockServer server_;
  std::shared_ptr<RobotImplMock> robot_impl_mock_ = std::make_shared<RobotImplMock>(
      std::move(std::make_unique<Network>("127.0.0.1", robot::kCommandPort)),
      0,
      RealtimeConfig::kIgnore);
  std::shared_ptr<RobotMock> robot_mock_ = std::make_shared<RobotMock>(robot_impl_mock_);

  auto startHandler() -> franka::AsyncPositionControlHandler::ConfigurationResult {
    EXPECT_CALL(*robot_impl_mock_, startMotion(testing::_, testing::_, testing::_, testing::_,
                                               kAsyncControlIsActive, kDefaultMaximumVelocities))
        .Times(1)
        .WillOnce(::testing::Return(default_motion_id));

    auto async_position_control_handler_result = franka::AsyncPositionControlHandler::configure(
        robot_mock_, franka::AsyncPositionControlHandler::Configuration{
                         .maximum_joint_velocities = kDefaultMaximumVelocities,
                         .goal_tolerance = kDefaultGoalTolerance});

    return async_position_control_handler_result;
  }
};

TEST_F(AsyncPositionControlTests, GivenValidPositionControl_WhenConfigure_ThenReturnsHandler) {
  auto async_position_control_handler_result = startHandler();

  ASSERT_NE(async_position_control_handler_result.handler, nullptr);
  ASSERT_FALSE(async_position_control_handler_result.error_message.has_value());
}

TEST_F(AsyncPositionControlTests,
       GivenNotStartedToMoveYet_WhenGetTargetFeedback_ThenReturnsIdleStatus) {
  auto async_position_control_handler_result = startHandler();
  const auto& position_control_handler = async_position_control_handler_result.handler;
  auto expected_robot_state = franka::RobotState{.robot_mode = franka::RobotMode::kIdle};

  EXPECT_CALL(*robot_impl_mock_, readOnce())
      .Times(1)
      .WillOnce(::testing::Return(expected_robot_state));

  auto feedback_result = position_control_handler->getTargetFeedback();

  ASSERT_EQ(feedback_result.status, franka::TargetStatus::kIdle);
  ASSERT_FALSE(feedback_result.error_message.has_value());
}

TEST_F(AsyncPositionControlTests,
       GivenValidPositionControlHandler_WhenSetJointPositionTarget_ThenReturnsSuccess) {
  auto expected_joint_positions = franka::JointPositions{{0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.5}};
  EXPECT_CALL(*robot_impl_mock_, writeOnce(expected_joint_positions)).Times(1);

  auto async_position_control_handler_result = startHandler();
  const auto& position_control_handler = async_position_control_handler_result.handler;

  auto target = franka::AsyncPositionControlHandler::JointPositionTarget{
      .joint_positions = {0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.5}};

  auto command_result = position_control_handler->setJointPositionTarget(target);

  ASSERT_TRUE(command_result.was_successful);
  ASSERT_FALSE(command_result.error_message.has_value());
}

TEST_F(AsyncPositionControlTests,
       GivenNormalExecution_WhenGetTargetFeedback_ThenGetExecutingStatus) {
  auto async_position_control_handler_result = startHandler();
  const auto& position_control_handler = async_position_control_handler_result.handler;
  auto expected_robot_state = franka::RobotState{.robot_mode = franka::RobotMode::kMove};

  expected_robot_state.q_d = {0.0, -0.0, 0.0, -1.0, 0.0, 1.0, 0.5};
  expected_robot_state.dq_d = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto moving_target = franka::AsyncPositionControlHandler::JointPositionTarget{
      .joint_positions = expected_robot_state.q_d};
  moving_target.joint_positions[0] += 0.5;
  auto expected_franka_joint_positions = franka::JointPositions{moving_target.joint_positions};

  EXPECT_CALL(*robot_impl_mock_, writeOnce(expected_franka_joint_positions)).Times(1);
  EXPECT_CALL(*robot_impl_mock_, readOnce())
      .Times(1)
      .WillOnce(::testing::Return(expected_robot_state));

  position_control_handler->setJointPositionTarget(moving_target);

  auto feedback_result = position_control_handler->getTargetFeedback();

  ASSERT_EQ(feedback_result.status, franka::TargetStatus::kExecuting);
  ASSERT_FALSE(feedback_result.error_message.has_value());
}

TEST_F(AsyncPositionControlTests,
       GivenPositionWithinTolerance_WhenGetTargetFeedback_ThenGetTargetReachedStatus) {
  auto async_position_control_handler_result = startHandler();
  const auto& position_control_handler = async_position_control_handler_result.handler;
  auto expected_robot_state = franka::RobotState{.robot_mode = franka::RobotMode::kMove};

  expected_robot_state.q_d = {0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.5};
  expected_robot_state.dq_d = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto target_within_tolerance = franka::AsyncPositionControlHandler::JointPositionTarget{
      .joint_positions = expected_robot_state.q_d};
  target_within_tolerance.joint_positions[0] += kDefaultGoalTolerance / 2;  // within tolerance
  auto expected_franka_joint_positions =
      franka::JointPositions{target_within_tolerance.joint_positions};

  EXPECT_CALL(*robot_impl_mock_, writeOnce(expected_franka_joint_positions)).Times(1);
  EXPECT_CALL(*robot_impl_mock_, readOnce())
      .Times(1)
      .WillOnce(::testing::Return(expected_robot_state));

  position_control_handler->setJointPositionTarget(target_within_tolerance);

  auto feedback_result = position_control_handler->getTargetFeedback();

  ASSERT_EQ(feedback_result.status, franka::TargetStatus::kTargetReached);
  ASSERT_FALSE(feedback_result.error_message.has_value());
}

TEST_F(AsyncPositionControlTests, GivenMovingRobot_WhenGetTargetFeedback_ThenGetExecutingStatus) {
  auto async_position_control_handler_result = startHandler();
  const auto& position_control_handler = async_position_control_handler_result.handler;
  auto expected_robot_state = franka::RobotState{.robot_mode = franka::RobotMode::kMove};

  expected_robot_state.q_d = {0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.5};
  expected_robot_state.dq_d = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  auto target_at_position = franka::AsyncPositionControlHandler::JointPositionTarget{
      .joint_positions = expected_robot_state.q_d};
  auto expected_franka_joint_positions = franka::JointPositions{target_at_position.joint_positions};

  EXPECT_CALL(*robot_impl_mock_, writeOnce(expected_franka_joint_positions)).Times(1);
  EXPECT_CALL(*robot_impl_mock_, readOnce())
      .Times(1)
      .WillOnce(::testing::Return(expected_robot_state));

  position_control_handler->setJointPositionTarget(target_at_position);

  auto feedback_result = position_control_handler->getTargetFeedback();

  ASSERT_EQ(feedback_result.status, franka::TargetStatus::kExecuting);
  ASSERT_FALSE(feedback_result.error_message.has_value());
}

TEST_F(AsyncPositionControlTests,
       GivenControlInAbortedState_WhenSetJointPositionTarget_ThenReturnsError) {
  auto async_position_control_handler_result = startHandler();
  const auto& position_control_handler = async_position_control_handler_result.handler;

  // Directly stop the control
  position_control_handler->stopControl();

  auto target = franka::AsyncPositionControlHandler::JointPositionTarget{
      .joint_positions = {0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.5}};

  auto command_result = position_control_handler->setJointPositionTarget(target);

  ASSERT_FALSE(command_result.was_successful);
  ASSERT_TRUE(command_result.error_message.has_value());
}

TEST_F(AsyncPositionControlTests,
       GivenRobotModeNotInMove_WhenGetTargetFeedback_ThenReturnsAbortedStatus) {
  auto async_position_control_handler_result = startHandler();
  const auto& position_control_handler = async_position_control_handler_result.handler;
  auto expected_robot_state = franka::RobotState{.robot_mode = franka::RobotMode::kReflex};

  EXPECT_CALL(*robot_impl_mock_, readOnce())
      .Times(1)
      .WillOnce(::testing::Return(expected_robot_state));

  auto feedback_result = position_control_handler->getTargetFeedback();

  ASSERT_EQ(feedback_result.status, franka::TargetStatus::kAborted);
  ASSERT_TRUE(feedback_result.error_message.has_value());
}
