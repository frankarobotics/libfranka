// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <fstream>
#include <memory>

#include <gmock/gmock.h>
#include <Eigen/Core>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_model_base.h>
#include <research_interface/robot/service_types.h>

#include "helpers.h"
#include "mock_server.h"
#include "test_utils.h"

using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::WithArgs;
using namespace research_interface::robot;

class MockRobotModel : public RobotModelBase {
 public:
  MOCK_METHOD5(mass,
               void(const std::array<double, 7>&,
                    const std::array<double, 9>&,
                    const double,
                    const std::array<double, 3>&,
                    std::array<double, 49>&));
  MOCK_METHOD6(coriolis,
               void(const std::array<double, 7>&,
                    const std::array<double, 7>&,
                    const std::array<double, 9>&,
                    const double,
                    const std::array<double, 3>&,
                    std::array<double, 7>&));
  MOCK_METHOD7(coriolis,
               void(const std::array<double, 7>&,
                    const std::array<double, 7>&,
                    const std::array<double, 9>&,
                    const double,
                    const std::array<double, 3>&,
                    const std::array<double, 3>&,
                    std::array<double, 7>&));
  MOCK_METHOD5(gravity,
               void(const std::array<double, 7>&,
                    const std::array<double, 3>&,
                    const double,
                    const std::array<double, 3>&,
                    std::array<double, 7>&));
  MOCK_METHOD2(pose, std::array<double, 16>(const std::array<double, 7>&, int));
  MOCK_METHOD2(poseEe,
               std::array<double, 16>(const std::array<double, 7>&, const std::array<double, 16>&));
  MOCK_METHOD1(poseFlange, std::array<double, 16>(const std::array<double, 7>&));
  MOCK_METHOD3(poseStiffness,
               std::array<double, 16>(const std::array<double, 7>&,
                                      const std::array<double, 16>&,
                                      const std::array<double, 16>&));
  MOCK_METHOD2(bodyJacobian, std::array<double, 42>(const std::array<double, 7>&, int));
  MOCK_METHOD2(bodyJacobianEe,
               std::array<double, 42>(const std::array<double, 7>&, const std::array<double, 16>&));
  MOCK_METHOD1(bodyJacobianFlange, std::array<double, 42>(const std::array<double, 7>&));
  MOCK_METHOD3(bodyJacobianStiffness,
               std::array<double, 42>(const std::array<double, 7>&,
                                      const std::array<double, 16>&,
                                      const std::array<double, 16>&));
  MOCK_METHOD2(zeroJacobian, std::array<double, 42>(const std::array<double, 7>&, int));
  MOCK_METHOD1(zeroJacobianFlange, std::array<double, 42>(const std::array<double, 7>&));
  MOCK_METHOD2(zeroJacobianEe,
               std::array<double, 42>(const std::array<double, 7>&, const std::array<double, 16>&));
  MOCK_METHOD3(zeroJacobianStiffness,
               std::array<double, 42>(const std::array<double, 7>&,
                                      const std::array<double, 16>&,
                                      const std::array<double, 16>&));
};

struct Model : public ::testing::Test {
  Model() {}

  RobotMockServer server{};
  franka::Robot robot{"127.0.0.1", franka::RealtimeConfig::kIgnore};
};

TEST(CorrectModel, CanHandleNoModel) {
  RobotMockServer server;
  franka::Robot robot("127.0.0.1", franka::RealtimeConfig::kIgnore);

  // Find and load a valid URDF from the test directory
  std::string urdf_path = franka_test_utils::getUrdfPath(__FILE__);
  auto urdf_string = franka_test_utils::readFileToString(urdf_path);

  server
      .waitForCommand<GetRobotModel>(
          [urdf_string](const typename GetRobotModel::Request& /*request*/) {
            return GetRobotModel::Response(GetRobotModel::Status::kSuccess, urdf_string);
          })
      .spinOnce();

  // Robot should now get the URDF model from the server
  EXPECT_NO_THROW(robot.loadModel());
}

TEST_F(Model, CanGetMassMatrix) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);

  auto mock_robot_model = std::make_unique<MockRobotModel>();

  EXPECT_CALL(*mock_robot_model, mass(robot_state.q, robot_state.I_total, robot_state.m_total,
                                      robot_state.F_x_Ctotal, _))
      .WillOnce(WithArgs<4>(Invoke([=](std::array<double, 49>& output) {
        for (size_t i = 0; i < 49; i++) {
          output[i] = i;
        }
      })));

  franka::Model model(robot.loadModel(std::move(mock_robot_model)));
  auto matrix = model.mass(robot_state);
  for (size_t i = 0; i < matrix.size(); i++) {
    EXPECT_EQ(i, matrix[i]);
  }
}

TEST_F(Model, CanGetCoriolisVector) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  std::array<double, 7> expected_vector{12, 13, 14, 15, 16, 17, 18};

  auto mock_robot_model = std::make_unique<MockRobotModel>();

  EXPECT_CALL(*mock_robot_model, coriolis(robot_state.q, robot_state.dq, robot_state.I_total,
                                          robot_state.m_total, robot_state.F_x_Ctotal, _))
      .WillOnce(WithArgs<5>(Invoke([=](std::array<double, 7>& output) {
        std::copy(expected_vector.cbegin(), expected_vector.cend(), output.data());
      })));

  franka::Model model(robot.loadModel(std::move(mock_robot_model)));
  auto vector = model.coriolis(robot_state);
  EXPECT_EQ(expected_vector, vector);
}

TEST_F(Model, CanGetGravity) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  std::array<double, 3> gravity_earth{4, 5, 6};

  auto mock_robot_model = std::make_unique<MockRobotModel>();

  EXPECT_CALL(*mock_robot_model,
              gravity(robot_state.q, gravity_earth, robot_state.m_total, robot_state.F_x_Ctotal, _))
      .WillOnce(WithArgs<4>(Invoke([=](std::array<double, 7>& output) {
        for (size_t i = 0; i < 7; i++) {
          output[i] = i;
        }
      })));

  franka::Model model(robot.loadModel(std::move(mock_robot_model)));
  auto matrix = model.gravity(robot_state, gravity_earth);
  for (size_t i = 0; i < matrix.size(); i++) {
    EXPECT_EQ(i, matrix[i]);
  }
}

TEST_F(Model, CanGetJointPoses) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  auto mock_robot_model = std::make_unique<MockRobotModel>();

  std::array<double, 16> expected_pose{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}};

  EXPECT_CALL(*mock_robot_model, pose(robot_state.q, 1)).WillOnce(Return(expected_pose));
  EXPECT_CALL(*mock_robot_model, pose(robot_state.q, 2)).WillOnce(Return(expected_pose));
  EXPECT_CALL(*mock_robot_model, pose(robot_state.q, 3)).WillOnce(Return(expected_pose));
  EXPECT_CALL(*mock_robot_model, pose(robot_state.q, 4)).WillOnce(Return(expected_pose));
  EXPECT_CALL(*mock_robot_model, pose(robot_state.q, 5)).WillOnce(Return(expected_pose));
  EXPECT_CALL(*mock_robot_model, pose(robot_state.q, 6)).WillOnce(Return(expected_pose));
  EXPECT_CALL(*mock_robot_model, pose(robot_state.q, 7)).WillOnce(Return(expected_pose));
  EXPECT_CALL(*mock_robot_model, poseFlange(robot_state.q)).WillOnce(Return(expected_pose));
  EXPECT_CALL(*mock_robot_model, poseEe(robot_state.q, robot_state.F_T_EE))
      .WillOnce(Return(expected_pose));
  EXPECT_CALL(*mock_robot_model,
              poseStiffness(robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K))
      .WillOnce(Return(expected_pose));

  franka::Model model(robot.loadModel(std::move(mock_robot_model)));
  for (franka::Frame joint = franka::Frame::kJoint1; joint <= franka::Frame::kStiffness; joint++) {
    auto pose = model.pose(joint, robot_state);
    EXPECT_EQ(expected_pose, pose);
  }
}

TEST_F(Model, CanGetBodyJacobian) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  auto mock_robot_model = std::make_unique<MockRobotModel>();

  std::array<double, 42> expected_jacobian;
  for (unsigned int i = 0; i < expected_jacobian.size(); i++) {
    expected_jacobian[i] = i;
  }

  EXPECT_CALL(*mock_robot_model, bodyJacobian(robot_state.q, 1))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, bodyJacobian(robot_state.q, 2))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, bodyJacobian(robot_state.q, 3))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, bodyJacobian(robot_state.q, 4))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, bodyJacobian(robot_state.q, 5))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, bodyJacobian(robot_state.q, 6))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, bodyJacobian(robot_state.q, 7))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, bodyJacobianFlange(robot_state.q))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, bodyJacobianEe(robot_state.q, robot_state.F_T_EE))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model,
              bodyJacobianStiffness(robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K))
      .WillOnce(Return(expected_jacobian));

  franka::Model model(robot.loadModel(std::move(mock_robot_model)));
  for (franka::Frame joint = franka::Frame::kJoint1; joint <= franka::Frame::kStiffness; joint++) {
    auto jacobian = model.bodyJacobian(joint, robot_state);
    EXPECT_EQ(expected_jacobian, jacobian);
  }
}

TEST_F(Model, CanGetZeroJacobian) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  auto mock_robot_model = std::make_unique<MockRobotModel>();

  std::array<double, 42> expected_jacobian;
  for (unsigned int i = 0; i < expected_jacobian.size(); i++) {
    expected_jacobian[i] = i;
  }

  EXPECT_CALL(*mock_robot_model, zeroJacobian(robot_state.q, 1))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, zeroJacobian(robot_state.q, 2))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, zeroJacobian(robot_state.q, 3))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, zeroJacobian(robot_state.q, 4))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, zeroJacobian(robot_state.q, 5))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, zeroJacobian(robot_state.q, 6))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, zeroJacobian(robot_state.q, 7))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, zeroJacobianFlange(robot_state.q))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model, zeroJacobianEe(robot_state.q, robot_state.F_T_EE))
      .WillOnce(Return(expected_jacobian));
  EXPECT_CALL(*mock_robot_model,
              zeroJacobianStiffness(robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K))
      .WillOnce(Return(expected_jacobian));

  franka::Model model(robot.loadModel(std::move(mock_robot_model)));
  for (franka::Frame joint = franka::Frame::kJoint1; joint <= franka::Frame::kStiffness; joint++) {
    auto jacobian = model.zeroJacobian(joint, robot_state);
    EXPECT_EQ(expected_jacobian, jacobian);
  }
}

TEST(Frame, CanIncrement) {
  franka::Frame frame = franka::Frame::kJoint3;
  EXPECT_EQ(franka::Frame::kJoint3, frame++);
  EXPECT_EQ(franka::Frame::kJoint4, frame);
}
