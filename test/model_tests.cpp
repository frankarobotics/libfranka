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
#include "model_library_interface.h"

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
};

struct MockModel : public ModelLibraryInterface {
  MOCK_METHOD1(Ji_J_J1, void(double*));
  MOCK_METHOD2(Ji_J_J2, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J3, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J4, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J5, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J6, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J7, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J8, void(const double*, double*));
  MOCK_METHOD3(Ji_J_J9, void(const double*, const double*, double*));

  MOCK_METHOD1(O_J_J1, void(double*));
  MOCK_METHOD2(O_J_J2, void(const double*, double*));
  MOCK_METHOD2(O_J_J3, void(const double*, double*));
  MOCK_METHOD2(O_J_J4, void(const double*, double*));
  MOCK_METHOD2(O_J_J5, void(const double*, double*));
  MOCK_METHOD2(O_J_J6, void(const double*, double*));
  MOCK_METHOD2(O_J_J7, void(const double*, double*));
  MOCK_METHOD2(O_J_J8, void(const double*, double*));
  MOCK_METHOD3(O_J_J9, void(const double*, const double*, double*));

  MOCK_METHOD2(O_T_J1, void(const double*, double*));
  MOCK_METHOD2(O_T_J2, void(const double*, double*));
  MOCK_METHOD2(O_T_J3, void(const double*, double*));
  MOCK_METHOD2(O_T_J4, void(const double*, double*));
  MOCK_METHOD2(O_T_J5, void(const double*, double*));
  MOCK_METHOD2(O_T_J6, void(const double*, double*));
  MOCK_METHOD2(O_T_J7, void(const double*, double*));
  MOCK_METHOD2(O_T_J8, void(const double*, double*));
  MOCK_METHOD3(O_T_J9, void(const double*, const double*, double*));
};

struct Model : public ::testing::Test {
  Model() {
    using namespace std::string_literals;

    std::ifstream model_library_stream(
        FRANKA_TEST_BINARY_DIR + "/libfcimodels.so"s,
        std::ios_base::in | std::ios_base::binary | std::ios_base::ate);
    std::vector<char> buffer;
    buffer.resize(model_library_stream.tellg());
    model_library_stream.seekg(0, std::ios::beg);
    if (!model_library_stream.read(buffer.data(), buffer.size())) {
      throw std::runtime_error("Model test: Cannot load mock libfcimodels.so");
    }

    server
        .generic([=](decltype(server)::Socket& tcp_socket, decltype(server)::Socket&) {
          CommandHeader header;
          server.receiveRequest<LoadModelLibrary>(tcp_socket, &header);
          server.sendResponse<LoadModelLibrary>(
              tcp_socket,
              CommandHeader(Command::kLoadModelLibrary, header.command_id,
                            sizeof(CommandMessage<LoadModelLibrary::Response>) + buffer.size()),
              LoadModelLibrary::Response(LoadModelLibrary::Status::kSuccess));
          tcp_socket.sendBytes(buffer.data(), buffer.size());
        })
        .spinOnce();

    model_library_interface = nullptr;
  }

  RobotMockServer server{};
  franka::Robot robot{"127.0.0.1", franka::RealtimeConfig::kIgnore};
};

TEST(InvalidModel, ThrowsIfNoModelReceived) {
  RobotMockServer server;
  franka::Robot robot("127.0.0.1", franka::RealtimeConfig::kIgnore);

  server
      .waitForCommand<GetRobotModel>([this](const typename GetRobotModel::Request& /*request*/) {
        return GetRobotModel::Response(GetRobotModel::Status::kSuccess);
      })
      .spinOnce();

  server
      .waitForCommand<LoadModelLibrary>([&](const LoadModelLibrary::Request&) {
        return LoadModelLibrary::Response(LoadModelLibrary::Status::kError);
      })
      .spinOnce();

  EXPECT_THROW(robot.loadModel(), franka::ModelException);
}

TEST(InvalidModel, ThrowsIfInvalidModelReceived) {
  RobotMockServer server;
  franka::Robot robot("127.0.0.1", franka::RealtimeConfig::kIgnore);
  auto mock_robot_model = std::make_unique<MockRobotModel>();

  std::array<char, 10> buffer{};
  server
      .generic([&](decltype(server)::Socket& tcp_socket, decltype(server)::Socket&) {
        CommandHeader header;
        server.receiveRequest<LoadModelLibrary>(tcp_socket, &header);
        server.sendResponse<LoadModelLibrary>(
            tcp_socket,
            CommandHeader(Command::kLoadModelLibrary, header.command_id,
                          sizeof(CommandMessage<LoadModelLibrary::Response>) + buffer.size()),
            LoadModelLibrary::Response(LoadModelLibrary::Status::kSuccess));
        tcp_socket.sendBytes(buffer.data(), buffer.size());
      })
      .spinOnce();

  EXPECT_THROW(robot.loadModel(std::move(mock_robot_model)), franka::ModelException);
}

TEST_F(Model, CanCreateModel) {
  auto mock_robot_model = std::make_unique<MockRobotModel>();

  EXPECT_NO_THROW(robot.loadModel(std::move(mock_robot_model)));
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

  MockModel mock;
  EXPECT_CALL(mock, Ji_J_J1(_)).WillOnce(WithArgs<0>(Invoke([=](double* output) {
    std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
  })));
  EXPECT_CALL(mock, Ji_J_J2(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J3(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J4(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J5(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J6(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J7(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J8(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J9(robot_state.q.data(), _, _))
      .WillOnce(WithArgs<1, 2>(Invoke([=](const double* input, double* output) {
        std::array<double, 16> expected;
        Eigen::Map<Eigen::Matrix4d>(expected.data(), 4, 4) =
            (Eigen::Matrix4d(robot_state.F_T_EE.data()) *
             Eigen::Matrix4d(robot_state.EE_T_K.data()));
        std::array<double, 16> input_array;
        std::copy(&input[0], &input[16], input_array.data());
        EXPECT_EQ(expected, input_array);
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J9(robot_state.q.data(), robot_state.F_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));

  model_library_interface = &mock;

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

  MockModel mock;
  EXPECT_CALL(mock, O_J_J1(_)).WillOnce(WithArgs<0>(Invoke([=](double* output) {
    std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
  })));
  EXPECT_CALL(mock, O_J_J2(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J3(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J4(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J5(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J6(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J7(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J8(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J9(robot_state.q.data(), _, _))
      .WillOnce(WithArgs<1, 2>(Invoke([=](const double* input, double* output) {
        std::array<double, 16> expected;
        Eigen::Map<Eigen::Matrix4d>(expected.data(), 4, 4) =
            Eigen::Matrix4d(Eigen::Matrix4d(robot_state.F_T_EE.data()) *
                            Eigen::Matrix4d(robot_state.EE_T_K.data()));
        std::array<double, 16> input_array;
        std::copy(&input[0], &input[16], input_array.data());
        EXPECT_EQ(expected, input_array);
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J9(robot_state.q.data(), robot_state.F_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));

  model_library_interface = &mock;

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
