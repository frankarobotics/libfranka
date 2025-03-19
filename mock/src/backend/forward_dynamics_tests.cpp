#include <franka_mock/backend/forward_dynamics.h>

#include <gtest/gtest.h>
#include <iostream>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace aescape {
constexpr bool kDebug = false;  // Enable debug print

const std::string kUrdfPath = std::string(AESCAPE_MOCK_SOURCE_DIR) + "/resources/aescape_simulation_panda.urdf";
const Eigen::IOFormat kcleanFmt(4, 0, ", ", "\n", "[", "]");
constexpr double kEpsilon = 1e-3;
constexpr size_t kIterations = 2;
constexpr double dt = 0.001;  // Assume to run at 1000Hz

class ForwardDynamicsTests : public ::testing::Test {
 public:
  ForwardDynamicsTests() { pinocchio::urdf::buildModel(kUrdfPath, model_); }

  RobotState updateRobotState(const RobotState& prev_robot_state,
                              const Eigen::VectorXd& tau,
                              double dt,
                              InertiaMatrixMethod method) const {
    auto new_robot_state = aescape::updateRobotState(model_, prev_robot_state, tau, dt, method);
    if (kDebug) {
      std::cout << "-- Prev State ----" << std::endl;
      std::cout << "q        : " << prev_robot_state.q.transpose().format(kcleanFmt) << std::endl;
      std::cout << "dq       : " << prev_robot_state.dq.transpose().format(kcleanFmt) << std::endl;
      std::cout << "ddq      : " << prev_robot_state.ddq.transpose().format(kcleanFmt) << std::endl;
      std::cout << "tau      : " << tau.transpose().format(kcleanFmt) << std::endl;
      std::cout << "-- Updated State ----" << std::endl;
      std::cout << "q        : " << new_robot_state.q.transpose().format(kcleanFmt) << std::endl;
      std::cout << "dq       : " << new_robot_state.dq.transpose().format(kcleanFmt) << std::endl;
      std::cout << "ddq      : " << new_robot_state.ddq.transpose().format(kcleanFmt) << std::endl;
      std::cout << "tau      : " << new_robot_state.tau.transpose().format(kcleanFmt) << std::endl;
      std::cout << "-------------------------" << std::endl;
    }

    return new_robot_state;
  }

 public:
  pinocchio::Model model_;
};

TEST_F(ForwardDynamicsTests, StayInPlace) {
  RobotState robot_state;
  robot_state.q = pinocchio::randomConfiguration(model_);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(kNumJoints);

  auto new_robot_state = updateRobotState(robot_state, tau, dt, InertiaMatrixMethod::kCRBA);
  for (int i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(new_robot_state.q[i], robot_state.q[i], kEpsilon);
    EXPECT_NEAR(new_robot_state.dq[i], robot_state.dq[i], kEpsilon);
    EXPECT_NEAR(new_robot_state.ddq[i], 0.0, kEpsilon);
    EXPECT_NEAR(new_robot_state.tau[i], 0.0, kEpsilon);
  }
}

TEST_F(ForwardDynamicsTests, TorqueOnJoint) {
  // Try moving a single joint each time
  for (size_t joint = 0; joint < kNumJoints; ++joint) {
    RobotState robot_state;
    robot_state.q = pinocchio::randomConfiguration(model_);
    robot_state.q = robot_state.q.cwiseMax(kQMin).cwiseMin(kQMax);

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(kNumJoints);
    tau[joint] += 0.1;

    const auto original_robot_state = robot_state;
    // Run for a few iterations
    for (size_t iteration = 0; iteration < kIterations; ++iteration) {
      auto new_robot_state = updateRobotState(robot_state, tau, dt, InertiaMatrixMethod::kCRBA);
      for (int i = 0; i < kNumJoints; ++i) {
        EXPECT_NEAR(new_robot_state.tau[i], tau[i], kEpsilon);
      }
      robot_state = new_robot_state;
    }

    // Check if joint is moving
    std::cout << "-- Original State ----" << std::endl;
    std::cout << "q        : " << original_robot_state.q.transpose().format(kcleanFmt) << std::endl;
    std::cout << "dq       : " << original_robot_state.dq.transpose().format(kcleanFmt)
              << std::endl;
    std::cout << "ddq      : " << original_robot_state.ddq.transpose().format(kcleanFmt)
              << std::endl;
    std::cout << "tau      : " << original_robot_state.tau.transpose().format(kcleanFmt)
              << std::endl;
    std::cout << "-- Final State ----" << std::endl;
    std::cout << "q        : " << robot_state.q.transpose().format(kcleanFmt) << std::endl;
    std::cout << "dq       : " << robot_state.dq.transpose().format(kcleanFmt) << std::endl;
    std::cout << "ddq      : " << robot_state.ddq.transpose().format(kcleanFmt) << std::endl;
    std::cout << "tau      : " << robot_state.tau.transpose().format(kcleanFmt) << std::endl;
    std::cout << "-------------------------" << std::endl;
    for (int i = 0; i < kNumJoints; ++i) {
      if (i == joint) {
        EXPECT_GT(robot_state.q[i], original_robot_state.q[i]);
        EXPECT_GT(robot_state.dq[i], original_robot_state.dq[i]);
        EXPECT_GT(robot_state.ddq[i], original_robot_state.ddq[i]);
      } else {
        EXPECT_NEAR(robot_state.q[i], original_robot_state.q[i],
                    5 / 180.0 * M_PI);  // Within 5 degrees
      }
    }
  }
}

}  // namespace aescape

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
