#include <fstream>
#include <mutex>

#include <magic_enum.hpp>
#include <tracy/Tracy.hpp>

#include <franka/robot_model.h>
#include <franka/control_types.h>
#include <franka/exception.h>

#include <franka_mock/backend/forward_dynamics_backend.h>
#include <franka_mock/backend/proportional_controller_backend.h>
#include <franka_mock/aescape_mock_server.h>
#include <franka_mock/server_types.h>

#include "logging_macros.h"

/*
 * Mock Server Example
 * This example demonstrates how to use the MockServer class and respond to different commands
 * from the franka::robot class.
 */
const std::string kUrdfPath =
    std::string(AESCAPE_MOCK_SOURCE_DIR) + "/resources/aescape_simulation_panda.urdf";
constexpr int kNumJoints = 7;


int main(int argc, char* argv[]) {
  if (argc < 2) {
    LOG_INFO("Usage: {} <server_ip>\nUse 127.0.0.1 if unsure.", argv[0]);
    return 1;
  }
  // Set initial state
  Eigen::VectorXd q(kNumJoints);
  q << 0.054, -1.225, -0.008, -1.820, -0.019, 0.697, -0.021;  //Idle pose
  aescape::AescapeMockServer<RobotTypes> server(std::make_shared<aescape::ProportionalControllerBackend>(q), 1000, kUrdfPath, std::string(argv[1]));
  server.run();
}
