#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot ip>" << std::endl;
    return -1;
  }

  try {
    // Connect to robot
    franka::Robot robot(argv[1]);

    // Read robot state
    franka::RobotState state = robot.readOnce();

    // Print robot information
    std::cout << "Successfully connected to robot!" << std::endl;

    // Print end-effector position
    std::cout << "End-effector position (x, y, z): " << state.O_T_EE[12] << ", " << state.O_T_EE[13]
              << ", " << state.O_T_EE[14] << std::endl;

    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
}
