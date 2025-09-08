#pragma once

// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace franka_test_utils {

inline std::string readFileToString(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return "";
  }

  std::ostringstream oss;
  oss << file.rdbuf();
  file.close();

  return oss.str();
}

// Common constants
constexpr double kEps = 1e-5;
constexpr double kForwardKinematicsEps = 1e-3;

// Helper to get URDF path from current file
inline std::string getUrdfPath(const std::string& file_path) {
  return file_path.substr(0, file_path.find_last_of("/\\") + 1) + "fr3.urdf";
}

}  // namespace franka_test_utils
