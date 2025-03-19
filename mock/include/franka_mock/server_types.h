#pragma once

#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

struct RobotTypes {
  using Connect = research_interface::robot::Connect;
  using State = research_interface::robot::RobotState;
  static constexpr uint16_t kCommandPort = research_interface::robot::kCommandPort;
};