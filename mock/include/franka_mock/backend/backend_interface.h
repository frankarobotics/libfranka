#pragma once

#include <research_interface/robot/rbk_types.h>

namespace aescape {
struct BackendInterface {
  /**
   * Read the newest robot state.
   * @param[out] state: The state object to fill
   * @returns True if the state was updated, false otherwise (no new state available).
   */
  virtual bool updateRobotState(research_interface::robot::RobotState& state) = 0;
  /**
   * Update backend with newest command
   * @param[in] cmd: Command to update backend with
   * @returns True if command was accepted
   */
  virtual bool updateRobotCommand(const research_interface::robot::RobotCommand& cmd) = 0;
};
}  // namespace aescape