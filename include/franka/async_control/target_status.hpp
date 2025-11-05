// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

namespace franka {

/**
 * Describes the current status of the asynchronous control.
 */
enum class TargetStatus { kIdle, kExecuting, kTargetReached, kAborted };

}  // namespace franka
