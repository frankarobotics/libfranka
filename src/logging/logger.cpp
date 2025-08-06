// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <map>

#include <franka/logging/logger.hpp>

namespace franka::logging {

std::map<std::string, std::shared_ptr<LoggingSinkInterface>>
    loggers;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

auto addLogger(const std::shared_ptr<LoggingSinkInterface>& logger) -> void {
  loggers.emplace(logger->getName(), logger);
}

auto removeLogger(const std::string& logger_name) -> void {
  loggers.erase(logger_name);
}

auto removeAllLoggers() -> void {
  loggers.clear();
}

auto logInfo(const std::string& message) -> void {
  for (const auto& [logger_name, logger] : loggers) {
    logger->logInfo(message);
  }
}

auto logWarn(const std::string& message) -> void {
  for (const auto& [logger_name, logger] : loggers) {
    logger->logWarn(message);
  }
}

auto logError(const std::string& message) -> void {
  for (const auto& [logger_name, logger] : loggers) {
    logger->logError(message);
  }
}

}  // namespace franka::logging
