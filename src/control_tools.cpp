// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/control_tools.h>

#include <cstring>
#include <exception>
#include <fstream>
#include <string>

#include "platform.h"

#ifdef LIBFRANKA_WINDOWS
#include <Windows.h>
#else
#include <pthread.h>
#endif

// `using std::string_literals::operator""s` produces a GCC warning that cannot be disabled, so we
// have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace franka {

bool hasRealtimeKernel() {
#ifdef LIBFRANKA_WINDOWS
  return true;
#else
  std::ifstream realtime("/sys/kernel/realtime", std::ios_base::in);
  bool is_realtime{};
  realtime >> is_realtime;
  return is_realtime;
#endif
}

bool setCurrentThreadToHighestSchedulerPriority(std::string* error_message) {
#ifdef LIBFRANKA_WINDOWS
  auto get_last_windows_error = []() -> std::string {
    DWORD error_id = GetLastError();
    LPSTR buffer = nullptr;
    size_t size = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, error_id, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)(&buffer), 0, nullptr);
    return std::string(buffer, size);
  };

  if (!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS)) {
    if (error_message != nullptr) {
      *error_message =
          "libfranka: unable to set priority for the process: "s + get_last_windows_error();
    }
    return false;
  }

  if (!SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL)) {
    if (error_message != nullptr) {
      *error_message =
          "libfranka: unable to set priority for the thread: "s + get_last_windows_error();
    }
    return false;
  }

  return true;
#else
  const int thread_priority = sched_get_priority_max(SCHED_FIFO);
  if (thread_priority == -1) {
    if (error_message != nullptr) {
      *error_message =
          "libfranka: unable to get maximum possible thread priority: "s + std::strerror(errno);
    }
    return false;
  }

  sched_param thread_param{};
  thread_param.sched_priority = thread_priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    if (error_message != nullptr) {
      *error_message = "libfranka: unable to set realtime scheduling: "s + std::strerror(errno);
    }
    return false;
  }
  return true;
#endif
}

}  // namespace franka
