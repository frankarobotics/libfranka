// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/**
 * This is a modified version of the mock server used for unit tests from the libfranka
 * code. A separate thread and command loop was added for the UDP connection, separate from
 * TCP. This code could be severely simplified for the purpose of our requirements at Aescape,
 * but this works for now.
 */

#include "mock_server.h"
#include "logging_macros.h"

#include <franka/exception.h>

#include <sstream>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>

template <typename C>
MockServer<C>::MockServer(ConnectCallbackT on_connect, uint32_t sequence_number, std::string ip, bool sim_time)
    : block_{false},
      shutdown_{false},
      continue_{false},
      initialized_{false},
      sequence_number_{sequence_number},
      on_connect_{on_connect},
      ip_(std::move(ip)),
      sim_time_(sim_time) {
  std::unique_lock<std::mutex> lock(command_mutex_);
  server_thread_ = std::thread(&MockServer<C>::serverThread, this);

  cv_.wait(lock, [this] { return initialized_; });
  lock.unlock();
  spinOnce();  // Spin to accept connections immediately.
}

template <typename C>
MockServer<C>::~MockServer() {
  shutdown_ = true;
  cv_.notify_one();
  udp_cv_.notify_one();
  server_thread_.join();

  LOG_INFO("Joined server thread");

  if (!commands_.empty()) {
    std::stringstream ss;
    ss << "Mock server did not process all commands. Unprocessed commands:" << std::endl;
    while (!commands_.empty()) {
      ss << commands_.front().first << std::endl;
      commands_.pop_front();
    }
    LOG_INFO(ss.str());
  }
}

template <typename C>
MockServer<C>& MockServer<C>::onReceiveRobotCommand(
    ReceiveRobotCommandCallbackT on_receive_robot_command) {
  ZoneScoped;

  std::lock_guard<std::mutex> _(command_mutex_);
  commands_.emplace_back("onReceiveRobotCommand", [=](Socket&, Socket& udp_socket) {
    research_interface::robot::RobotCommand robot_command;
    udp_socket.receiveBytes(&robot_command, sizeof(robot_command));
    if (on_receive_robot_command) {
      on_receive_robot_command(robot_command);
    }
  });
  return *this;
}

template <typename C>
MockServer<C>& MockServer<C>::onReceiveRobotCommandSeparateQueue(
    ReceiveRobotCommandCallbackT on_receive_robot_command) {
  ZoneScoped;

  std::lock_guard<std::mutex> _(udp_command_mutex_);
  udp_commands_.emplace_back("onReceiveRobotCommandSeparateQueue", [=](Socket&, Socket& udp_socket) {
    research_interface::robot::RobotCommand robot_command;
    do {
      ZoneScopedN("RcvOneUDPCommand");
      robot_command.message_id = 0;
      udp_socket.receiveBytes(&robot_command, sizeof(robot_command));

      TRACY_MESSAGE(msg, "sequence_number:{},robot_command.message_id:{}", MockServer<C>::sequenceNumber(), robot_command.message_id);
    } while (robot_command.message_id < sequenceNumber() && robot_command.message_id != 0);
    if (on_receive_robot_command) {
      on_receive_robot_command(robot_command);
    }
  });
  udp_cv_.notify_one();
  return *this;
}


template <typename C>
MockServer<C>& MockServer<C>::spinOnce() {
  ZoneScoped;

  std::unique_lock<std::mutex> lock(command_mutex_);
  continue_ = true;
  cv_.notify_one();
  if (block_) {
    cv_.wait(lock, [this]() { return !continue_; });
  }
  block_ = false;
  return *this;
}

template <typename C>
void MockServer<C>::ignoreUdpBuffer() {
  ignore_udp_buffer_ = true;
}

template <typename C>
void MockServer<C>::serverThread() {
  std::unique_lock<std::mutex> lock(command_mutex_);

  const char* kHostname = ip_.c_str();
  Poco::Net::ServerSocket srv;
  srv.bind({kHostname, C::kCommandPort}, true);
  srv.listen();

  initialized_ = true;

  cv_.notify_one();
  cv_.wait(lock, [this] { return continue_; });

  if (shutdown_) {
    return;
  }

  Poco::Net::SocketAddress remote_address;
  Poco::Net::StreamSocket tcp_socket = srv.acceptConnection(remote_address);
  tcp_socket.setBlocking(true);
  tcp_socket.setNoDelay(true);

  Socket tcp_socket_wrapper;
  tcp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    std::lock_guard<std::mutex> _(tcp_mutex_);
    int rv = tcp_socket.sendBytes(data, size);
    if (static_cast<int>(size) != rv) {
      throw franka::NetworkException("Send error on TCP socket");
    }
  };
  tcp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    std::lock_guard<std::mutex> _(tcp_mutex_);
    int rv = tcp_socket.receiveBytes(data, size);
    if (static_cast<int>(size) != rv) {
      throw franka::NetworkException("Receive error on TCP socket");
    }
  };

  uint16_t udp_port;
  handleCommand<typename C::Connect>(
      tcp_socket_wrapper, [&, this](const typename C::Connect::Request& request) {
        udp_port = request.udp_port;
        return on_connect_ ? on_connect_(request)
                           : typename C::Connect::Response(C::Connect::Status::kSuccess);
      });

  Poco::Net::DatagramSocket udp_socket({kHostname, 0});
  udp_socket.setBlocking(true);
  if (!sim_time_) {
    // 1ms
    udp_socket.setReceiveTimeout(Poco::Timespan(0, 1000));
  } else {
    // 100ms timeout in simulated time. On the client side, after 1s we will get a UDP timeout and close the connection
    // So we want to wait as long as possible in simulated time, but also not lose the connection if we experienced
    // a packet loss
    udp_socket.setReceiveTimeout(Poco::Timespan(0, 100000));
  }
  Socket udp_socket_wrapper;
  udp_socket_wrapper.sendBytes = [&](const void* data, size_t size) {
    std::lock_guard<std::mutex> _(udp_mutex_);
    int rv = udp_socket.sendTo(data, size, {remote_address.host(), udp_port});
    if (static_cast<int>(size) != rv) {
      throw franka::NetworkException("Send error on UDP socket");
    }
  };
  udp_socket_wrapper.receiveBytes = [&](void* data, size_t size) {
    std::lock_guard<std::mutex> _(udp_mutex_);
    try {
      int rv = udp_socket.receiveFrom(data, size, remote_address);
    if (static_cast<int>(size) != rv) {
      throw franka::NetworkException("Receive error on UDP socket");
    }
    } catch(const Poco::TimeoutException& exc) {
      std::cout << "Timeout receiving command" << std::endl;
    }
  };

  tcp_socket_wrapper_ = tcp_socket_wrapper;
  udp_socket_wrapper_ = udp_socket_wrapper;

  sendInitialState(udp_socket_wrapper);

  std::cout << "Starting two threads to handle commands" << std::endl;

  // todo: start two threads, one for tcp and one for udp
  auto udp_only_command_thread_ = std::thread(&MockServer<C>::UdpOnlyThread, this);

  while (!shutdown_) {
    ZoneScopedN("server_loop");
    cv_.wait(lock, [this] { return continue_ || shutdown_; });
    while (!commands_.empty()) {
      auto callback = commands_.front().second;
      commands_.pop_front();
      lock.unlock();
      callback(tcp_socket_wrapper_, udp_socket_wrapper_);
      lock.lock();
    }

    continue_ = false;
    cv_.notify_one();
  }
  
  udp_only_command_thread_.join();

  if (!ignore_udp_buffer_) {
    if (udp_socket.poll(Poco::Timespan(), Poco::Net::Socket::SelectMode::SELECT_READ)) {
      throw franka::NetworkException("UDP socket still has data");
    }
  }

  if (tcp_socket.poll(Poco::Timespan(), Poco::Net::Socket::SelectMode::SELECT_READ)) {
    // Received something on the TCP socket.
    // Test that the Server closed the connection.
    std::array<uint8_t, 16> buffer;

    int rv = tcp_socket.receiveBytes(buffer.data(), buffer.size());
    if (rv != 0) {
      throw franka::NetworkException("TCP socket still has data");
    }
  }
}

template <typename C>
void MockServer<C>::UdpOnlyThread() {
  std::unique_lock<std::mutex> udp_cmd_lck(udp_command_mutex_);
  while (!shutdown_) {
    ZoneScopedN("UdpOnlyThread");

    {
      ZoneScopedN("WaitForUDPCommands");
      udp_cv_.wait(udp_cmd_lck, [this]{return !udp_commands_.empty() || shutdown_;});
    }
    while (!udp_commands_.empty()) {
      auto callback = udp_commands_.front().second;
      udp_commands_.pop_front();
      udp_cmd_lck.unlock();
      callback(tcp_socket_wrapper_, udp_socket_wrapper_);
      udp_cmd_lck.lock();
    }
  }
}

template <typename C>
MockServer<C>& MockServer<C>::generic(
    std::function<void(MockServer<C>::Socket&, MockServer<C>::Socket&)> generic_command) {
  ZoneScoped;

  std::lock_guard<std::mutex> _(command_mutex_);
  commands_.emplace_back("generic", generic_command);
  return *this;
}

template <typename C>
void MockServer<C>::sendInitialState(Socket&) {}

template <>
void MockServer<RobotTypes>::sendInitialState(Socket& udp_socket) {
  ZoneScoped;

  research_interface::robot::RobotState state{};
  state.message_id = ++sequence_number_;
  udp_socket.sendBytes(&state, sizeof(state));
}

template <typename C>
MockServer<C>& MockServer<C>::doForever(std::function<bool()> callback) {
  ZoneScoped;

  std::lock_guard<std::mutex> _(command_mutex_);
  return doForever(callback, commands_.end());
}

template <typename C>
MockServer<C>& MockServer<C>::doForever(std::function<bool()> callback,
                                        typename decltype(MockServer::commands_)::iterator it) {
  ZoneScoped;

  auto callback_wrapper = [=](Socket&, Socket&) {
    std::unique_lock<std::mutex> lock(command_mutex_);
    if (shutdown_) {
      return;
    }
    size_t old_commands = commands_.size();
    lock.unlock();
    if (callback()) {
      lock.lock();
      size_t new_commands = commands_.size() - old_commands;

      // Reorder the commands added by callback to the front.
      decltype(commands_) commands(commands_.cbegin() + old_commands, commands_.cend());
      commands.insert(commands.end(), commands_.cbegin(), commands_.cbegin() + old_commands);
      commands_ = commands;

      // Insert after the new commands added by callback.
      doForever(callback, commands_.begin() + new_commands);
      lock.unlock();
    }
    std::this_thread::yield();
  };
  commands_.emplace(it, "doForever", callback_wrapper);
  return *this;
}

template class MockServer<RobotTypes>;
