// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/**
 * This is a modified version of the mock server used for unit tests from the libfranka
 * code. A separate thread and command loop was added for the UDP connection, separate from
 * TCP. This code could be severely simplified for the purpose of our requirements at Aescape,
 * but this works for now.
 */
#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <tracy/Tracy.hpp>

#include <Poco/Net/StreamSocket.h>

#include <franka/robot_state.h>
#include <franka/exception.h>

#include <franka_mock/server_types.h>
#include "logging_macros.h"

template <typename C>
class MockServer {
 public:
  struct UDPSocket {
    std::function<void(const void*, size_t)> sendBytes;
    std::function<void(void*, size_t)> receiveBytes;
  };

  struct TCPSocket {
    std::function<void(const void*, size_t)> sendBytes;
    std::function<void(void*, size_t)> receiveBytes;
    Poco::Net::StreamSocket socket_impl;
  };

  using ConnectCallbackT =
      std::function<typename C::Connect::Response(const typename C::Connect::Request&)>;
  using ReceiveRobotCommandCallbackT =
      std::function<void(const research_interface::robot::RobotCommand&)>;

  MockServer(ConnectCallbackT on_connect = ConnectCallbackT(),
             uint32_t sequence_number = 0,
             std::string ip = "127.0.0.1",
             bool sim_time = false);
  virtual ~MockServer();


  void Initialize();

  void Shutdown();

  template <typename T>
  MockServer& sendEmptyState();

  template <typename T>
  MockServer& sendResponse(const uint32_t& command_id,
                           std::function<typename T::Response()> create_response);

  template <typename T>
  MockServer& queueResponse(const uint32_t& command_id,
                            std::function<typename T::Response()> create_response);

  template <typename T>
  MockServer& sendRandomState(std::function<void(T&)> random_generator, T* sent_state = nullptr);

  MockServer& doForever(std::function<bool()> callback);

  template <typename T>
  MockServer& onSendUDP(std::function<void(T&)> on_send_udp);

  template <typename T>
  MockServer& onSendUDPSeparateQueue(std::function<void(T&)> on_send_udp);

  MockServer& onReceiveRobotCommand(
      ReceiveRobotCommandCallbackT on_receive_robot_command = ReceiveRobotCommandCallbackT());

  MockServer& onReceiveRobotCommandSeparateQueue(
      ReceiveRobotCommandCallbackT on_receive_robot_command = ReceiveRobotCommandCallbackT());

  MockServer& generic(std::function<void(TCPSocket&, UDPSocket&)> generic_command);

  template <typename T>
  typename T::Request receiveRequest(TCPSocket& tcp_socket, typename T::Header* header = nullptr);

  template <typename T>
  void sendResponse(TCPSocket& tcp_socket,
                    const typename T::Header& header,
                    const typename T::Response& response);

  template <typename T>
  void handleCommand(TCPSocket& tcp_socket,
                     std::function<typename T::Response(const typename T::Request&)> callback,
                     uint32_t* command_id = nullptr);

  template <typename T>
  MockServer& waitForCommand(
      std::function<typename T::Response(const typename T::Request&)> callback,
      uint32_t* command_id = nullptr);

  virtual MockServer& spinOnce();

  template <typename T>
  T randomState();

  uint32_t sequenceNumber() const { return sequence_number_; }

  void ignoreUdpBuffer();

  void close() {
    ZoneScoped;
    LOG_ERROR("Forcibly closing TCP socket");
    tcp_socket_wrapper_.socket_impl.shutdown();
  }

 private:
  void serverThread();
  void UdpOnlyThread();

  void sendInitialState(UDPSocket& udp_socket);

  template <typename T>
  MockServer& onSendUDP(std::function<T()> on_send_udp);

  template <typename T>
  MockServer& onSendUDPSeparateQueue(std::function<T()> on_send_udp);

  std::condition_variable_any cv_;
  TracyLockable(std::mutex, command_mutex_);
  std::condition_variable_any udp_cv_;
  TracyLockable(std::mutex, udp_command_mutex_);
  std::mutex tcp_mutex_;
  std::mutex udp_mutex_;
  std::thread server_thread_;
  bool block_;
  std::atomic<bool> shutdown_;
  bool continue_;
  bool initialized_;
  uint32_t sequence_number_;
  bool ignore_udp_buffer_ = false;

  const ConnectCallbackT on_connect_;
  std::string ip_;

  bool sim_time_;

  TCPSocket tcp_socket_wrapper_;
  UDPSocket udp_socket_wrapper_;

  std::deque<std::pair<std::string, std::function<void(TCPSocket&, UDPSocket&)>>> commands_;
  std::deque<std::pair<std::string, std::function<void(TCPSocket&, UDPSocket&)>>> udp_commands_;

  MockServer& doForever(std::function<bool()> callback,
                        typename decltype(MockServer::commands_)::iterator it);
};

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::sendResponse(const uint32_t& command_id,
                                           std::function<typename T::Response()> create_response) {
  ZoneScoped;

  using namespace std::string_literals;

  std::lock_guard _(command_mutex_);

  if (shutdown_) {
    throw franka::NetworkException("MockServer is shutting down");
  }

  block_ = true;
  commands_.emplace_back(
      "sendResponse<"s + typeid(typename T::Response).name() + ">",
      [=, &command_id](TCPSocket& tcp_socket, UDPSocket&) {
        ZoneScoped;
        TRACY_MESSAGE(msg, "command_id:{}", command_id);
        typename T::template Message<typename T::Response> message(
            typename T::Header(T::kCommand, command_id,
                               sizeof(typename T::template Message<typename T::Response>)),
            create_response());
        tcp_socket.sendBytes(&message, sizeof(message));
      });
  return *this;
}

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::queueResponse(const uint32_t& command_id,
                                            std::function<typename T::Response()> create_response) {
  ZoneScoped;

  using namespace std::string_literals;

  std::lock_guard _(command_mutex_);

  if (shutdown_) {
    throw franka::NetworkException("MockServer is shutting down");
  }

  commands_.emplace_back(
      "sendResponse<"s + typeid(typename T::Response).name() + ">",
      [=, &command_id](TCPSocket& tcp_socket, UDPSocket&) {
        typename T::template Message<typename T::Response> message(
            typename T::Header(T::kCommand, command_id,
                               sizeof(typename T::template Message<typename T::Response>)),
            create_response());
        tcp_socket.sendBytes(&message, sizeof(message));
      });
  return *this;
}

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::sendEmptyState() {
  ZoneScoped;

  return onSendUDP<T>([=](T& state) { state.message_id = ++sequence_number_; });
}

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::sendRandomState(std::function<void(T&)> random_generator,
                                              T* sent_state) {
  ZoneScoped;

  return onSendUDP<T>([=](T& state) {
    random_generator(state);
    state.message_id = ++sequence_number_;
    if (sent_state != nullptr) {
      *sent_state = state;
    }
  });
}


template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::onSendUDPSeparateQueue(std::function<T()> on_send_udp) {
  ZoneScoped;

  std::lock_guard _(udp_command_mutex_);

  if (shutdown_) {
    throw franka::NetworkException("MockServer is shutting down");
  }

  udp_commands_.emplace_back("onSendUDPSeparateQueue", [=](TCPSocket&, UDPSocket& udp_socket) {
    T state = on_send_udp();
    udp_socket.sendBytes(&state, sizeof(state));
  });
  udp_cv_.notify_one();
  return *this;
}

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::onSendUDPSeparateQueue(std::function<void(T&)> on_send_udp) {
  ZoneScopedN("onSendUDPSeparateQueue_2");

  return onSendUDPSeparateQueue<T>([=]() {
    T state{};
    state.message_id = ++sequence_number_;
    if (on_send_udp) {
      on_send_udp(state);
    }
    return state;
  });
}

template <typename C>
template <typename T>
typename T::Request MockServer<C>::receiveRequest(TCPSocket& tcp_socket,
                                                  typename T::Header* header_ptr) {
  ZoneScoped;

  typename T::template Message<typename T::Request> request_message;
  if constexpr (std::is_same<research_interface::robot::GetRobotModel, T>::value) {
    tcp_socket.receiveBytes(&request_message.header, sizeof(request_message.header));
  } else {
    tcp_socket.receiveBytes(&request_message, sizeof(request_message));
  }
  if (header_ptr != nullptr) {
    *header_ptr = request_message.header;
  }
  return request_message.getInstance();
}

template <typename C>
template <typename T>
void MockServer<C>::sendResponse(TCPSocket& tcp_socket,
                                 const typename T::Header& header,
                                 const typename T::Response& response) {
  ZoneScoped;

  typename T::template Message<typename T::Response> response_message(header, response);
  tcp_socket.sendBytes(&response_message, sizeof(response_message));
}

template <>
template <>
inline void MockServer<RobotTypes>::sendResponse<research_interface::robot::GetRobotModel>(
    TCPSocket& tcp_socket,
    const research_interface::robot::GetRobotModel::Header& header,
    const research_interface::robot::GetRobotModel::Response& response) {
  ZoneScoped;

  research_interface::robot::GetRobotModel::Message<
      research_interface::robot::GetRobotModel::Response>
      response_message(header, response);
  auto byte_vector = response_message.serialize();
  tcp_socket.sendBytes(byte_vector.data(), byte_vector.size());
}

template <typename C>
template <typename T>
void MockServer<C>::handleCommand(
    TCPSocket& tcp_socket,
    std::function<typename T::Response(const typename T::Request&)> callback,
    uint32_t* command_id) {
  ZoneScoped;

  typename T::Header header;
  typename T::Request request = receiveRequest<T>(tcp_socket, &header);
  if (command_id != nullptr) {
    *command_id = header.command_id;
  }
  sendResponse<T>(tcp_socket,
                  typename T::Header(T::kCommand, header.command_id,
                                     sizeof(typename T::template Message<typename T::Response>)),
                  callback(request));
}

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::waitForCommand(
    std::function<typename T::Response(const typename T::Request&)> callback,
    uint32_t* command_id) {
  ZoneScoped;

  using namespace std::string_literals;

  std::lock_guard _(command_mutex_);

  if (shutdown_) {
    throw franka::NetworkException("MockServer is shutting down");
  }

  std::string name = "waitForCommand<"s + typeid(typename T::Request).name() + ", " +
                     typeid(typename T::Response).name();
  commands_.emplace_back(name, [this, callback, command_id](TCPSocket& tcp_socket, UDPSocket&) {
    handleCommand<T>(tcp_socket, callback, command_id);
  });
  return *this;
}

using RobotMockServer = MockServer<RobotTypes>;
