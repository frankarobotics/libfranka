// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <franka/robot_state.h>
#include <research_interface/gripper/types.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>
#include <research_interface/vacuum_gripper/types.h>

struct RobotTypes {
  using Connect = research_interface::robot::Connect;
  using State = research_interface::robot::RobotState;
  static constexpr uint16_t kCommandPort = research_interface::robot::kCommandPort;
};

struct GripperTypes {
  using Connect = research_interface::gripper::Connect;
  using State = research_interface::gripper::GripperState;
  static constexpr uint16_t kCommandPort = research_interface::gripper::kCommandPort;
};

struct VacuumGripperTypes {
  using Connect = research_interface::vacuum_gripper::Connect;
  using State = research_interface::vacuum_gripper::VacuumGripperState;
  static constexpr uint16_t kCommandPort = research_interface::vacuum_gripper::kCommandPort;
};

template <typename C>
class MockServer {
 public:
  struct Socket {
    std::function<void(const void*, size_t)> sendBytes;
    std::function<void(void*, size_t)> receiveBytes;
  };

  using ConnectCallbackT =
      std::function<typename C::Connect::Response(const typename C::Connect::Request&)>;
  using ReceiveRobotCommandCallbackT =
      std::function<void(const research_interface::robot::RobotCommand&)>;

  MockServer(ConnectCallbackT on_connect = ConnectCallbackT(), uint32_t sequence_number = 0);
  ~MockServer();

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

  MockServer& onReceiveRobotCommand(
      ReceiveRobotCommandCallbackT on_receive_robot_command = ReceiveRobotCommandCallbackT());

  MockServer& generic(std::function<void(Socket&, Socket&)> generic_command);

  template <typename T>
  typename T::Request receiveRequest(Socket& tcp_socket, typename T::Header* header = nullptr);

  template <typename T>
  void sendResponse(Socket& tcp_socket,
                    const typename T::Header& header,
                    const typename T::Response& response);

  template <typename T>
  void handleCommand(Socket& tcp_socket,
                     std::function<typename T::Response(const typename T::Request&)> callback,
                     uint32_t* command_id = nullptr);

  template <typename T>
  MockServer& waitForCommand(
      std::function<typename T::Response(const typename T::Request&)> callback,
      uint32_t* command_id = nullptr);

  MockServer& spinOnce();

  template <typename T>
  T randomState();

  uint32_t sequenceNumber() const { return sequence_number_; }

  void ignoreUdpBuffer();

 private:
  void serverThread();
  void sendInitialState(Socket& udp_socket);
  void initializeServer();
  void addDefaultRobotModelHandler();  // Specialized for RobotTypes

  template <typename T>
  MockServer& onSendUDP(std::function<T()> on_send_udp);

  std::condition_variable_any cv_;
  std::mutex command_mutex_;
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
  std::deque<std::pair<std::string, std::function<void(Socket&, Socket&)>>> commands_;

  MockServer& doForever(std::function<bool()> callback,
                        typename decltype(MockServer::commands_)::iterator it);
};

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::sendResponse(const uint32_t& command_id,
                                           std::function<typename T::Response()> create_response) {
  using namespace std::string_literals;

  std::lock_guard<std::mutex> _(command_mutex_);
  block_ = true;
  commands_.emplace_back(
      "sendResponse<"s + typeid(typename T::Response).name() + ">",
      [=, &command_id](Socket& tcp_socket, Socket&) {
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
  using namespace std::string_literals;

  std::lock_guard<std::mutex> _(command_mutex_);
  commands_.emplace_back(
      "sendResponse<"s + typeid(typename T::Response).name() + ">",
      [=, &command_id](Socket& tcp_socket, Socket&) {
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
  return onSendUDP<T>([=](T& state) { state.message_id = ++sequence_number_; });
}

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::sendRandomState(std::function<void(T&)> random_generator,
                                              T* sent_state) {
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
MockServer<C>& MockServer<C>::onSendUDP(std::function<T()> on_send_udp) {
  std::lock_guard<std::mutex> _(command_mutex_);
  commands_.emplace_back("onSendUDP", [=](Socket&, Socket& udp_socket) {
    T state = on_send_udp();
    udp_socket.sendBytes(&state, sizeof(state));
  });
  block_ = true;
  return *this;
}

template <typename C>
template <typename T>
MockServer<C>& MockServer<C>::onSendUDP(std::function<void(T&)> on_send_udp) {
  return onSendUDP<T>([=]() {
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
typename T::Request MockServer<C>::receiveRequest(Socket& tcp_socket,
                                                  typename T::Header* header_ptr) {
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
void MockServer<C>::sendResponse(Socket& tcp_socket,
                                 const typename T::Header& header,
                                 const typename T::Response& response) {
  typename T::template Message<typename T::Response> response_message(header, response);
  tcp_socket.sendBytes(&response_message, sizeof(response_message));
}

template <>
template <>
inline void MockServer<RobotTypes>::sendResponse<research_interface::robot::GetRobotModel>(
    Socket& tcp_socket,
    const research_interface::robot::GetRobotModel::Header& header,
    const research_interface::robot::GetRobotModel::Response& response) {
  research_interface::robot::GetRobotModel::Message<
      research_interface::robot::GetRobotModel::Response>
      response_message(header, response);
  auto byte_vector = response_message.serialize();
  tcp_socket.sendBytes(byte_vector.data(), byte_vector.size());
}

template <typename C>
template <typename T>
void MockServer<C>::handleCommand(
    Socket& tcp_socket,
    std::function<typename T::Response(const typename T::Request&)> callback,
    uint32_t* command_id) {
  typename T::Header header{};
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
  using namespace std::string_literals;

  std::lock_guard<std::mutex> _(command_mutex_);
  std::string name = "waitForCommand<"s + typeid(typename T::Request).name() + ", " +
                     typeid(typename T::Response).name();
  commands_.emplace_back(name, [this, callback, command_id](Socket& tcp_socket, Socket&) {
    handleCommand<T>(tcp_socket, callback, command_id);
  });
  return *this;
}

using RobotMockServer = MockServer<RobotTypes>;
using GripperMockServer = MockServer<GripperTypes>;
using VacuumGripperMockServer = MockServer<VacuumGripperTypes>;
