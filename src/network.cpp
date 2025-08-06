// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "network.h"

#include <memory>
#include <sstream>

using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace franka {

Network::Network(const std::string& franka_address,
                 uint16_t franka_port,
                 std::chrono::milliseconds tcp_timeout,
                 std::chrono::milliseconds udp_timeout,
                 std::tuple<bool, int, int, int> tcp_keepalive) {
  try {
    Poco::Timespan poco_timeout(1000L * tcp_timeout.count());
    Poco::Net::SocketAddress address(franka_address, franka_port);

    tcp_socket_.connect(address, poco_timeout);
    tcp_socket_.setBlocking(true);
    tcp_socket_.setSendTimeout(poco_timeout);
    tcp_socket_.setReceiveTimeout(poco_timeout);

    if (std::get<0>(tcp_keepalive)) {
      tcp_socket_.setKeepAlive(true);
      try {
        tcp_socket_.setOption(IPPROTO_TCP, TCP_KEEPIDLE, std::get<1>(tcp_keepalive));
        tcp_socket_.setOption(IPPROTO_TCP, TCP_KEEPCNT, std::get<2>(tcp_keepalive));
        tcp_socket_.setOption(IPPROTO_TCP, TCP_KEEPINTVL, std::get<3>(tcp_keepalive));
      } catch (...) {
      }
    }

    udp_socket_.bind({"0.0.0.0", 0});
    udp_socket_.setReceiveTimeout(Poco::Timespan{1000L * udp_timeout.count()});
    udp_port_ = udp_socket_.address().port();
  } catch (const Poco::Net::ConnectionRefusedException& e) {
    throw NetworkException(
        "libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk."s);
  } catch (const Poco::Net::NetException& e) {
    throw NetworkException("libfranka: Connection error: "s + e.what());
  } catch (const Poco::TimeoutException& e) {
    throw NetworkException(
        "libfranka: Connection timeout. Please check your network connection or settings."s);
  } catch (const Poco::Exception& e) {
    throw NetworkException("libfranka: "s + e.what());
  }
}

Network::~Network() {
  try {
    tcp_socket_.shutdown();
  } catch (...) {
  }
}

uint16_t Network::udpPort() const noexcept {
  return udp_port_;
}

bool Network::isTcpSocketAlive() const noexcept {
  return !tcp_socket_.poll(Poco::Timespan(0), Poco::Net::Socket::SELECT_ERROR);
}

void Network::tcpThrowIfConnectionClosed() try {
  std::unique_lock<std::mutex> lock(tcp_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
    std::array<uint8_t, 1> buffer;
    int received_bytes =
        tcp_socket_.receiveBytes(buffer.data(), static_cast<int>(buffer.size()), MSG_PEEK);

    if (received_bytes == 0) {
      throw NetworkException("libfranka: server closed connection");
    }
  }
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: "s + e.what());
}

}  // namespace franka
