#pragma once

#include <memory>
#include <string>

#include <franka_mock/backend/backend_interface.h>
#include <franka_mock/server_types.h>

namespace aescape {

template <typename ServerType>
class AescapeMockServer {
public:
  /**
   * Start a server for a mock franka arm. How states are computed depends on the
   * backend used. run() must be called after instantiation to start the server, which will
   * continue running until the connection is stopped for some reason (i.e. the client sending a stop command).
   * 
   * In simulated time, the rate is ignored and the server runs as fast as possible. The backend is responsible for blocking
   * appropriately in its update functions.
   *
   * @tparam ServerType: Struct specifying state type, port... Use types provided in server_types.h
   * @param robot_backend: Backend used for computing robot states from commands
   * @param state_send_rate_hz: At what rate the server should send states to the client
   * @param urdfPath: Path of the urdf file to use to send the model to the client
   * @param ip: IP address of the server
   * @param sim_time: Run the server in simulated time
   */
    AescapeMockServer(std::shared_ptr<BackendInterface> robot_backend,
                      double state_send_rate_hz,
                      const std::string& urdfPath,
                      std::string ip = "127.0.0.1",
                      bool sim_time = false);

    ~AescapeMockServer();

    // Start running the server. Will wait for connection
    void run();

    // Stop the server
    void stop();

private:
    // Forward declaration
    class Impl;
    // Private implementation
    std::unique_ptr<Impl> impl_;
};

template class AescapeMockServer<RobotTypes>;

}  // namespace aescape
