#include <franka_mock/aescape_mock_server.h>
#include "mock_server.h"
#include "logging_macros.h"
#include <tracy/Tracy.hpp>

#include <fstream>
#include <magic_enum.hpp>

/*
 * Mock Server Example
 * This example demonstrates how to use the MockServer class and respond to different commands
 * from the franka::robot class.
 */

using namespace std::string_literals;
using namespace std::placeholders;
namespace r = research_interface::robot;
using move_ctrl_mode_t = r::Move::ControllerMode;
using move_motion_mode_t = r::Move::MotionGeneratorMode;

namespace aescape {
/*
 * Convert enums from research_interface::robot::Move::ControllerMode to
 * research_interface::robot::ControllerMode
 */
r::ControllerMode convertMode(const move_ctrl_mode_t& mode) {
  switch (mode) {
    case move_ctrl_mode_t::kCartesianImpedance:
      return r::ControllerMode::kCartesianImpedance;
    case move_ctrl_mode_t::kExternalController:
      return r::ControllerMode::kExternalController;
    case move_ctrl_mode_t::kJointImpedance:
      return r::ControllerMode::kJointImpedance;
    default:
      LOG_INFO("Unknown Controller Mode");
      return r::ControllerMode::kOther;
  }
}
/*
 * Convert enums from research_interface::robot::Move::MotionGeneratorMode to
 * research_interface::robot::MotionGeneratorMode
 */
r::MotionGeneratorMode convertMode(const move_motion_mode_t& mode) {
  switch (mode) {
    case move_motion_mode_t::kCartesianPosition:
      return r::MotionGeneratorMode::kCartesianPosition;
    case move_motion_mode_t::kCartesianVelocity:
      return r::MotionGeneratorMode::kCartesianVelocity;
    case move_motion_mode_t::kJointPosition:
      return r::MotionGeneratorMode::kJointPosition;
    case move_motion_mode_t::kJointVelocity:
      return r::MotionGeneratorMode::kJointVelocity;
    default:
      return r::MotionGeneratorMode::kIdle;
  }
}

struct Modes {
  r::ControllerMode ctrl_mode{r::ControllerMode::kOther};
  r::MotionGeneratorMode motion_mode{r::MotionGeneratorMode::kIdle};
  r::RobotMode robot_mode{r::RobotMode::kIdle};
};

/*
 * Create a success move response
 */
inline r::Move::Response moveSuccessResp() {
  return r::Move::Response(r::Move::Status::kSuccess);
}

/*
 * Create a preempt stop move response
 */
inline r::Move::Response movePreemptedResp() {
  return r::Move::Response(r::Move::Status::kPreempted);
}

/*
 * Aescape Mock Server
 */
template <typename C>
class AescapeMockServer<C>::Impl : public MockServer<C> {
 public:
  Impl(
      std::shared_ptr<aescape::BackendInterface> robot_backend,
      double state_send_rate_hz,
      const std::string& urdfPath,
      std::string ip = "127.0.0.1",
      bool sim_time = false)
      : MockServer<C>(std::bind(&Impl::onConnect, this, _1),
        0, std::move(ip), sim_time),
        robot_backend_(robot_backend),
        state_send_rate_hz_(state_send_rate_hz),
        kUrdf_str_(loadUrdf(urdfPath)),
        model_buffer_(loadModelStream()),
        sim_time_(sim_time)
      {
        MockServer<C>::Initialize();
        // Grab initial state from backend
        robot_backend_->updateRobotState(state_);

      }

  void startUpdateThread(){
    update_thread_ = std::thread(&Impl::updateLoop, this);
  }
  /*
   * Handle robot.readOnce() by responding with the robot state
   */
  Impl& handleReadOnce(bool finish = false) {
    ZoneScoped;

    MockServer<C>::template onSendUDPSeparateQueue<r::RobotState>(
        [this, finish](r::RobotState& robot_state) {
          ZoneScopedN("SendState");
          std::lock_guard<std::mutex> lck(mut_);
          // Set server local state_.message_id to the server's sequence number (which is what is being sent to client)
          // the message_id is set from the sequence number in onSendUDP()
          TRACY_MESSAGE(msg, "sequence_number:{},robot_state.message_id:{}", MockServer<C>::sequenceNumber(), robot_state.message_id)

          state_.message_id = robot_state.message_id;
          robot_state = state_;
          // Mode
          if (finish) {
            LOG_INFO("    -- Stopped, send finished state");
            robot_state.robot_mode = r::RobotMode::kIdle;
            robot_state.controller_mode = r::ControllerMode::kOther;
            robot_state.motion_generator_mode = r::MotionGeneratorMode::kIdle;
          } else {
            robot_state.robot_mode = modes_.robot_mode;
            robot_state.controller_mode = modes_.ctrl_mode;
            robot_state.motion_generator_mode = modes_.motion_mode;
          }
        });
    return *this;
  }

  /*
   * Handle robot.update() by sending the robot state and receiving the robot command
   */
  AescapeMockServer<C>::Impl& handleUpdate() {
    ZoneScoped;

    handleReadOnce().onReceiveRobotCommandSeparateQueue(
        [this](const r::RobotCommand& cmd) {
          ZoneScopedN("OnReceiveRobotCommand");
          //LOG_INFO("    -- Received control robot command");
          std::lock_guard<std::mutex> state_lck(mut_);
          if (cmd.motion.motion_generation_finished) {
            LOG_INFO("    -- Motion generation finished");
            modes_.motion_mode = r::MotionGeneratorMode::kIdle;
            modes_.ctrl_mode = r::ControllerMode::kOther;
            stopped_ = true;
            MockServer<C>::close();
          } else if (cmd.message_id == 0) {
            // We never receive a robot command until the sequence number is > 0, as we
            // send an initial state. So a message_id == 0 means that we didn't receive a command
            LOG_INFO("Command timed out");
            cmd_timeout_ = true;
          } else {
            cmd_ = cmd;
          }
          cv_.notify_all();
        });
    return *this;
  }

  /*
   * Wait for a move command and responding with a success
   */
  AescapeMockServer<C>::Impl& waitForMove() {
    ZoneScoped;

    MockServer<C>::template waitForCommand<r::Move>(
        [&](const r::Move::Request& req) {
          ZoneScopedN("WaitForMove");
          std::unique_lock<std::mutex> state_lck(mut_);
          {
            ZoneScopedN("WaitForMoveLocked");
            modes_.motion_mode = convertMode(req.motion_generator_mode);
            modes_.ctrl_mode = convertMode(req.controller_mode);
            modes_.robot_mode = r::RobotMode::kMove;
            LOG_INFO("    -- Move Received: Controller Mode: {}, Motion Mode: {}",
                    magic_enum::enum_name(modes_.ctrl_mode),
                    magic_enum::enum_name(modes_.motion_mode));
          }
          LOG_INFO("Stopped Update Thread");
	   // We don't call stopUpdateThread() here because joining a thread that sleeps would potentially block us
          // and we want to respond ASAP
          stop_update_thread_ = true;
          state_lck.unlock();

          return moveSuccessResp();
        },
        &move_id_);

    return *this;
  }

  /*
   * Respond to the move command with a success without waiting for a request
   */
  AescapeMockServer<C>::Impl& respondMove() {
    ZoneScoped;

    std::unique_lock<std::mutex> lck(mut_);
    if (stopped_) {
      LOG_INFO("    -- Stopped, send preempted response");
      MockServer<C>::template sendResponse<r::Move>(move_id_, movePreemptedResp);
    } else {
      LOG_INFO("    -- Moving send success response");
      MockServer<C>::template sendResponse<r::Move>(move_id_, moveSuccessResp);
    }
    return *this;
  }

  /*
   * Override MockServer SpinOnce to return the derived AescapeMockServer class instead
   */
  AescapeMockServer<C>::Impl& spinOnce() override {
    MockServer<C>::spinOnce();
    return *this;
  }

  void updateLoop() {
    ZoneScoped;

    std::unique_lock<std::mutex> lck(mut_);

    // Run without update if we are not in state loop and not stopped
    while (!stop_update_thread_) {
      lck.unlock();
      handleReadOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      lck.lock();
    }
  }

  void stateLoop() {
    /**
     * Send states and wait for commands from the client. Everytime a command is received, we compute
     * the next state and send it back.
     */
    ZoneScoped;
    using clock = std::chrono::steady_clock;
    uint64_t cycles = 0;
    auto init_time = clock::now();
    uint64_t dt_us = 1e6 / state_send_rate_hz_;

    std::unique_lock<std::mutex> mut(mut_);

    while (!stopped_) {
      ZoneScopedN("SendStateLoop");

      uint64_t nxt_seq_num = MockServer<C>::sequenceNumber() + 1;

      mut.unlock();

      // Start of the loop, determine when the next loop is supposed to run
      auto deadline = std::chrono::microseconds(++cycles*dt_us) + init_time;

      // Send current state back to client and wait for next command, or command rcv timeout
      handleUpdate();


      mut.lock();

      {
        ZoneScopedN("Wait for command notify");
        TRACY_MESSAGE(msg, "seq_num:{},cmd.message_id:{}", nxt_seq_num, cmd_.message_id);

        // Wait until we receive the expected command, or if we get a timeout
        auto wait_fn = [this, nxt_seq_num, cycles]{return cmd_.message_id == nxt_seq_num || cmd_timeout_ || stopped_;};

        cv_.wait(mut, wait_fn);
      }

      r::RobotCommand cmd = cmd_;
      r::RobotState state = state_;
      
      mut.unlock();

      if (!cmd_timeout_) {
        // Do nothing, we will resend the previous state at the next iteration.
        // Alternatively, we could still update the state without updating the command
        robot_backend_->updateRobotCommand(cmd);
      }

      robot_backend_->updateRobotState(state);

      if (!sim_time_) {
        // Sleep until appropriate to send the computed state. In sim time, we expect the backend to regulate time by
        // blocking in updateRobotState(). In other words, in simulated time, we send states in lockstep with their computation.
        // It would probably be a cleaner approach to pass in a clock object with a sleep function (MOT-2036)
        auto prev_time = clock::now();
        auto tts = std::chrono::duration_cast<std::chrono::microseconds>(deadline - prev_time);
        std::this_thread::sleep_for(tts);
      }

      mut.lock();
      // Update state under lock
      state_ = state;
      // Reset timeout
      cmd_timeout_ = false;
    }
    LOG_INFO("Getting out of state loop");
  }

  void stopUpdateLoop() {
    ZoneScoped;
    std::unique_lock<std::mutex> lck(mut_);
    stop_update_thread_ = true;
    lck.unlock();
    cv_.notify_all();
    if (update_thread_.joinable()) {
      update_thread_.join();
    }
  }

  void stopStateLoop() {
    ZoneScoped;
    std::lock_guard<std::mutex> lck(mut_);
    if (!stopped_)  {
      stopped_ = true;
      cv_.notify_all();
    }
  }

  void stopMotionThreadFn() {
    ZoneScoped;
    bool stop_move = false;

    try {
      MockServer<C>::template waitForCommand<r::StopMove>([this, &stop_move](const r::StopMove::Request&) {
        ZoneScopedN("StopMove_recv");
        LOG_INFO(" ++ Received StopMove");
        std::unique_lock<std::mutex> lck(mut_);
        stop_move = true;
        cv_.notify_all();

        return r::StopMove::Response(r::StopMove::Status::kSuccess);
      }).spinOnce();
    } catch (const franka::NetworkException& e) {
      LOG_ERROR("Shutting down mock server, Exception: {}", e.what());
      stop_move = true;
    }

    std::unique_lock<std::mutex> lck(mut_);
    cv_.wait(lck, [this, &stop_move] {return stop_move || stopped_;});
    if (!stopped_) {
      lck.unlock();
      LOG_INFO("Stopping state loop");
      stopStateLoop();
    }
  }

  void stopStopMotionThread() {
    ZoneScoped;

    // This should be called when stopping
    assert(stopped_);

    if (stop_thread_.joinable()){
      stop_thread_.join();
    }
  }

  void stop() {
    stopUpdateLoop();
    // Stop thread should join after stopStateLoop too
    stopStateLoop();
    stopStopMotionThread();

    MockServer<C>::Shutdown();
  }

  void run() {
    std::unique_lock<std::mutex> lck(mut_);

    while (!connected_) {
      LOG_INFO("Waiting for connection...");
      cv_.wait(lck);
    }
    lck.unlock();

    // ReadOnce() needs states to be sent, but trying to sync on timing with the client is tricky
    // Easier to have a thread send states at init and then kill it once we get into Move mode
    startUpdateThread();

    try{
      // Handle init methods from the controls stack
      MockServer<C>::template waitForCommand<r::GetRobotModel>([this](const typename r::GetRobotModel::Request& /*request*/) {
           ZoneScopedN("LoadModelCommand");
           LOG_INFO("--- Receive load model request -> loadModel");
           LOG_INFO(" ++ GetRobotModel");
           return r::GetRobotModel::Response(r::GetRobotModel::Status::kSuccess, kUrdf_str_);
         })
         .spinOnce()
         .generic([this](typename MockServer<C>::TCPSocket& tcp_socket, typename MockServer<C>::UDPSocket&) {
           ZoneScopedN("LoadModelResponse");
           LOG_INFO(" ++ LoadModelLibrary");
           r::CommandHeader header;
           this->template receiveRequest<r::LoadModelLibrary>(tcp_socket, &header);
           this->template sendResponse<r::LoadModelLibrary>(
               tcp_socket,
               r::CommandHeader(
                   r::Command::kLoadModelLibrary, header.command_id,
                   sizeof(r::CommandMessage<r::LoadModelLibrary::Response>) + model_buffer_.size()),
               r::LoadModelLibrary::Response(r::LoadModelLibrary::Status::kSuccess));
           tcp_socket.sendBytes(model_buffer_.data(), model_buffer_.size());
         })
         .spinOnce()
         .template waitForCommand<r::AutomaticErrorRecovery>(
             [](const typename r::AutomaticErrorRecovery::Request&) {
               ZoneScopedN("ErrorRecoveryCmd");
               LOG_INFO("--- Automatic Error Recovery");
               return r::AutomaticErrorRecovery::Response(r::AutomaticErrorRecovery::Status::kSuccess);
             })
         .spinOnce()
         .template waitForCommand<r::SetCollisionBehavior>(
             [](const typename r::SetCollisionBehavior::Request&) {
               ZoneScopedN("CollisonBehaviorCmd");
               LOG_INFO("--- SetCollisionBehavior");
               return r::SetCollisionBehavior::Response(r::SetCollisionBehavior::Status::kSuccess);
             })
         .spinOnce()
         .template waitForCommand<r::SetJointImpedance>([](const typename r::SetJointImpedance::Request&) {
           ZoneScopedN("JointImpedanceCmd");
           LOG_INFO("--- SetJointImpedance");
           return r::SetJointImpedance::Response(r::SetJointImpedance::Status::kSuccess);
         })
         .spinOnce()
         .template waitForCommand<r::SetCartesianImpedance>(
             [](const typename r::SetCartesianImpedance::Request&) {
               ZoneScopedN("CartesianImpedanceCmd");
               LOG_INFO("--- SetCartesianImpedance");
               return r::SetCartesianImpedance::Response(r::SetCartesianImpedance::Status::kSuccess);
             })
         .spinOnce();
 

      LOG_INFO("--- Read current state ->readOnce (Handle by update thread)");  // Can miss. Try twice here.

      LOG_INFO("--- Run control loop ->control");
      LOG_INFO(" ++ ControlLoop Constructor");
      LOG_INFO("    -- Handle robot.startMotion()");
      // Send Move response again for startMotion haven't done a RobotState update yet
      // and it checks for a Move response. It does not send another request so simply send a response


      waitForMove().spinOnce();
      // Handle extraneous move responses sent.
      //respondMove().spinOnce();
      LOG_INFO("    -- Handle robot.update() (Handle by update thread)");
      LOG_INFO(" ++ ControlLoop Constructor DONE");

      LOG_INFO(" ++ operator()()");

      std::this_thread::sleep_for(std::chrono::milliseconds(2));

      stop_thread_ = std::thread(&Impl::stopMotionThreadFn, this);

      stateLoop();

      LOG_INFO("    -- Finish control callback loop");

      LOG_INFO("    -- Start robot.finishMotion");

      std::this_thread::sleep_for(std::chrono::milliseconds(4));

      handleReadOnce(true).spinOnce()
          .respondMove()  // No request. Only waiting for response.
          .spinOnce()
          .handleReadOnce(true).spinOnce();
      LOG_INFO("    -- Finish robot.finishMotion");

      LOG_INFO("Joining Stop Thread");
      stop_thread_.join();

      MockServer<C>::template waitForCommand<r::StopMove>([this](const r::StopMove::Request&) {
        ZoneScopedN("StopMove_recv2");
        LOG_INFO(" ++ Received StopMove 2");
        return r::StopMove::Response(r::StopMove::Status::kSuccess);
      }).spinOnce();
    }
    catch (const franka::NetworkException& e) {
      LOG_ERROR("Shutting down mock server, Exception: {}", e.what());
      stop();
    }
  }

  ~Impl() 
  {
    stop();
    MockServer<C>::ignoreUdpBuffer();
    std::cout << "Destructor call finished" << std::endl;
  }

 private:
  RobotTypes::Connect::Response onConnect(const RobotTypes::Connect::Request& req) {
    ZoneScoped;
    LOG_INFO("Connected! port: {}", req.udp_port);
    std::unique_lock<std::mutex> lck(mut_);
    connected_ = true;
    cv_.notify_all();
    return RobotTypes::Connect::Response(RobotTypes::Connect::Status::kSuccess);
  }

  std::vector<char> loadModelStream() {
    // Prepare for load model
    std::ifstream model_library_stream(
        AESCAPE_MOCK_BINARY_DIR + "/libaescape_fcimodels.so"s,
        std::ios_base::in | std::ios_base::binary | std::ios_base::ate);
    std::vector<char> buffer;
    buffer.resize(model_library_stream.tellg());
    model_library_stream.seekg(0, std::ios::beg);
    if (!model_library_stream.read(buffer.data(), buffer.size())) {
      throw std::runtime_error("Model test: Cannot load mock libfcimodels.so");
    }

    return buffer;
  }

  std::string loadUrdf(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      std::cerr << "Error opening file: " << path << std::endl;
      return "";
    }

    std::ostringstream oss;
    oss << file.rdbuf();
    file.close();

    return oss.str();
  }
  
  std::shared_ptr<aescape::BackendInterface> robot_backend_;
  double state_send_rate_hz_;
  std::string kUrdf_str_;
  std::vector<char> model_buffer_;
  bool sim_time_;
  std::mutex mut_;
  std::condition_variable cv_;
  Modes modes_{};
  uint32_t move_id_;
  r::RobotCommand cmd_{};
  r::RobotState state_{};
  bool stopped_{false};
  bool connected_{false};
  std::thread update_thread_;
  bool stop_update_thread_{false};
  std::thread stop_thread_;
  bool cmd_timeout_{false};
};

template <typename C>
AescapeMockServer<C>::AescapeMockServer(std::shared_ptr<aescape::BackendInterface> robot_backend,
    double state_send_rate_hz,
    const std::string& urdf_path,
    std::string ip, bool sim_time) :
        impl_(std::make_unique<Impl>(robot_backend, state_send_rate_hz, urdf_path, ip, sim_time)) {}


template <typename C>
AescapeMockServer<C>::~AescapeMockServer() {
  stop();
}

template <typename C>
void AescapeMockServer<C>::run() {
    impl_->run();
}

template <typename C>
void AescapeMockServer<C>::stop() {
    impl_->stop();
}

}