#include <fcntl.h>
#include <mavsdk/connection_result.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/manual_control/manual_control.h>
#include <thread>
#include <unistd.h>

#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "cereal/gen/cpp/custom.capnp.h"
#include <capnp/message.h>

#include <capnp/serialize-packed.h>

static bool isFileDescriptiorValid(int client_socket_fd) {
  return fcntl(client_socket_fd, F_GETFD) != -1 || errno != EBADF;
}

struct BridgeConfig {
  float steering_rate = 0.25;
  float throttle_rate = 5.0;
  uint16_t port = 4069;
  //Generic_Port *port;
};

class MavlinkBridge {
public:
  explicit MavlinkBridge(const BridgeConfig config = {});
  ~MavlinkBridge() {
    if (isFileDescriptiorValid(log_file_fd_)) {
      close(log_file_fd_);
    }
  };
  void run(const std::string);

private:
  void send_odometry(int);
  void apply_command(int);
  void takeoff(int);
  void land(int);
  void on_odometry(mavsdk::Telemetry::Odometry);
  void on_velocity(mavsdk::Telemetry::PositionVelocityNed);
  void on_position(mavsdk::Telemetry::Position);
  bool connect(const std::string);
  bool bind_telemetry();
  void run_tcp_server(const uint16_t port);
  float get_yaw();
  int log_file_fd_ = -1;
  float last_angle = 0;
  mavsdk::Telemetry::Odometry last_odometry;
  mavsdk::Telemetry::PositionVelocityNed last_position_velocity;
  mavsdk::Telemetry::Position last_position;
  mavsdk::Mavsdk mavsdk_;
  BridgeConfig config_;
  std::optional<std::shared_ptr<mavsdk::System>> system_ = std::nullopt;
  std::shared_ptr<mavsdk::Action> action_;
  std::shared_ptr<mavsdk::Offboard> offboard_;
  std::shared_ptr<mavsdk::Telemetry> telemetry_;
  std::shared_ptr<mavsdk::ManualControl> manual_control_;
  std::thread tcp_server_thread_;
//  Autopilot_Interface autopilot_interface_;
};

