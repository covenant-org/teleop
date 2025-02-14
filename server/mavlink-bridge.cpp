//
// Example that demonstrates offboard control using attitude, velocity control
// in NED (North-East-Down), and velocity control in body (Forward-Right-Down)
// coordinates.
//

#include <asm-generic/socket.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <future>
#include <iostream>
#include <kj/exception.h>
#include <mavsdk/component_type.h>
#include <memory>
#include <signal.h>
#include <thread>
#include <unistd.h>

#include "cereal/gen/cpp/custom.capnp.h"
#include "mavlink-bridge.hpp"
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/socket.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/manual_control/manual_control.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

MavlinkBridge::MavlinkBridge(const BridgeConfig config)
    : mavsdk_(Mavsdk::Configuration{ComponentType::GroundStation}),
      config_(config) {}

bool MavlinkBridge::connect(const std::string uri) {
  ConnectionResult connection_result = mavsdk_.add_any_connection(uri);
  if (connection_result != ConnectionResult::Success) {
    return false;
  }
  this->system_ = mavsdk_.first_autopilot(3.0);
  if (!this->system_) {
    return false;
  }
  this->action_ = std::make_shared<Action>(this->system_.value());
  this->offboard_ = std::make_shared<Offboard>(this->system_.value());
  this->telemetry_ = std::make_shared<Telemetry>(this->system_.value());
  this->manual_control_ = std::make_shared<ManualControl>(this->system_.value());
  // while (!this->telemetry_->health_all_ok()) {
  //   sleep_for(seconds(1));
  // }
  return true;
}

bool MavlinkBridge::bind_telemetry() {
  if (this->telemetry_ == nullptr) {
    return false;
  }
  this->telemetry_->subscribe_odometry(
      std::bind(&MavlinkBridge::on_odometry, this, std::placeholders::_1));
  this->telemetry_->subscribe_position_velocity_ned(
      std::bind(&MavlinkBridge::on_velocity, this, std::placeholders::_1));
  this->telemetry_->subscribe_position(
      std::bind(&MavlinkBridge::on_position, this, std::placeholders::_1));
  return true;
}

void MavlinkBridge::on_position(mavsdk::Telemetry::Position msg) {
  this->last_position = msg;
}

void MavlinkBridge::on_velocity(mavsdk::Telemetry::PositionVelocityNed msg) {
  this->last_position_velocity = msg;
}

void MavlinkBridge::on_odometry(mavsdk::Telemetry::Odometry msg) {
  this->last_odometry = msg;
}

void MavlinkBridge::run(const std::string uri) {
  std::cout << "Starting MavlinkBridge" << std::endl;
  std::cout << "Connecting to system through " << uri << std::endl;
  if (!this->connect(uri)) {
    throw std::runtime_error("Failed to connect to system");
  }
  if (!this->bind_telemetry()) {
    throw std::runtime_error("Failed to bind telemetry");
  }
  std::cout << "Starting tcp server" << std::endl;
  this->log_file_fd_ = open("/tmp/mavlink-bridge.csv",
                            O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
  if (this->log_file_fd_ < 0) {
    throw std::runtime_error("Failed to open log file");
  }
  std::string header =
      "time,gas,break,front_speed,steering,yaw_speed_deg,yaw_speed_deg\n";
  write(this->log_file_fd_, header.c_str(), header.size());
  this->run_tcp_server(this->config_.port);
}

void MavlinkBridge::run_tcp_server(const uint16_t port) {
  int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_socket < 0) {
    throw std::runtime_error("Failed to create TCP socket");
  }
  const int optval = 1;
  setsockopt(tcp_socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  sockaddr_in server_addr{.sin_family = AF_INET,
                          .sin_port = htons(port),
                          .sin_addr =
                              {
                                  .s_addr = INADDR_ANY,
                              },
                          .sin_zero = {0, 0, 0, 0, 0, 0, 0, 0}};
  if (bind(tcp_socket, reinterpret_cast<sockaddr *>(&server_addr),
           sizeof(server_addr)) < 0) {
    throw std::runtime_error("Failed to bind TCP socket");
  };
  if (listen(tcp_socket, 10) < 0) {
    throw std::runtime_error("Failed to listen on TCP socket");
  }
  std::cout << "Listening on port " << port << std::endl;
  signal(SIGPIPE, SIG_IGN);
  while (true) {
    sockaddr_in client_addr{};
    socklen_t client_addr_len = sizeof(client_addr);
    int client_socket_fd =
        accept(tcp_socket, reinterpret_cast<sockaddr *>(&client_addr),
               &client_addr_len);
    if (client_socket_fd < 0) {
      throw std::runtime_error("Failed to accept TCP connection");
    }
    uint8_t buf[1];
    auto bytes_read = recv(client_socket_fd, buf, sizeof(uint8_t), MSG_WAITALL);
    if (bytes_read <= 0) {
      continue;
    }
    std::thread client_thread([this, client_socket_fd, buf]() {
      try {
        if (buf[0] == 0x01) {
          this->send_odometry(client_socket_fd);
        } else if (buf[0] == 0x02) {
          this->apply_command(client_socket_fd);
        } else if (buf[0] == 0x03) {
          this->takeoff(client_socket_fd);
        } else if (buf[0] == 0x04) {
          this->land(client_socket_fd);
        }
      } catch (kj::Exception& e) {
        std::cout << "packed reader error: " << e.getDescription().cStr() << std::endl;
      }
      if (isFileDescriptiorValid(client_socket_fd)) {
        shutdown(client_socket_fd, SHUT_RDWR);
        close(client_socket_fd);
      }
    });
    client_thread.detach();
  }
}

float MavlinkBridge::get_yaw() {
  mavsdk::Telemetry::Quaternion q = this->last_odometry.q;
  return std::atan2(2.0 * (q.y * q.z + q.w * q.x),
                    q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

void MavlinkBridge::apply_command(int client_socket_fd) {
  if (!isFileDescriptiorValid(client_socket_fd)) {
    return;
  }
  ::capnp::PackedFdMessageReader message(client_socket_fd);
  cereal::CustomReserved0::Reader command =
      message.getRoot<cereal::CustomReserved0>();
  auto newRoll = command.getRoll();
  auto newPitch = command.getPitch();
  auto newYaw = command.getYaw();
  auto newThrust = command.getThrottle();

  std::cout << newRoll << std::endl;
  std::cout << newPitch << std::endl;
  std::cout << newYaw << std::endl;
  std::cout << newThrust << std::endl;

  auto result = this->manual_control_->set_manual_control_input(newPitch, newRoll, newThrust, newYaw);
  if (result != mavsdk::ManualControl::Result::Success) {
    std::cerr << "Failed to send manual control input" << std::endl;
    std::cerr << result << std::endl;
    throw std::runtime_error("Failed to send manual control input");
  }
}

void MavlinkBridge::takeoff(int client_socket_fd) {
  if (this->action_ == nullptr) {
    throw std::runtime_error("Action plugin not initialized");
  }
  if (this->last_position.relative_altitude_m <= 1) {
    std::cout << "Stopping offboard" << std::endl;
    this->offboard_->stop();
  }
  const auto arm_result = this->action_->arm();
  if (arm_result != mavsdk::Action::Result::Success) {
    throw std::runtime_error("Failed to arm");
  }
  const auto set_takeoff_altitude_result =
      this->action_->set_takeoff_altitude(3.0);
  if (set_takeoff_altitude_result != mavsdk::Action::Result::Success) {
    throw std::runtime_error("Failed to set takeoff altitude");
  }
  const auto takeoff_result = this->action_->takeoff();
  if (takeoff_result != mavsdk::Action::Result::Success) {
    throw std::runtime_error("Failed to takeoff");
  }
  std::promise in_air_promise = std::promise<void>{};
  auto in_air_future = in_air_promise.get_future();
  mavsdk::Telemetry::LandedStateHandle handle =
      this->telemetry_->subscribe_landed_state(
          [this, &in_air_promise,
           &handle](mavsdk::Telemetry::LandedState state) {
            if (state == mavsdk::Telemetry::LandedState::InAir) {
              this->telemetry_->unsubscribe_landed_state(handle);
              in_air_promise.set_value();
            }
          });
  in_air_future.wait_for(seconds(10));
  if (in_air_future.wait_for(seconds(10)) == std::future_status::timeout) {
    throw std::runtime_error("Timed out waiting for takeoff");
  }

  mavsdk::Offboard::VelocityBodyYawspeed stay{};
  this->offboard_->set_velocity_body(stay);

  mavsdk::Offboard::Result offboard_result = this->offboard_->start();
  if (offboard_result != mavsdk::Offboard::Result::Success) {
    throw std::runtime_error("Failed to start offboard");
  }
  char buff[1] = {0};
  if (isFileDescriptiorValid(client_socket_fd)) {
    write(client_socket_fd, buff, 1);
  }
}

void MavlinkBridge::land(int client_socket_fd) {
  if (this->action_ == nullptr) {
    throw std::runtime_error("Action plugin not initialized");
  }

  const auto land_result = this->action_->land();
  if (land_result != mavsdk::Action::Result::Success) {
    throw std::runtime_error("Failed to land");
  }

  char buff[1] = {0};
  if (isFileDescriptiorValid(client_socket_fd)) {
    write(client_socket_fd, buff, 1);
  }
}

void MavlinkBridge::send_odometry(int client_socket_fd) {
  ::capnp::MallocMessageBuilder message;
  cereal::GpsLocationData::Builder odom =
      message.initRoot<cereal::GpsLocationData>();
  odom.setLatitude(this->last_position.latitude_deg);
  odom.setLongitude(this->last_position.longitude_deg);
  odom.setAltitude(this->last_position.absolute_altitude_m);
  mavsdk::Telemetry::Quaternion q = this->last_odometry.q;
  float yaw = std::atan2(2.0 * (q.y * q.z + q.w * q.x),
                         q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
  odom.setBearingDeg(yaw * 180.0 / M_PI);
  kj::ArrayPtr<const float> velocities = {
      this->last_position_velocity.velocity.north_m_s,
      this->last_position_velocity.velocity.east_m_s,
      this->last_position_velocity.velocity.down_m_s};
  odom.setVNED(velocities);
  odom.setSpeed(this->last_odometry.angular_velocity_body.yaw_rad_s * 180.0 /
                M_PI * -1.0);
  if (isFileDescriptiorValid(client_socket_fd)) {
    writePackedMessageToFd(client_socket_fd, message);
  }
}
