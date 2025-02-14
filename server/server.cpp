//
// Example that demonstrates offboard control using attitude, velocity control
// in NED (North-East-Down), and velocity control in body (Forward-Right-Down)
// coordinates.
//

#include <asm-generic/socket.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <mavsdk/component_type.h>
#include <thread>
#include <unistd.h>

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

void usage(const std::string &bin_name) {
  std::cerr
      << "Usage : " << bin_name
      << " <connection_url> [-s steering] [-t throttle]\n"
      << "Connection URL format should be :\n"
      << " For TCP : tcp://[server_host][:server_port]\n"
      << " For UDP : udp://[bind_host][:bind_port]\n"
      << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
      << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char **argv) {
  if (argc < 2) {
    usage(argv[0]);
    return 1;
  }
  BridgeConfig config = {};
  for (int i = 2; i < argc; i++) {
    if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
      config.steering_rate = std::stof(argv[++i]);
    }
    if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
      config.throttle_rate = std::stof(argv[++i]);
    }
  }
  MavlinkBridge bridge(config);
  bridge.run(argv[1]);
  return 0;
}

