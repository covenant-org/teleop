//
// Example that demonstrates how to use manual control to fly a drone
// using a joystick or gamepad accessed using SDL2.
//
// Requires libsdl2 to be installed
// (for Ubuntu: sudo apt install libsdl2-dev).
//

#include <capnp/common.h>
#include <capnp/serialize.h>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <future>
#include <kj/array.h>
#include <kj/common.h>
#include <kj/exception.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <unistd.h>
#include <memory>
#include <iostream>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/manual_control/manual_control.h>

#include "joystick.h"
#include "cereal/gen/cpp/custom.capnp.h"
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>


using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

bool isFileDescriptiorValid(int client_socket_fd) {
  return fcntl(client_socket_fd, F_GETFD) != -1 || errno != EBADF;
}

// This config works for Logitech Extreme 3D Pro
struct JoystickMapping {
    int roll_axis = 3;
    int pitch_axis = 4;
    int yaw_axis = 0;
    int throttle_axis = 1;

    bool roll_inverted = false;
    bool pitch_inverted = true;
    bool yaw_inverted = false;
    bool throttle_inverted = true;
} joystick_mapping{};

int get_socket_connection(const std::string &ip) {
    const uint32_t port = 4069;

    int client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_fd < 0) {
        throw std::runtime_error("Failed to create TCP socket");
    }

    sockaddr_in server_addr{.sin_family = AF_INET,
                          .sin_port = htons(port),
                          .sin_addr =
                              {
                                  .s_addr = inet_addr(ip.c_str()),
                              }};
    int status = connect(client_fd, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr));
    if (status < 0) {
        throw std::runtime_error("Failed to bind TCP socket");
    }

    return client_fd;
}

void send_op(int fd, uint8_t op) {
    printf("sending op: %d\n", op);
    uint8_t buf[1] = {op};
    write(fd, buf, sizeof(buf));
}

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <ip>\n"
              << "ip from server:\n"
              << "For example, to connect to the simulator use IP: 127.0.0.1\n";
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage("client");
        return 1;
    }

    std::string ip = argv[1];

    auto joystick = Joystick::create();
    if (!joystick) {
        std::cerr << "Could not find any joystick\n";
        return 1;
    }


    bool has_started = false;

    while (true) {
        if (has_started) {
            { // axis
                ::capnp::MallocMessageBuilder message;
                cereal::CustomReserved0::Builder jst = message.initRoot<cereal::CustomReserved0>();

                int fd = get_socket_connection(ip);
                const float roll = joystick->get_axis(joystick_mapping.roll_axis) *
                                   (joystick_mapping.roll_inverted ? -1.f : 1.f);
                const float pitch = joystick->get_axis(joystick_mapping.pitch_axis) *
                                    (joystick_mapping.pitch_inverted ? -1.f : 1.f);
                const float yaw = joystick->get_axis(joystick_mapping.yaw_axis) *
                                  (joystick_mapping.yaw_inverted ? -1.f : 1.f);
                float throttle = joystick->get_axis(joystick_mapping.throttle_axis) *
                                 (joystick_mapping.throttle_inverted ? -1.f : 1.f);
                // Scale -1 to 1 throttle range to 0 to 1
                throttle = throttle / 2.f + 0.5f;
                send_op(fd, 2);

                jst.setYaw(yaw);
                jst.setPitch(pitch);
                jst.setRoll(roll);
                jst.setThrottle(throttle);

                if (isFileDescriptiorValid(fd)) {
                    try {
                        capnp::writePackedMessageToFd(fd, message);
                    } catch (kj::Exception& e) {
                        std::cout << "Failed to write message" << e.getDescription().cStr() << std::endl;
                    }
                }

                uint8_t response[1];
                int bytes = read(fd, response, sizeof(response));
                if (bytes == -1) {
                    printf("error reading response");
                }
                close(fd);
            }

            { // rb
                if (joystick->get_button(5)) {
                    int fd = get_socket_connection(ip);
                    send_op(fd, 4);
                    uint8_t response[1];
                    int bytes = read(fd, response, sizeof(response));
                    if (bytes == -1) {
                        printf("error reading response");
                    }
                    close(fd);
                    has_started = false;
                }
            }

            sleep_for(milliseconds(20));
        } else {
            if (joystick->get_button(4)) { // lb
                int fd = get_socket_connection(ip);
                send_op(fd, 3);
                uint8_t response[1];
                int bytes = read(fd, response, sizeof(response));
                if (bytes == -1) {
                    printf("error reading response");
                }
                close(fd);
                has_started = true;
            }
        }
    }

    return 0;
}
