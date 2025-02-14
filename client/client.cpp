//
// Example that demonstrates how to use manual control to fly a drone
// using a joystick or gamepad accessed using SDL2.
//
// Requires libsdl2 to be installed
// (for Ubuntu: sudo apt install libsdl2-dev).
//

#include <capnp/serialize.h>
#include <chrono>
#include <cstdio>
#include <future>
#include <netinet/in.h>
#include <netinet/ip.h>
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


using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

// This config works for Logitech Extreme 3D Pro
struct JoystickMapping {
    int roll_axis = 0;
    int pitch_axis = 1;
    int yaw_axis = 2;
    int throttle_axis = 3;

    bool roll_inverted = false;
    bool pitch_inverted = true;
    bool yaw_inverted = false;
    bool throttle_inverted = true;
} joystick_mapping{};

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 1) {
        // usage(argv[0]);
        return 1;
    }

    auto joystick = Joystick::create();
    if (!joystick) {
        std::cerr << "Could not find any joystick\n";
        return 1;
    }

    const uint32_t port = 4069;

    int client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_fd < 0) {
        throw std::runtime_error("Failed to create TCP socket");
    }

    sockaddr_in server_addr{.sin_family = AF_INET,
                          .sin_port = htons(port),
                          .sin_addr =
                              {
                                  .s_addr = INADDR_ANY,
                              }};
    int status = connect(client_fd, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr));
    if (status < 0) {
        throw std::runtime_error("Failed to bind TCP socket");
    }

    ::capnp::MallocMessageBuilder message;
    cereal::JoystickCommand::Builder joystickBuilder = message.initRoot<cereal::JoystickCommand>();

    bool has_started = false;

    while (true) {
        if (has_started) {
            { // axis
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

                uint8_t buf[1];

                joystickBuilder.setYaw(yaw);
                joystickBuilder.setPitch(pitch);
                joystickBuilder.setRoll(roll);
                joystickBuilder.setThrottle(throttle);
                writePackedMessageToFd(client_fd, message);
                recv(client_fd, buf, sizeof(uint8_t), MSG_WAITALL);
            }

            { // rb
                if (joystick->get_button(5)) {
                    uint8_t buf[] = {4};
                    send(client_fd, buf, sizeof(buf), 0);
                    uint8_t out[1];
                    recv(client_fd, out, sizeof(uint8_t), MSG_WAITALL);
                    if (out[0] == 0) {
                        has_started = false;
                    }
                }
            }

            sleep_for(milliseconds(20));
        } else {
            if (joystick->get_button(4)) { // lb
                uint8_t buf[] = {3};
                send(client_fd, buf, sizeof(buf), 0);
                uint8_t out[1];
                recv(client_fd, out, sizeof(uint8_t), MSG_WAITALL);
                if (out[0] == 0) {
                    has_started = true;
                }
            }
        }
    }

    return 0;
}
