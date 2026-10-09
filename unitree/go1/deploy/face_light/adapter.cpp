// Go1 faceLightSDK v1.0.1 adapter. SDK owns destination and packet encoding.
// One process/client per card: exit reclaims SDK allocations even if its destructor
// (observed in the vendor library) only closes its socket.
#include "FaceLightClient.h"
#include <array>
#include <cerrno>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/socket.h>

namespace {
unsigned send_calls = 0;
ssize_t sent = -1;
size_t requested = 0;
int send_error = 0;
}

// The verified vendor SDK calls sendto through its PLT but returns void.
// Export this symbol so we can observe the actual syscall without accessing
// SDK private fields or reproducing its UDP address / packet format.
extern "C" ssize_t sendto(int fd, const void *buffer, size_t length, int flags,
                          const sockaddr *address, socklen_t address_length) {
    using Send = ssize_t (*)(int, const void *, size_t, int, const sockaddr *, socklen_t);
    static auto real_send = reinterpret_cast<Send>(dlsym(RTLD_NEXT, "sendto"));
    ++send_calls;
    requested = length;
    if (!real_send) {
        errno = ENOSYS;
        send_error = errno;
        return sent = -1;
    }
    // Nonblocking send bounds cancellation even if the UDP socket is congested.
    sent = real_send(fd, buffer, length, flags | MSG_DONTWAIT, address, address_length);
    send_error = sent < 0 ? errno : 0;
    return sent;
}

int main() {
    try {
        FaceLightClient client;
        std::cout << "READY face-light-v1\n" << std::flush;
        std::string line;
        while (std::getline(std::cin, line)) {
            // Line is a LOCAL IPC RGB frame, not a hardware datagram.
            if (line.size() > 512) {
                std::cout << "ERROR invalid frame length\n" << std::flush;
                continue;
            }
            std::istringstream input(line);
            std::array<uint8_t, 36> colors{};
            bool valid = true;
            for (auto &channel : colors) {
                int value;
                if (!(input >> value) || value < 0 || value > 255) {
                    valid = false;
                    break;
                }
                channel = static_cast<uint8_t>(value);
            }
            input >> std::ws;
            if (!valid || !input.eof()) {
                std::cout << "ERROR expected exactly 36 RGB integers in 0..255\n" << std::flush;
                continue;
            }
            for (uint32_t i = 0; i < 12; ++i) {
                client.setLedColor(i, colors.data() + 3 * i);
            }
            send_calls = 0;
            sent = -1;
            requested = 0;
            send_error = 0;
            client.sendCmd();
            if (send_calls != 1 || sent < 0 || static_cast<size_t>(sent) != requested) {
                std::cout << "ERROR SDK UDP send failed: "
                          << (send_error ? std::strerror(send_error) : "send interception unavailable or short send")
                          << '\n' << std::flush;
            } else {
                std::cout << "SENT\n" << std::flush;  // socket accepted, NOT LED feedback
            }
        }
        // EOF runs SDK destructor; the OS also reclaims process memory and fds.
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
