// Native software test double using the actual vendor header. Never uses IP.
#include "FaceLightClient.h"
#include <array>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <sys/socket.h>
namespace {
std::array<uint8_t, 36> frame{};
int pair_fds[2];
}
FaceLightClient::FaceLightClient() {
    if (socketpair(AF_UNIX, SOCK_DGRAM, 0, pair_fds)) throw std::runtime_error("socketpair failed");
}
FaceLightClient::~FaceLightClient() {close(pair_fds[0]); close(pair_fds[1]);}
void FaceLightClient::setLedColor(uint32_t id, const uint8_t *rgb) {
    if (id >= 12) throw std::runtime_error("bad LED index");
    for (unsigned c = 0; c < 3; ++c) frame[id * 3 + c] = rgb[c];
}
void FaceLightClient::sendCmd() {
    if (const char *log = std::getenv("FACE_TEST_LOG")) {
        std::ofstream output(log, std::ios::app);
        for (auto value : frame) output << static_cast<unsigned>(value) << ' ';
        output << '\n';
    }
    if (std::getenv("FACE_TEST_ERROR")) {
        sendto(-1, frame.data(), frame.size(), 0, nullptr, 0);
    } else {
        sendto(pair_fds[0], frame.data(), frame.size(), 0, nullptr, 0);
        uint8_t received[36];
        recv(pair_fds[1], received, sizeof(received), 0);
    }
}
