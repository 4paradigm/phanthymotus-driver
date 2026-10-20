// Private stdin/stdout worker; it exposes no network control listener.
#include "slamware_adapter.h"
#include "velocity_session.h"
#include <poll.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

static void reply(unsigned long seq, bool ok, unsigned long id, const std::string& text) {
    std::cout << "MOTUS_SDK " << seq << (ok ? " OK " : " ERROR ");
    if (ok) std::cout << id << " " << text;
    else {
        const char* hex = "0123456789abcdef";
        for (unsigned char c : text) std::cout << hex[c >> 4] << hex[c & 15];
    }
    std::cout << std::endl;
}

int main(int argc, char** argv) {
    unsigned long sequence = 0;
    try {
        if (argc != 6) throw std::runtime_error("expected host port connect_ms max_vx max_wz");
        const double max_vx = std::stod(argv[4]), max_wz = std::stod(argv[5]);
        if (!std::isfinite(max_vx) || !std::isfinite(max_wz) || max_vx <= 0 || max_wz <= 0)
            throw std::runtime_error("invalid velocity limits");
        SlamwareAdapter sdk(argv[1], std::stoi(argv[2]), std::stoi(argv[3]));
        VelocitySession<SlamwareAdapter> session(sdk);
        reply(0, true, 0, "ready"); // Connect is read-only, no velocity action.
        std::string buffer;
        try {
            while (true) {
                session.tick(VelocitySession<SlamwareAdapter>::Clock::now());
                pollfd fd{STDIN_FILENO, POLLIN, 0};
                const int result = poll(&fd, 1, 20);
                if (result < 0) throw std::runtime_error("stdin poll failed");
                if (!result) continue;
                char chunk[1024];
                auto count = read(STDIN_FILENO, chunk, sizeof(chunk));
                if (count <= 0) { session.stop(); return 0; }
                buffer.append(chunk, count);
                if (buffer.size() > 4096) throw std::runtime_error("oversized SDK request");
                size_t newline;
                while ((newline = buffer.find('\n')) != std::string::npos) {
                    std::istringstream in(buffer.substr(0, newline));
                    buffer.erase(0, newline + 1);
                    std::string verb, extra;
                    if (!(in >> sequence >> verb)) throw std::runtime_error("malformed SDK request");
                    if (verb == "SET") {
                        long long expiry;
                        double vx, wz;
                        if (!(in >> expiry >> vx >> wz) || (in >> extra) ||
                            !std::isfinite(vx) || !std::isfinite(wz) ||
                            std::abs(vx) > max_vx || std::abs(wz) > max_wz)
                            throw std::runtime_error("invalid/out-of-range SDK velocity");
                        auto wall = std::chrono::system_clock::now().time_since_epoch();
                        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(wall).count();
                        auto lease_ms = expiry - now_ms;
                        if (lease_ms <= 0 || lease_ms > 300)
                            throw std::runtime_error("expired or excessive command lease");
                        auto deadline = VelocitySession<SlamwareAdapter>::Clock::now() +
                                        std::chrono::milliseconds(lease_ms);
                        if (vx == 0 && wz == 0) session.stop();
                        else session.set(vx, wz, deadline);
                    } else if (verb == "STOP") {
                        if (in >> extra) throw std::runtime_error("malformed STOP");
                        session.stop();
                    } else throw std::runtime_error("unknown SDK request");
                    reply(sequence, true, sdk.action_id(), session.active() ? "active" : "ready");
                }
            }
        } catch (...) {
            try { session.stop(); } catch (...) {} // heartbeat stays armed if cancel is unconfirmed
            throw;
        }
    } catch (const std::exception& e) {
        reply(sequence, false, 0, e.what());
        return 1; // Never reconnect/replay a failed command.
    } catch (...) {
        reply(sequence, false, 0, "unknown SDK exception");
        return 1;
    }
}
