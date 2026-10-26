// Private stdin/stdout worker; it exposes no network control listener.
#include "slamware_adapter.h"
#include "velocity_session.h"
#include <poll.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

static void reply(unsigned long seq, bool ok, unsigned long id, const std::string& text,
                  const char* failure_status = "ERROR", const std::string& stats = "") {
    std::cout << "MOTUS_SDK " << seq << " " << (ok ? "OK" : failure_status) << " ";
    if (ok) std::cout << id << " " << text;
    else {
        const char* hex = "0123456789abcdef";
        for (unsigned char c : text) std::cout << hex[c >> 4] << hex[c & 15];
    }
    if (!stats.empty()) std::cout << " " << stats;
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
        using Session = VelocitySession<SlamwareAdapter>;
        Session session(sdk);
        // Connect is read-only. The capability applies only to this worker
        // lifetime; older workers without it require an explicit STOP first.
        reply(0, true, 0, "ready", "ERROR",
              "{\"capabilities\":{\"atomic_expiry_update\":true,\"staged_stop\":true}}");
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
                if (count <= 0) { session.stop(Session::StopReason::InputClosed); return 0; }
                buffer.append(chunk, count);
                if (buffer.size() > 4096) throw std::runtime_error("oversized SDK request");
                size_t newline;
                while ((newline = buffer.find('\n')) != std::string::npos) {
                    std::istringstream in(buffer.substr(0, newline));
                    buffer.erase(0, newline + 1);
                    std::string verb, extra;
                    if (!(in >> sequence >> verb)) throw std::runtime_error("malformed SDK request");
                    if (verb == "FOLLOW") {
                        long long motion_expiry, cancel_expiry;
                        double vx, wz;
                        if (!(in >> motion_expiry >> cancel_expiry >> vx >> wz) || (in >> extra) ||
                            !std::isfinite(vx) || !std::isfinite(wz) ||
                            std::abs(vx) > max_vx || std::abs(wz) > max_wz)
                            throw std::runtime_error("invalid/out-of-range FOLLOW velocity");
                        // Existing accepted leases run independently of packet
                        // validity, including when several requests share a read.
                        session.tick(Session::Clock::now());
                        try {
                            const auto wall = std::chrono::system_clock::now().time_since_epoch();
                            const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(wall).count();
                            const auto motion_ms = motion_expiry - now_ms;
                            const auto cancel_ms = cancel_expiry - now_ms;
                            if (motion_ms <= 0 || cancel_ms <= 0)
                                throw StaleVelocityCommand("stale FOLLOW deadline");
                            if (motion_ms > 300 || motion_ms > cancel_ms || cancel_ms > 1000)
                                throw std::runtime_error("invalid/excessive FOLLOW lease");
                            const auto now = Session::Clock::now();
                            session.follow(vx, wz, now + std::chrono::milliseconds(motion_ms),
                                           now + std::chrono::milliseconds(cancel_ms));
                        } catch (const StaleVelocityCommand& e) {
                            // This packet never renews or cancels a different
                            // fresh command. tick() owns the accepted deadlines.
                            auto stats = session.stats_json();
                            stats.pop_back();
                            stats += ",\"session_active\":" + std::string(session.active() ? "true" : "false") +
                                     ",\"action_id\":" + std::to_string(sdk.action_id()) + "}";
                            reply(sequence, false, 0, e.what(), "STALE", stats);
                            continue;
                        }
                    } else if (verb == "SOFT_STOP") {
                        if (in >> extra) throw std::runtime_error("malformed SOFT_STOP");
                        // No supplied deadline: this cannot prolong the session.
                        session.soft_stop();
                    } else if (verb == "SET" || verb == "UPDATE") {
                        long long expiry;
                        double vx, wz;
                        if (!(in >> expiry >> vx >> wz) || (in >> extra) ||
                            !std::isfinite(vx) || !std::isfinite(wz) ||
                            std::abs(vx) > max_vx || std::abs(wz) > max_wz)
                            throw std::runtime_error("invalid/out-of-range SDK velocity");
                        try {
                            auto wall = std::chrono::system_clock::now().time_since_epoch();
                            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(wall).count();
                            auto lease_ms = expiry - now_ms;
                            if (lease_ms <= 0)
                                throw CommandLeaseExpired("command lease expired at worker: remaining_ms=" +
                                                          std::to_string(lease_ms));
                            if (lease_ms > 300)
                                throw std::runtime_error("excessive command lease: remaining_ms=" +
                                                         std::to_string(lease_ms) + ", limit_ms=300");
                            auto deadline = VelocitySession<SlamwareAdapter>::Clock::now() +
                                            std::chrono::milliseconds(lease_ms);
                            if (vx == 0 && wz == 0) session.stop();
                            else if (verb == "UPDATE") session.update(vx, wz, deadline);
                            else session.set(vx, wz, deadline);
                        } catch (const CommandLeaseExpired& e) {
                            // Never extend/replay an expired command. A positive
                            // stop acknowledgement allows a later fresh command;
                            // a failed stop escapes to the fatal path below.
                            session.stop(Session::StopReason::LeaseExpired);
                            reply(sequence, false, 0, e.what(), "EXPIRED", session.stats_json());
                            continue;
                        }
                    } else if (verb == "STOP") {
                        if (in >> extra) throw std::runtime_error("malformed STOP");
                        session.stop();
                    } else throw std::runtime_error("unknown SDK request");
                    reply(sequence, true, sdk.action_id(), session.active() ? "active" : "ready",
                          "ERROR", session.stats_json());
                }
            }
        } catch (const std::exception& e) {
            try { session.stop(Session::StopReason::SdkException); } catch (...) {}
            reply(sequence, false, 0, e.what(), "ERROR", session.stats_json());
            return 1; // Never reconnect/replay a failed command.
        } catch (...) {
            try { session.stop(Session::StopReason::SdkException); } catch (...) {}
            reply(sequence, false, 0, "unknown SDK exception", "ERROR", session.stats_json());
            return 1; // Heartbeat stays armed if cancel is unconfirmed.
        }
    } catch (const std::exception& e) {
        reply(sequence, false, 0, e.what());
        return 1; // Never reconnect/replay a failed command.
    } catch (...) {
        reply(sequence, false, 0, "unknown SDK exception");
        return 1;
    }
}
