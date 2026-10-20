#include "velocity_session.h"
#include <cassert>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

struct FakeChassis {
    bool other = false, cancel_fails = false, begin_fails = false, preempted = false;
    int begin_delay_ms = 0, set_delay_ms = 0, heartbeat_starts = 0, refreshes = 0, heartbeat_stops = 0;
    int begins = 0, cancels = 0, writes = 0;
    double vx = 0, wz = 0;
    bool busy() { return other; }
    void start_heartbeat() { ++heartbeat_starts; }
    void refresh_heartbeat() { ++refreshes; }
    void stop_heartbeat() { ++heartbeat_stops; }
    void begin() {
        ++begins;
        if (begin_fails) throw std::runtime_error("ambiguous action creation");
        if (begin_delay_ms) std::this_thread::sleep_for(std::chrono::milliseconds(begin_delay_ms));
    }
    void verify_owned() { if (preempted) throw std::runtime_error("replaced"); }
    void set(double x, double yaw) {
        ++writes; vx = x; wz = yaw;
        if (set_delay_ms) std::this_thread::sleep_for(std::chrono::milliseconds(set_delay_ms));
    }
    void cancel() { ++cancels; if (cancel_fails || begin_fails) throw std::runtime_error("cancel unconfirmed"); }
};

int main(int argc, char** argv) {
    assert(argc == 2);
    std::string test(argv[1]);
    using Session = VelocitySession<FakeChassis>;
    using Clock = Session::Clock;
    FakeChassis chassis;
    Session session(chassis);
    auto deadline = Clock::now() + std::chrono::milliseconds(300);
    if (test == "idle") {
        session.stop(); session.tick(deadline);
        assert(chassis.writes == 0 && chassis.cancels == 0 && chassis.heartbeat_starts == 0);
    } else if (test == "combined") {
        session.set(0.05, -0.1, deadline);
        assert(chassis.vx == 0.05 && chassis.wz == -0.1 && chassis.begins == 1);
        session.set(-0.03, 0.2, deadline);
        assert(chassis.vx == -0.03 && chassis.wz == 0.2 && chassis.begins == 1);
        session.stop();
        assert(chassis.vx == 0 && chassis.wz == 0 && chassis.cancels == 1);
        assert(chassis.heartbeat_stops == 1 && !session.active());
        session.tick(deadline);
        assert(chassis.refreshes == 0);
    } else if (test == "watchdog") {
        session.set(0.05, 0, deadline);
        session.tick(Clock::now() + std::chrono::milliseconds(150));
        assert(chassis.refreshes == 1);
        session.tick(deadline);
        assert(!session.active() && chassis.vx == 0 && chassis.cancels == 1);
        assert(chassis.heartbeat_stops == 1);
        chassis.other = true; // A later home action must not see any more writes.
        session.tick(deadline + std::chrono::seconds(2)); session.stop();
        assert(chassis.cancels == 1 && chassis.writes == 2);
    } else if (test == "foreign") {
        chassis.other = true;
        bool threw = false;
        try { session.set(0.05, 0, deadline); } catch (...) { threw = true; }
        assert(threw && chassis.heartbeat_starts == 0 && chassis.begins == 0 && chassis.writes == 0);
    } else if (test == "expired") {
        bool threw = false;
        try { session.set(0.05, 0, Clock::now() - std::chrono::milliseconds(1)); } catch (...) { threw = true; }
        assert(threw && chassis.heartbeat_starts == 0 && chassis.writes == 0);
    } else if (test == "slow_begin") {
        chassis.begin_delay_ms = 30;
        bool threw = false;
        try { session.set(0.05, 0, Clock::now() + std::chrono::milliseconds(10)); } catch (...) { threw = true; }
        assert(threw && chassis.vx == 0 && chassis.writes == 1 && chassis.cancels == 1);
        assert(chassis.heartbeat_stops == 1);
    } else if (test == "slow_set") {
        chassis.set_delay_ms = 30;
        bool threw = false;
        try { session.set(0.05, 0, Clock::now() + std::chrono::milliseconds(10)); } catch (...) { threw = true; }
        assert(threw && chassis.vx == 0 && chassis.cancels == 1 && chassis.heartbeat_stops == 1);
    } else if (test == "cancel_failure") {
        session.set(0.05, 0, deadline);
        chassis.cancel_fails = true;
        bool threw = false;
        try { session.stop(); } catch (...) { threw = true; }
        assert(threw && chassis.heartbeat_stops == 0 && session.active());
    } else if (test == "begin_failure") {
        chassis.begin_fails = true;
        bool threw = false;
        try { session.set(0.05, 0, deadline); } catch (...) { threw = true; }
        assert(threw);
        try { session.stop(); } catch (...) {}
        assert(chassis.heartbeat_stops == 0); // keep firmware protection armed
    } else if (test == "preempted") {
        session.set(0.05, 0, deadline);
        chassis.preempted = true;
        bool threw = false;
        try { session.set(0.2, 0, deadline); } catch (...) { threw = true; }
        assert(threw && chassis.begins == 1 && chassis.writes == 1);
    } else return 2;
}
