#include "velocity_session.h"
#include <cassert>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

struct FakeChassis {
    bool other = false, cancel_fails = false, begin_fails = false, preempted = false;
    int begin_delay_ms = 0, set_delay_ms = 0, cancel_delay_ms = 0;
    int heartbeat_starts = 0, refreshes = 0, heartbeat_stops = 0;
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
    void cancel() {
        ++cancels;
        if (cancel_delay_ms) std::this_thread::sleep_for(std::chrono::milliseconds(cancel_delay_ms));
        if (cancel_fails || begin_fails) throw std::runtime_error("cancel unconfirmed");
    }
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
        assert(session.stats_json().find("\"stop_attempts\":0") != std::string::npos);
    } else if (test == "combined") {
        session.set(0.05, -0.1, deadline);
        assert(chassis.vx == 0.05 && chassis.wz == -0.1 && chassis.begins == 1);
        session.set(-0.03, 0.2, deadline);
        assert(chassis.vx == -0.03 && chassis.wz == 0.2 && chassis.begins == 1);
        auto active_stats = session.stats_json();
        assert(active_stats.find("\"action_starts\":1") != std::string::npos);
        assert(active_stats.find("\"set_attempts\":2") != std::string::npos);
        assert(active_stats.find("\"busy_ms\":null") != std::string::npos);
        assert(chassis.cancels == 0 && chassis.heartbeat_starts == 1);
        session.stop();
        assert(chassis.vx == 0 && chassis.wz == 0 && chassis.cancels == 1);
        assert(chassis.heartbeat_stops == 1 && !session.active());
        assert(session.stats_json().find("\"confirmed_stops\":1") != std::string::npos);
        session.tick(deadline);
        assert(chassis.refreshes == 0);
    } else if (test == "watchdog") {
        session.set(0.05, 0, deadline);
        session.tick(Clock::now() + std::chrono::milliseconds(150));
        assert(chassis.refreshes == 1);
        session.tick(deadline);
        assert(!session.active() && chassis.vx == 0 && chassis.cancels == 1);
        assert(chassis.heartbeat_stops == 1);
        assert(session.stats_json().find("\"heartbeat_refreshes\":1") != std::string::npos);
        assert(session.stats_json().find("\"stop_reason\":\"lease_expired\"") != std::string::npos);
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
        assert(session.stats_json().find("\"stop_attempts\":1") != std::string::npos);
        assert(session.stats_json().find("\"confirmed_stops\":0") != std::string::npos);
    } else if (test == "begin_failure") {
        chassis.begin_fails = true;
        bool threw = false;
        try { session.set(0.05, 0, deadline); } catch (...) { threw = true; }
        assert(threw);
        assert(session.stats_json().find("\"action_start_attempts\":1") != std::string::npos);
        assert(session.stats_json().find("\"action_starts\":0") != std::string::npos);
        try { session.stop(); } catch (...) {}
        assert(chassis.heartbeat_stops == 0); // keep firmware protection armed
    } else if (test == "preempted") {
        session.set(0.05, 0, deadline);
        chassis.preempted = true;
        bool threw = false;
        try { session.set(0.2, 0, deadline); } catch (...) { threw = true; }
        assert(threw && chassis.begins == 1 && chassis.writes == 1);
    } else if (test == "atomic_fresh_update") {
        session.set(0.05, 0, deadline);
        session.update(0.12, -0.2, deadline);
        assert(session.active() && chassis.vx == 0.12 && chassis.wz == -0.2);
        assert(chassis.begins == 1 && chassis.cancels == 0 && chassis.writes == 2);
        assert(chassis.heartbeat_starts == 1 && chassis.heartbeat_stops == 0);
        session.stop();
    } else if (test == "atomic_expiry_update") {
        auto old_deadline = Clock::now() + std::chrono::milliseconds(30);
        session.set(0.05, 0, old_deadline);
        std::this_thread::sleep_until(old_deadline + std::chrono::milliseconds(1));
        // No tick/STOP in between: UPDATE must stop before replacing the
        // old deadline, then create a fresh action with the new velocity.
        session.update(0.12, -0.2, Clock::now() + std::chrono::milliseconds(250));
        assert(session.active() && chassis.vx == 0.12 && chassis.wz == -0.2);
        assert(chassis.begins == 2 && chassis.cancels == 1 && chassis.writes == 3);
        assert(chassis.heartbeat_starts == 2 && chassis.heartbeat_stops == 1);
        assert(session.stats_json().find("\"confirmed_stops\":1") != std::string::npos);
        assert(session.stats_json().find("\"stop_reason\":\"lease_expired\"") != std::string::npos);
        session.stop();
    } else if (test == "atomic_stop_exhausts_new_lease") {
        auto old_deadline = Clock::now() + std::chrono::milliseconds(30);
        session.set(0.05, 0, old_deadline);
        std::this_thread::sleep_until(old_deadline + std::chrono::milliseconds(1));
        chassis.cancel_delay_ms = 60;
        bool expired = false;
        try { session.update(0.12, 0, Clock::now() + std::chrono::milliseconds(20)); }
        catch (const CommandLeaseExpired&) { expired = true; }
        assert(expired && !session.active() && chassis.vx == 0);
        assert(chassis.begins == 1 && chassis.cancels == 1 && chassis.writes == 2);
        assert(chassis.heartbeat_starts == 1 && chassis.heartbeat_stops == 1);
    } else if (test == "atomic_stop_failure") {
        auto old_deadline = Clock::now() + std::chrono::milliseconds(30);
        session.set(0.05, 0, old_deadline);
        std::this_thread::sleep_until(old_deadline + std::chrono::milliseconds(1));
        chassis.cancel_fails = true;
        bool failed = false, expired = false;
        try { session.update(0.12, 0, Clock::now() + std::chrono::milliseconds(250)); }
        catch (const CommandLeaseExpired&) { expired = true; }
        catch (const std::runtime_error&) { failed = true; }
        assert(failed && !expired && session.active());
        assert(chassis.begins == 1 && chassis.cancels == 1 && chassis.writes == 2);
        assert(chassis.vx == 0 && chassis.heartbeat_stops == 0);
        assert(session.stats_json().find("\"confirmed_stops\":0") != std::string::npos);
    } else if (test == "zero_crossing") {
        session.set(0, 0.2, deadline);
        session.zero_velocity(deadline);
        assert(session.active() && chassis.vx == 0 && chassis.wz == 0);
        assert(chassis.begins == 1 && chassis.cancels == 0 && chassis.writes == 2);
        session.set(0, -0.2, deadline);
        assert(chassis.begins == 1 && chassis.wz == -0.2);
        session.tick(deadline);
        assert(!session.active() && chassis.cancels == 1 && chassis.heartbeat_stops == 1);
    } else if (test == "zero_idle") {
        session.zero_velocity(deadline);
        assert(!session.active() && chassis.begins == 0 && chassis.writes == 0);
        assert(chassis.heartbeat_starts == 0);
    } else if (test == "zero_expired_session") {
        auto old_deadline = Clock::now() + std::chrono::milliseconds(20);
        session.set(0, 0.2, old_deadline);
        std::this_thread::sleep_until(old_deadline + std::chrono::milliseconds(1));
        session.zero_velocity(Clock::now() + std::chrono::milliseconds(200));
        assert(!session.active() && chassis.begins == 1 && chassis.cancels == 1);
        assert(chassis.vx == 0 && chassis.wz == 0 && chassis.heartbeat_stops == 1);
    } else if (test == "zero_preempted") {
        session.set(0, 0.2, deadline);
        chassis.preempted = true;
        bool threw = false;
        try { session.zero_velocity(deadline); }
        catch (const std::runtime_error&) { threw = true; }
        assert(threw && chassis.writes == 1 && chassis.begins == 1);
        session.stop(Session::StopReason::SdkException);
        assert(!session.active() && chassis.cancels == 1);
    } else if (test == "zero_slow_set") {
        session.set(0, 0.2, deadline);
        chassis.set_delay_ms = 30;
        bool expired = false;
        try { session.zero_velocity(Clock::now() + std::chrono::milliseconds(10)); }
        catch (const CommandLeaseExpired&) { expired = true; }
        assert(expired && !session.active() && chassis.cancels == 1);
        assert(chassis.begins == 1 && chassis.heartbeat_stops == 1);
    } else if (test == "zero_expired_stop_failure") {
        auto old_deadline = Clock::now() + std::chrono::milliseconds(20);
        session.set(0, 0.2, old_deadline);
        std::this_thread::sleep_until(old_deadline + std::chrono::milliseconds(1));
        chassis.cancel_fails = true;
        bool threw = false;
        try { session.zero_velocity(Clock::now() + std::chrono::milliseconds(200)); }
        catch (const std::runtime_error&) { threw = true; }
        assert(threw && chassis.begins == 1 && chassis.heartbeat_stops == 0);
    } else if (test == "follow_soft_resume") {
        const auto motion = Clock::now() + std::chrono::milliseconds(20);
        const auto cancel = Clock::now() + std::chrono::milliseconds(250);
        session.follow(0.1, -0.2, motion, cancel);
        std::this_thread::sleep_until(motion + std::chrono::milliseconds(1));
        session.tick(Clock::now());
        assert(session.active() && chassis.vx == 0 && chassis.wz == 0);
        assert(chassis.cancels == 0 && chassis.begins == 1 && chassis.heartbeat_stops == 0);
        assert(session.stats_json().find("\"soft_stops\":1") != std::string::npos);
        session.follow(0.12, 0.3, Clock::now() + std::chrono::milliseconds(100),
                       Clock::now() + std::chrono::milliseconds(200));
        assert(session.active() && chassis.vx == 0.12 && chassis.wz == 0.3);
        assert(chassis.begins == 1 && chassis.cancels == 0);
        session.stop();
    } else if (test == "follow_hard_timeout") {
        const auto motion = Clock::now() + std::chrono::milliseconds(20);
        const auto cancel = Clock::now() + std::chrono::milliseconds(140);
        session.follow(0.1, 0.2, motion, cancel);
        std::this_thread::sleep_until(motion + std::chrono::milliseconds(1));
        session.tick(Clock::now());
        session.soft_stop(); session.soft_stop();
        assert(chassis.cancels == 0 && chassis.writes == 2);
        std::this_thread::sleep_until(cancel - std::chrono::milliseconds(20));
        session.tick(Clock::now());
        assert(chassis.refreshes == 1 && session.active());
        std::this_thread::sleep_until(cancel + std::chrono::milliseconds(1));
        session.tick(Clock::now());
        assert(!session.active() && chassis.cancels == 1 && chassis.heartbeat_stops == 1);
    } else if (test == "follow_stale_ignored") {
        const auto motion = Clock::now() + std::chrono::milliseconds(100);
        const auto cancel = Clock::now() + std::chrono::milliseconds(180);
        session.follow(0.1, -0.2, motion, cancel);
        bool stale = false;
        try { session.follow(0.2, 0.4, Clock::now() - std::chrono::milliseconds(1),
                             Clock::now() + std::chrono::milliseconds(1000)); }
        catch (const StaleVelocityCommand&) { stale = true; }
        assert(stale && session.active() && chassis.vx == 0.1 && chassis.wz == -0.2);
        assert(chassis.cancels == 0 && chassis.begins == 1 && chassis.writes == 1);
        std::this_thread::sleep_until(cancel + std::chrono::milliseconds(1));
        session.tick(Clock::now());
        assert(!session.active() && chassis.cancels == 1);
    } else if (test == "follow_stale_after_soft_stop") {
        const auto motion = Clock::now() + std::chrono::milliseconds(20);
        const auto cancel = Clock::now() + std::chrono::milliseconds(80);
        session.follow(0.1, 0.2, motion, cancel);
        std::this_thread::sleep_until(motion + std::chrono::milliseconds(1));
        session.soft_stop();
        for (int packet = 0; packet < 3; ++packet) {
            bool stale = false;
            try { session.follow(0.2, 0.4, Clock::now() - std::chrono::milliseconds(1),
                                 Clock::now() + std::chrono::milliseconds(1000)); }
            catch (const StaleVelocityCommand&) { stale = true; }
            assert(stale && session.active() && chassis.writes == 2 && chassis.cancels == 0);
        }
        std::this_thread::sleep_until(cancel + std::chrono::milliseconds(1));
        session.tick(Clock::now());
        assert(!session.active() && chassis.cancels == 1);
    } else if (test == "follow_slow_write") {
        chassis.set_delay_ms = 30;
        bool stale = false;
        try { session.follow(0.1, 0.2, Clock::now() + std::chrono::milliseconds(10),
                             Clock::now() + std::chrono::milliseconds(200)); }
        catch (const StaleVelocityCommand&) { stale = true; }
        assert(stale && session.active() && chassis.vx == 0 && chassis.wz == 0);
        assert(chassis.cancels == 0 && chassis.begins == 1 && chassis.heartbeat_stops == 0);
        chassis.set_delay_ms = 0;
        session.follow(0.12, -0.3, Clock::now() + std::chrono::milliseconds(100),
                       Clock::now() + std::chrono::milliseconds(200));
        assert(chassis.begins == 1 && chassis.vx == 0.12 && chassis.wz == -0.3);
        session.stop();
    } else if (test == "follow_soft_ownership") {
        session.follow(0.1, 0.2, Clock::now() + std::chrono::milliseconds(100),
                       Clock::now() + std::chrono::milliseconds(200));
        chassis.preempted = true;
        bool failed = false;
        try { session.soft_stop(); } catch (const std::runtime_error&) { failed = true; }
        assert(failed && chassis.writes == 1 && chassis.begins == 1);
    } else if (test == "follow_slow_write_hard_deadline") {
        chassis.set_delay_ms = 30;
        const auto original_cancel = Clock::now() + std::chrono::milliseconds(100);
        bool stale = false;
        try { session.follow(0.1, 0.2, Clock::now() + std::chrono::milliseconds(10), original_cancel); }
        catch (const StaleVelocityCommand&) { stale = true; }
        assert(stale && session.active() && chassis.vx == 0 && chassis.cancels == 0);
        assert(chassis.begins == 1 && chassis.heartbeat_stops == 0);
        chassis.set_delay_ms = 0;
        session.soft_stop(); // An ACK cannot start another cancellation budget.
        std::this_thread::sleep_until(original_cancel + std::chrono::milliseconds(1));
        session.tick(Clock::now());
        assert(!session.active() && chassis.cancels == 1 && chassis.heartbeat_stops == 1);
        assert(chassis.begins == 1);
    } else if (test == "follow_zero_idle") {
        session.follow(0, 0, Clock::now() + std::chrono::milliseconds(100),
                       Clock::now() + std::chrono::milliseconds(200));
        session.soft_stop();
        assert(!session.active() && chassis.writes == 0 && chassis.begins == 0);
        assert(chassis.heartbeat_starts == 0);
    } else if (test == "follow_explicit_stop") {
        session.follow(0.1, 0.2, Clock::now() + std::chrono::milliseconds(100),
                       Clock::now() + std::chrono::milliseconds(200));
        session.stop();
        assert(!session.active() && chassis.cancels == 1 && chassis.heartbeat_stops == 1);
        session.tick(Clock::now() + std::chrono::seconds(1));
        assert(chassis.writes == 2 && chassis.begins == 1);
    } else return 2;
}
