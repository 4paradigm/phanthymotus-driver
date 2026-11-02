#pragma once

#include <chrono>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

// Expiry is recoverable only after stop/cancel has acknowledged. Transport,
// ownership and cancellation errors must still terminate the worker.
struct CommandLeaseExpired : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// FOLLOW rejects stale packets without cancelling or renewing a different,
// already accepted command. The session's own two deadlines remain in force.
struct StaleVelocityCommand : CommandLeaseExpired {
    using CommandLeaseExpired::CommandLeaseExpired;
};

// Independent of ROS and the SDK, so the actual lease/ownership implementation
// can be tested with a fake chassis. The firmware heartbeat is the backup if a
// vendor call blocks or this worker/Python dies; its resolution is seconds.
template<class Adapter> class VelocitySession {
public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    enum class StopReason { None, ExplicitStop, LeaseExpired, InputClosed, SdkException };
private:
    Adapter& sdk_;
    bool heartbeat_ = false;
    bool motion_possible_ = false;
    bool staged_ = false, soft_stopped_ = false;
    Time deadline_, cancel_deadline_, refreshed_;
    struct SetTiming {
        // Negative durations mean the stage was not attempted. Lease values
        // have separate flags because a negative remaining lease is meaningful.
        double busy_ms = -1, start_heartbeat_ms = -1, begin_ms = -1;
        double verify_owned_ms = -1, set_velocity_ms = -1;
        double remaining_before_set_ms = 0, remaining_after_set_ms = 0;
        bool before_set = false, after_set = false;
    } last_set_;
    unsigned long set_attempts_ = 0, action_start_attempts_ = 0, action_starts_ = 0;
    unsigned long stop_attempts_ = 0, confirmed_stops_ = 0, heartbeat_refreshes_ = 0;
    unsigned long soft_stop_attempts_ = 0, soft_stops_ = 0;
    double last_heartbeat_ms_ = -1, max_heartbeat_ms_ = -1;
    StopReason stop_reason_ = StopReason::None;

    static double elapsed_ms(Time from) {
        return std::chrono::duration<double, std::milli>(Clock::now() - from).count();
    }
    template<class F> static void measured(double& duration, F operation) {
        const auto start = Clock::now();
        try { operation(); }
        catch (...) { duration = elapsed_ms(start); throw; }
        duration = elapsed_ms(start);
    }
    double remaining_ms() const {
        return std::chrono::duration<double, std::milli>(deadline_ - Clock::now()).count();
    }
    static const char* reason_name(StopReason reason) {
        switch (reason) {
        case StopReason::ExplicitStop: return "explicit_stop";
        case StopReason::LeaseExpired: return "lease_expired";
        case StopReason::InputClosed: return "input_closed";
        case StopReason::SdkException: return "sdk_exception";
        default: return "none";
        }
    }
    static void json_number(std::ostringstream& out, double value, bool present = true) {
        if (present) out << value;
        else out << "null";
    }
    void check_write_deadline(const char* message) {
        if (Clock::now() < deadline_) return;
        if (staged_) {
            // A fresh FOLLOW can spend its motion lease inside the SDK. Stop
            // velocity but retain only the already bounded cancellation lease.
            if (motion_possible_) soft_stop();
            else stop(StopReason::LeaseExpired);
            throw StaleVelocityCommand(message);
        }
        stop(StopReason::LeaseExpired);
        throw CommandLeaseExpired(message);
    }
    void apply(double vx, double wz, Time deadline, Time cancel_deadline, bool staged) {
        ++set_attempts_;
        last_set_ = SetTiming();
        deadline_ = deadline;
        cancel_deadline_ = cancel_deadline;
        staged_ = staged;
        soft_stopped_ = false;
        if (!motion_possible_) {
            bool busy = false;
            measured(last_set_.busy_ms, [&] { busy = sdk_.busy(); });
            if (busy) throw std::runtime_error("another chassis action is active");
            // No motion until firmware dead-client protection acknowledges.
            measured(last_set_.start_heartbeat_ms, [&] { sdk_.start_heartbeat(); });
            heartbeat_ = true;
            refreshed_ = Clock::now();
            check_write_deadline("lease expired before action creation");
            motion_possible_ = true; // includes ambiguous begin() failures
            ++action_start_attempts_;
            measured(last_set_.begin_ms, [&] { sdk_.begin(); });
            ++action_starts_;
        }
        measured(last_set_.verify_owned_ms, [&] { sdk_.verify_owned(); });
        last_set_.before_set = true;
        last_set_.remaining_before_set_ms = remaining_ms();
        check_write_deadline("lease expired before velocity write");
        try { measured(last_set_.set_velocity_ms, [&] { sdk_.set(vx, wz); }); }
        catch (...) {
            last_set_.after_set = true;
            last_set_.remaining_after_set_ms = remaining_ms();
            throw;
        }
        last_set_.after_set = true;
        last_set_.remaining_after_set_ms = remaining_ms();
        check_write_deadline("lease expired during SDK write");
    }
public:
    explicit VelocitySession(Adapter& sdk) : sdk_(sdk) {}
    bool active() const { return motion_possible_; }
    void stop(StopReason reason = StopReason::ExplicitStop) {
        const bool attempted = motion_possible_ || heartbeat_;
        if (attempted) { ++stop_attempts_; stop_reason_ = reason; }
        if (motion_possible_) {
            // A zero failure does not prevent an attempt to cancel our handle.
            try { sdk_.set(0, 0); } catch (...) {}
            sdk_.cancel();  // If this fails, leave the firmware heartbeat armed.
            motion_possible_ = false;
        }
        if (heartbeat_) {
            sdk_.stop_heartbeat();
            heartbeat_ = false;
        }
        if (attempted) ++confirmed_stops_;
    }
    void set(double vx, double wz, Time deadline) {
        if (deadline <= Clock::now()) throw CommandLeaseExpired("command lease expired");
        apply(vx, wz, deadline, deadline, false);
    }
    void update(double vx, double wz, Time deadline) {
        // Only an explicit UPDATE requests atomic expiry recovery. Ordinary
        // SET retains its protocol/behaviour for older Python controllers.
        if (deadline <= Clock::now()) throw CommandLeaseExpired("command lease expired");
        if (motion_possible_ && Clock::now() >= deadline_) {
            stop(StopReason::LeaseExpired);
            if (deadline <= Clock::now())
                throw CommandLeaseExpired("command lease expired during previous action stop");
        }
        set(vx, wz, deadline);
    }
    void follow(double vx, double wz, Time motion_deadline, Time cancel_deadline) {
        // Process accepted leases first; a stale packet cannot keep an old
        // action alive, cancel a still fresh one, or refresh its heartbeat.
        tick(Clock::now());
        if (motion_deadline <= Clock::now() || cancel_deadline <= Clock::now())
            throw StaleVelocityCommand("stale FOLLOW deadline");
        if (motion_deadline > cancel_deadline)
            throw std::runtime_error("motion deadline exceeds cancellation deadline");
        if (vx == 0 && wz == 0 && !motion_possible_) return;
        const bool had_staged_action = motion_possible_ && staged_;
        const auto previous_cancel_deadline = cancel_deadline_;
        try { apply(vx, wz, motion_deadline, cancel_deadline, true); }
        catch (const StaleVelocityCommand&) {
            // A candidate that expires inside the SDK confirmed zero, but did
            // not become a new valid frame. It must not prolong an old action.
            if (had_staged_action && motion_possible_) {
                if (previous_cancel_deadline < cancel_deadline_)
                    cancel_deadline_ = previous_cancel_deadline;
                if (Clock::now() >= cancel_deadline_)
                    stop(StopReason::LeaseExpired);
            }
            throw;
        }
    }
    void soft_stop() {
        if (!motion_possible_) return;
        if (!staged_) { stop(); return; } // legacy sessions retain hard expiry
        if (Clock::now() >= cancel_deadline_) { stop(StopReason::LeaseExpired); return; }
        if (soft_stopped_) return;
        ++soft_stop_attempts_;
        sdk_.verify_owned();
        sdk_.set(0, 0); // failure is fatal; never claim a stop without its ACK
        soft_stopped_ = true;
        ++soft_stops_;
        if (Clock::now() >= cancel_deadline_) stop(StopReason::LeaseExpired);
    }
    void tick(Time now) {
        if (!motion_possible_) return;
        if (now >= cancel_deadline_) { stop(StopReason::LeaseExpired); return; }
        if (now >= deadline_) {
            if (!staged_) { stop(StopReason::LeaseExpired); return; }
            soft_stop();
            if (!motion_possible_) return;
        }
        if (now - refreshed_ >= std::chrono::milliseconds(100)) {
            try { measured(last_heartbeat_ms_, [&] { sdk_.refresh_heartbeat(); }); }
            catch (...) {
                if (last_heartbeat_ms_ > max_heartbeat_ms_) max_heartbeat_ms_ = last_heartbeat_ms_;
                throw;
            }
            if (last_heartbeat_ms_ > max_heartbeat_ms_) max_heartbeat_ms_ = last_heartbeat_ms_;
            ++heartbeat_refreshes_;
            refreshed_ = Clock::now();
        }
    }
    std::string stats_json() const {
        // Fixed names/enumerations and finite steady-clock durations only; no
        // SDK strings enter JSON. Compact output is one extra protocol field.
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out << std::fixed << std::setprecision(3) << "{\"last_set\":{";
        out << "\"busy_ms\":"; json_number(out, last_set_.busy_ms, last_set_.busy_ms >= 0);
        out << ",\"start_heartbeat_ms\":"; json_number(out, last_set_.start_heartbeat_ms, last_set_.start_heartbeat_ms >= 0);
        out << ",\"begin_ms\":"; json_number(out, last_set_.begin_ms, last_set_.begin_ms >= 0);
        out << ",\"verify_owned_ms\":"; json_number(out, last_set_.verify_owned_ms, last_set_.verify_owned_ms >= 0);
        out << ",\"set_velocity_ms\":"; json_number(out, last_set_.set_velocity_ms, last_set_.set_velocity_ms >= 0);
        out << ",\"remaining_before_set_ms\":"; json_number(out, last_set_.remaining_before_set_ms, last_set_.before_set);
        out << ",\"remaining_after_set_ms\":"; json_number(out, last_set_.remaining_after_set_ms, last_set_.after_set);
        out << "},\"set_attempts\":" << set_attempts_
            << ",\"action_start_attempts\":" << action_start_attempts_
            << ",\"action_starts\":" << action_starts_
            << ",\"stop_attempts\":" << stop_attempts_
            << ",\"confirmed_stops\":" << confirmed_stops_
            << ",\"heartbeat_refreshes\":" << heartbeat_refreshes_
            << ",\"soft_stop_attempts\":" << soft_stop_attempts_
            << ",\"soft_stops\":" << soft_stops_
            << ",\"soft_stopped\":" << (soft_stopped_ ? "true" : "false")
            << ",\"staged_stop\":" << (staged_ ? "true" : "false");
        out << ",\"last_heartbeat_ms\":"; json_number(out, last_heartbeat_ms_, last_heartbeat_ms_ >= 0);
        out << ",\"max_heartbeat_ms\":"; json_number(out, max_heartbeat_ms_, max_heartbeat_ms_ >= 0);
        out << ",\"stop_reason\":\"" << reason_name(stop_reason_) << "\"}";
        return out.str();
    }
};
