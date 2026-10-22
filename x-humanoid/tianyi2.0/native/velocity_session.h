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
    Time deadline_, refreshed_;
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
        ++set_attempts_;
        last_set_ = SetTiming();
        if (deadline <= Clock::now()) throw CommandLeaseExpired("command lease expired");
        deadline_ = deadline;
        if (!motion_possible_) {
            bool busy = false;
            measured(last_set_.busy_ms, [&] { busy = sdk_.busy(); });
            if (busy) throw std::runtime_error("another chassis action is active");
            // Do not create any motion until firmware dead-client protection
            // has acknowledged. No heartbeat is installed for idle/dry-run.
            measured(last_set_.start_heartbeat_ms, [&] { sdk_.start_heartbeat(); });
            heartbeat_ = true;
            refreshed_ = Clock::now();
            if (deadline_ <= Clock::now()) { stop(StopReason::LeaseExpired); throw CommandLeaseExpired("lease expired before action creation"); }
            motion_possible_ = true; // includes ambiguous begin() failures
            ++action_start_attempts_;
            measured(last_set_.begin_ms, [&] { sdk_.begin(); });
            ++action_starts_;
        }
        measured(last_set_.verify_owned_ms, [&] { sdk_.verify_owned(); });
        last_set_.before_set = true;
        last_set_.remaining_before_set_ms = remaining_ms();
        if (deadline_ <= Clock::now()) { stop(StopReason::LeaseExpired); throw CommandLeaseExpired("lease expired before velocity write"); }
        try { measured(last_set_.set_velocity_ms, [&] { sdk_.set(vx, wz); }); }
        catch (...) {
            last_set_.after_set = true;
            last_set_.remaining_after_set_ms = remaining_ms();
            throw;
        }
        last_set_.after_set = true;
        last_set_.remaining_after_set_ms = remaining_ms();
        if (deadline_ <= Clock::now()) { stop(StopReason::LeaseExpired); throw CommandLeaseExpired("lease expired during SDK write"); }
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
    void tick(Time now) {
        if (!motion_possible_) return;
        if (now >= deadline_) { stop(StopReason::LeaseExpired); return; }
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
            << ",\"heartbeat_refreshes\":" << heartbeat_refreshes_;
        out << ",\"last_heartbeat_ms\":"; json_number(out, last_heartbeat_ms_, last_heartbeat_ms_ >= 0);
        out << ",\"max_heartbeat_ms\":"; json_number(out, max_heartbeat_ms_, max_heartbeat_ms_ >= 0);
        out << ",\"stop_reason\":\"" << reason_name(stop_reason_) << "\"}";
        return out.str();
    }
};
