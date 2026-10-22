#pragma once

#include <chrono>
#include <stdexcept>

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
private:
    Adapter& sdk_;
    bool heartbeat_ = false;
    bool motion_possible_ = false;
    Time deadline_, refreshed_;
public:
    explicit VelocitySession(Adapter& sdk) : sdk_(sdk) {}
    bool active() const { return motion_possible_; }
    void stop() {
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
    }
    void set(double vx, double wz, Time deadline) {
        if (deadline <= Clock::now()) throw CommandLeaseExpired("command lease expired");
        deadline_ = deadline;
        if (!motion_possible_) {
            if (sdk_.busy()) throw std::runtime_error("another chassis action is active");
            // Do not create any motion until firmware dead-client protection
            // has acknowledged. No heartbeat is installed for idle/dry-run.
            sdk_.start_heartbeat();
            heartbeat_ = true;
            refreshed_ = Clock::now();
            if (deadline_ <= Clock::now()) { stop(); throw CommandLeaseExpired("lease expired before action creation"); }
            motion_possible_ = true; // includes ambiguous begin() failures
            sdk_.begin();
        }
        sdk_.verify_owned();
        if (deadline_ <= Clock::now()) { stop(); throw CommandLeaseExpired("lease expired before velocity write"); }
        sdk_.set(vx, wz);
        if (deadline_ <= Clock::now()) { stop(); throw CommandLeaseExpired("lease expired during SDK write"); }
    }
    void tick(Time now) {
        if (!motion_possible_) return;
        if (now >= deadline_) { stop(); return; }
        if (now - refreshed_ >= std::chrono::milliseconds(100)) {
            sdk_.refresh_heartbeat();
            refreshed_ = Clock::now();
        }
    }
};
