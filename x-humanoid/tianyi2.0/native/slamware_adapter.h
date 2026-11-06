#pragma once

#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/motion_planner/velocity_control_move_action.h>
#include <sstream>
#include <stdexcept>

// Only this file depends on the vendor SDK. Cancellation always addresses the
// held action handle, never a global "cancel current" that could stop home/nav.
class SlamwareAdapter {
    rpos::robot_platforms::SlamwareCorePlatform platform_;
    rpos::actions::VelocityControlMoveAction action_;
    rpos::features::system_resource::HeartBeatToken token_;

    // These reads are only used after an ownership check has already failed.
    // A failed diagnostic read must not hide the original failure or turn an
    // empty current action into a claim that another controller replaced us.
    template<class Action>
    static std::string describe_action(const char* label, Action& action) {
        std::ostringstream out;
        out << label << '=';
        if (!action) return out.str() + "empty";
        out << "{id=";
        try { out << action.getActionId(); }
        catch (...) { out << "unavailable"; }
        out << ",status=";
        try { out << static_cast<int>(action.getStatus()); }
        catch (...) { out << "unavailable"; }
        out << ",reason=";
        try { out << action.getReason(); }
        catch (...) { out << "unavailable"; }
        out << '}';
        return out.str();
    }
public:
    SlamwareAdapter(const std::string& host, int port, int timeout)
        : platform_(rpos::robot_platforms::SlamwareCorePlatform::connect(host, port, timeout)) {}
    bool busy() {
        auto current = platform_.getCurrentAction();
        if (!current) return false;
        auto state = current.getStatus();
        return state != rpos::core::ActionStatusFinished &&
               state != rpos::core::ActionStatusStopped &&
               state != rpos::core::ActionStatusError;
    }
    void start_heartbeat() { token_ = platform_.startHeartBeat(1); }
    void refresh_heartbeat() { platform_.refreshHeartBeat(token_); }
    void stop_heartbeat() { platform_.stopHeartBeat(token_); }
    void begin() {
        // Preserve the official default localization-quality monitor.
        action_ = platform_.velocityControl();
        if (!action_) throw std::runtime_error("SDK returned an empty velocity action");
    }
    void verify_owned() {
        auto current = platform_.getCurrentAction();
        if (!action_)
            throw std::runtime_error("velocity action ownership check failed: owned action handle is empty; " +
                                     describe_action("current", current) + "; explicit restart required");
        if (!current)
            throw std::runtime_error("velocity action ownership check failed: current action is empty; " +
                                     describe_action("owned", action_) + "; explicit restart required");
        if (current.getActionId() != action_.getActionId())
            throw std::runtime_error("velocity action was replaced; " + describe_action("owned", action_) +
                                     "; " + describe_action("current", current) + "; explicit restart required");
        auto state = current.getStatus();
        if (state != rpos::core::ActionStatusRunning && state != rpos::core::ActionStatusWaitingForStart)
            throw std::runtime_error("velocity action is not running; " + describe_action("owned", action_) +
                                     "; " + describe_action("current", current) + "; explicit restart required");
    }
    void set(double vx, double wz) { action_.setVelocity(vx, 0.0f, wz); }
    void cancel() {
        if (!action_) throw std::runtime_error("velocity action creation unconfirmed");
        action_.cancel();
        action_ = rpos::actions::VelocityControlMoveAction();
    }
    unsigned long action_id() { return action_ ? action_.getActionId() : 0; }
};
