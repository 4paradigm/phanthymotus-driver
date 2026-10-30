#pragma once

#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/motion_planner/velocity_control_move_action.h>
#include <stdexcept>

// Only this file depends on the vendor SDK. Cancellation always addresses the
// held action handle, never a global "cancel current" that could stop home/nav.
class SlamwareAdapter {
    rpos::robot_platforms::SlamwareCorePlatform platform_;
    rpos::actions::VelocityControlMoveAction action_;
    rpos::features::system_resource::HeartBeatToken token_;
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
        if (!action_ || !current || current.getActionId() != action_.getActionId())
            throw std::runtime_error("velocity action was replaced; explicit restart required");
        auto state = current.getStatus();
        if (state != rpos::core::ActionStatusRunning && state != rpos::core::ActionStatusWaitingForStart)
            throw std::runtime_error("velocity action is not running: " + current.getReason());
    }
    void set(double vx, double wz) { action_.setVelocity(vx, 0.0f, wz); }
    void cancel() {
        if (!action_) throw std::runtime_error("velocity action creation unconfirmed");
        action_.cancel();
        action_ = rpos::actions::VelocityControlMoveAction();
    }
    unsigned long action_id() { return action_ ? action_.getActionId() : 0; }
};
