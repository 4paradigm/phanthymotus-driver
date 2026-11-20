#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <memory>
#include <functional>

namespace fr3 {
using Joints = std::array<double, 7>;
// FR3 limits from resource/fr3.urdf; keep a small distance from hard limits.
constexpr Joints kLower{-2.9007, -1.8361, -2.9007, -3.077, -2.8763, 0.4398, -3.0508};
constexpr Joints kUpper{2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508};
constexpr double kLimitMargin = 0.01;
constexpr double kMinSpeed = 0.0017453292519943296;  // 0.1 degree/s
constexpr double kMaxSpeed = 0.17453292519943295;    // 10 degrees/s

struct TimedSegment {
  double end{};
  std::array<Joints, 6> controls{};
};

struct Command {
  std::string action;
  std::string id;
  double speed = 0.0;
  double tolerance = 0.0;
  double timeout = 0.0;
  Joints target{};
  std::vector<Joints> path;
  std::vector<TimedSegment> timed;
};

inline bool valid_id(const std::string& id) {
  return !id.empty() && id.size() <= 64 &&
         std::all_of(id.begin(), id.end(), [](unsigned char c) {
           return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-';
         });
}

inline void validate_joints(const Joints& joints) {
  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (!std::isfinite(joints[i]) || joints[i] < kLower[i] + kLimitMargin ||
        joints[i] > kUpper[i] - kLimitMargin) {
      throw std::invalid_argument("J" + std::to_string(i + 1) +
                                  " is outside FR3 joint limits (0.01 rad margin)");
    }
  }
}

inline Command parse_command(const std::string& line) {
  std::istringstream input(line);
  input.imbue(std::locale::classic());
  Command command;
  input >> command.action;
  if (command.action != "QUIT") {
    if (!(input >> command.id) || !valid_id(command.id)) {
      throw std::invalid_argument("Invalid command ID");
    }
  }
  if (command.action == "MOVE" || command.action == "PATH" || command.action == "TIMED") {
    if (!(input >> command.speed >> command.tolerance >> command.timeout)) {
      throw std::invalid_argument("MOVE requires speed, tolerance and timeout");
    }
    if (command.action == "PATH") {
      int count = 0;
      if (!(input >> count) || count < 2 || count > 21)
        throw std::invalid_argument("PATH requires 2 to 21 control points");
      command.path.resize(count);
      for (auto& point : command.path) {
        for (auto& q : point)
          if (!(input >> q)) throw std::invalid_argument("Incomplete PATH control point");
        validate_joints(point);
      }
      command.target = command.path.back();
    } else if (command.action == "TIMED") {
      int count = 0;
      if (!(input >> count) || count < 1 || count > 512)
        throw std::invalid_argument("TIMED requires 1 to 512 segments");
      double previous = 0.0;
      command.timed.resize(count);
      for (auto& segment : command.timed) {
        if (!(input >> segment.end) || !std::isfinite(segment.end) || segment.end - previous < 0.001)
          throw std::invalid_argument("Invalid TIMED segment time");
        previous = segment.end;
        for (auto& point : segment.controls) {
          for (auto& q : point)
            if (!(input >> q)) throw std::invalid_argument("Incomplete TIMED control point");
          validate_joints(point);
        }
      }
      command.target = command.timed.back().controls.back();
    } else for (auto& q : command.target) {
      if (!(input >> q)) throw std::invalid_argument("MOVE requires seven joint angles in radians");
    }
    if (!std::isfinite(command.speed) || command.speed < kMinSpeed || (command.action == "MOVE" && command.speed > kMaxSpeed))
      throw std::invalid_argument("Invalid joint speed (MOVE: 0.1–10 deg/s; PATH: at least 0.1 deg/s)");
    if (!std::isfinite(command.tolerance) || command.tolerance <= 0 ||
        command.tolerance > 0.017453292519943295)
      throw std::invalid_argument("Tolerance must be positive and at most one degree (in radians)");
    if (!std::isfinite(command.timeout) || command.timeout < 1 || command.timeout > 600)
      throw std::invalid_argument("Timeout must be 1 to 600 seconds");
    validate_joints(command.target);
  } else if (command.action != "STOP" && command.action != "QUIT") {
    throw std::invalid_argument("Unknown command");
  }
  std::string extra;
  if (input >> extra) throw std::invalid_argument("Unexpected trailing command data");
  return command;
}

// Seventh-order monotonic blend: velocity, acceleration and jerk all vanish at
// both ends. Conservative analytic maxima bound each joint to speed, 0.3 rad/s²
// and 1 rad/s³. The SDK's rate limiter remains enabled as a second boundary.
class Trajectory {
 public:
  Trajectory(const Joints& start, const Command& command) : start_(start), goal_(command.target) {
    validate_joints(start);
    validate_joints(goal_);
    double distance = 0.0;
    for (std::size_t i = 0; i < 7; ++i)
      distance = std::max(distance, std::abs(goal_[i] - start_[i]));
    duration_ = std::max({1.0, 2.1875 * distance / command.speed,
                          std::sqrt(7.514 * distance / 0.3), std::cbrt(52.5 * distance)});
    if (duration_ + 2.0 > command.timeout)
      throw std::invalid_argument("Trajectory exceeds timeout at the selected speed");
  }

  Joints at(double elapsed) const noexcept {
    const double t = std::clamp(elapsed / duration_, 0.0, 1.0);
    const double blend = t * t * t * t * (35.0 + t * (-84.0 + t * (70.0 - 20.0 * t)));
    Joints result{};
    for (std::size_t i = 0; i < 7; ++i) result[i] = start_[i] + (goal_[i] - start_[i]) * blend;
    return result;
  }
  double duration() const noexcept { return duration_; }

 private:
  Joints start_, goal_;
  double duration_;
};

// Execute the same quintic Bernstein polynomials checked by the planner.
class TimedTrajectory {
  static constexpr double kAccelerationLimit = 10.0;  // FR3 FCI, rad/s²
  static constexpr double kJerkLimit = 5000.0;  // FR3 FCI, rad/s³

 public:
  TimedTrajectory(const Joints& start, const Command& command)
      : segments_(command.timed), lead_(start, lead_command(command)) {
    const auto& first = segments_.front().controls.front();
    const auto& last = segments_.back().controls.back();
    double previous = 0.0;
    Joints p = first, v{}, a{};
    for (const auto& segment : segments_) {
      const auto& c = segment.controls;
      const double dt = segment.end - previous;
      for (size_t j = 0; j < 7; ++j) {
        if (std::abs(start[j] - first[j]) > .001 ||
            std::abs(c[0][j] - p[j]) > 1e-8 ||
            std::abs(5 * (c[1][j] - c[0][j]) / dt - v[j]) > 1e-6 ||
            std::abs(20 * (c[2][j] - 2*c[1][j] + c[0][j]) / (dt*dt) - a[j]) > 1e-4)
          throw std::invalid_argument("TIMED start or segment derivatives are discontinuous");
        for (size_t k = 0; k < 5; ++k)
          if (std::abs(5 * (c[k+1][j] - c[k][j]) / dt) > command.speed * 1.001)
            throw std::invalid_argument("TIMED joint velocity exceeds requested speed");
        for (size_t k = 0; k < 4; ++k)
          if (std::abs(20 * (c[k+2][j] - 2*c[k+1][j] + c[k][j]) / (dt*dt)) >= kAccelerationLimit)
            throw std::invalid_argument("TIMED joint acceleration exceeds limit");
        for (size_t k = 0; k < 3; ++k)
          if (std::abs(60 * (c[k+3][j] - 3*c[k+2][j] + 3*c[k+1][j] - c[k][j]) / (dt*dt*dt)) >= kJerkLimit)
            throw std::invalid_argument("TIMED joint jerk exceeds limit");
        p[j] = c[5][j];
        v[j] = 5 * (c[5][j] - c[4][j]) / dt;
        a[j] = 20 * (c[5][j] - 2*c[4][j] + c[3][j]) / (dt*dt);
      }
      previous = segment.end;
    }
    for (size_t j = 0; j < last.size(); ++j)
      if (std::abs(v[j]) > 1e-6 || std::abs(a[j]) > 1e-4)
        throw std::invalid_argument("TIMED endpoint must be stationary");
    if (duration() + 2.0 > command.timeout)
      throw std::invalid_argument("TIMED trajectory exceeds timeout");
  }

  Joints at(double elapsed) const noexcept {
    if (elapsed < lead_.duration()) return lead_.at(elapsed);
    elapsed -= lead_.duration();
    const auto it = std::upper_bound(segments_.begin(), segments_.end(), elapsed,
        [](double time, const TimedSegment& segment) { return time < segment.end; });
    if (it == segments_.end()) return segments_.back().controls.back();
    const double start = it == segments_.begin() ? 0.0 : (it-1)->end;
    const double t = (elapsed - start) / (it->end - start);
    auto c = it->controls;
    for (size_t n = c.size()-1; n > 0; --n)
      for (size_t k = 0; k < n; ++k)
        for (size_t j = 0; j < c[k].size(); ++j)
          c[k][j] += t * (c[k+1][j] - c[k][j]);
    return c.front();
  }

  double duration() const noexcept { return lead_.duration() + segments_.back().end; }

 private:
  static Command lead_command(const Command& original) {
    Command lead;
    lead.target = original.timed.front().controls.front();
    lead.speed = original.speed;
    lead.timeout = original.timeout;
    return lead;
  }

  const std::vector<TimedSegment>& segments_;
  Trajectory lead_;
};

// Uniform cubic B-spline. Repeated end points give zero start/end velocity
// and acceleration. The curve stays in each four-point convex hull.
class PathTrajectory {
 public:
  using VelocityLimits = std::function<Joints(const Joints&)>;
  PathTrajectory(const Joints& start, const Command& command,
                 VelocityLimits upper = {}, VelocityLimits lower = {}) {
    if (command.path.empty()) {
      single_ = true;
      single_path_ = std::make_unique<Trajectory>(start, command);
      return;
    }
    for (std::size_t j = 0; j < 7; ++j)
      if (std::abs(start[j]-command.path.front()[j]) > 0.001)
        throw std::invalid_argument("PATH start differs from desired robot position");
    points_.push_back(command.path.front());
    points_.push_back(command.path.front());
    points_.insert(points_.end(), command.path.begin(), command.path.end());
    points_.push_back(command.path.back());
    points_.push_back(command.path.back());
    double velocity = 0, acceleration = 0, jerk = 0;
    for (std::size_t i = 1; i < points_.size(); ++i)
      for (std::size_t j = 0; j < 7; ++j) {
        velocity = std::max(velocity, std::abs(points_[i][j]-points_[i-1][j]));
        if (i >= 2) acceleration = std::max(acceleration,
          std::abs(points_[i][j]-2*points_[i-1][j]+points_[i-2][j]));
        if (i >= 3) jerk = std::max(jerk,
          std::abs(points_[i][j]-3*points_[i-1][j]+3*points_[i-2][j]-points_[i-3][j]));
      }
    interval_ = std::max({.05, velocity/command.speed,
                         std::sqrt(acceleration/.15), std::cbrt(jerk/.5)});
    if (upper && lower) {
      for (std::size_t i = 0; i + 3 < points_.size(); ++i) {
        Joints lo{}, hi{};
        for (std::size_t j = 0; j < 7; ++j) {
          auto bounds = {points_[i][j], points_[i+1][j], points_[i+2][j], points_[i+3][j]};
          lo[j] = std::min(bounds);
          hi[j] = std::max(bounds);
        }
        // Spline positions lie in the control-point hull. The directional
        // velocity limit is tightest on the hull edge nearest the joint limit.
        const auto positive = upper(hi);
        const auto negative = lower(lo);
        for (std::size_t k = i; k < i + 3; ++k)
          for (std::size_t j = 0; j < 7; ++j) {
            const double step = points_[k+1][j] - points_[k][j];
            if (step == 0) continue;
            const double limit = step > 0 ? positive[j] : -negative[j];
            if (!std::isfinite(limit) || limit <= 0)
              throw std::invalid_argument("PATH reaches a joint position with no velocity allowance");
            interval_ = std::max(interval_, std::abs(step) / limit);
          }
      }
    }
    duration_ = interval_*(points_.size()-3);
    if (duration_+2 > command.timeout)
      throw std::invalid_argument("PATH exceeds timeout at the selected speed");
  }
  Joints at(double elapsed) const noexcept {
    if (single_) return single_path_->at(elapsed);
    if (elapsed >= duration_) return points_.back();
    const double scaled = std::max(0.0, elapsed)/interval_;
    const auto i = static_cast<std::size_t>(scaled);
    const double u = scaled-i;
    const double weights[4] = {std::pow(1-u,3)/6, (3*u*u*u-6*u*u+4)/6,
                               (-3*u*u*u+3*u*u+3*u+1)/6, u*u*u/6};
    Joints result{};
    for (std::size_t j=0;j<7;++j)
      for (std::size_t k=0;k<4;++k) result[j]+=weights[k]*points_[i+k][j];
    return result;
  }
  double duration() const noexcept { return single_ ? single_path_->duration() : duration_; }
 private:
  bool single_ = false;
  std::unique_ptr<Trajectory> single_path_;
  std::vector<Joints> points_;
  double interval_ = 0, duration_ = 0;
};

inline double max_error(const Joints& actual, const Joints& target) {
  double result = 0.0;
  for (std::size_t i = 0; i < 7; ++i) {
    if (!std::isfinite(actual[i]) || !std::isfinite(target[i]))
      throw std::runtime_error("Non-finite robot state");
    result = std::max(result, std::abs(actual[i] - target[i]));
  }
  return result;
}
}  // namespace fr3
