// Hardware-free trajectory and wire validation. Also runnable with:
// c++ -std=c++17 -Wall -Wextra -Wpedantic test_joint_motion.cpp -o /tmp/fr3-motion-test
#include "joint_motion.h"

#include <iostream>
#include <limits>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
template <typename Function>
void rejects(Function function) {
  bool rejected = false;
  try { function(); } catch (const std::exception&) { rejected = true; }
  require(rejected, "Expected malformed/unsafe command to be rejected");
}
}  // namespace

int main() {
  const auto command = fr3::parse_command("MOVE abc_1 0.08726646259971647 0.008726646259971648 120 0 -0.5 0 -1.5 0 1.5 0");
  require(command.id == "abc_1", "Command identity not preserved");
  require(fr3::parse_command("STOP abc_1").action == "STOP", "STOP not parsed");
  require(fr3::parse_command("QUIT").action == "QUIT", "QUIT not parsed");
  for (const auto* line : {
       "MOVE a 0.1 0.01 120 0 0 0 -1 0 1",  // missing joint
       "MOVE a 0.1 0.01 120 0 0 0 -1 0 1 nan",
       "MOVE a 1 0.01 120 0 0 0 -1 0 1 0",  // excessive speed
       "MOVE a 0.1 0.01 120 0 0 0 0 0 1 0", // J4 limit
       "MOVE a 0.1 0.01 120 0 0 0 -1 0 0 0",// J6 limit
       "MOVE a 0.1 0.01 0 0 0 0 -1 0 1 0",  // deadline
       "MOVE a 0.1 0.1 120 0 0 0 -1 0 1 0", // tolerance
       "STOP bad/id", "STOP id extra", "QUIT extra", "RECOVER id"}) {
    rejects([&] { fr3::parse_command(line); });
  }
  auto start = command.target;
  start[0] = -0.4;
  start[2] = 0.2;
  start[5] = 1.0;
  const fr3::Trajectory trajectory(start, command);
  require(fr3::max_error(trajectory.at(0), start) == 0, "Trajectory start is discontinuous");
  require(fr3::max_error(trajectory.at(trajectory.duration()), command.target) < 1e-12,
          "Trajectory does not reach its target");
  auto previous = start;
  const double step = trajectory.duration() / 20000.0;
  auto previous_velocity = fr3::Joints{};
  auto previous_acceleration = fr3::Joints{};
  for (int sample = 1; sample <= 20000; ++sample) {
    const auto current = trajectory.at(sample * step);
    fr3::validate_joints(current);
    for (std::size_t joint = 0; joint < 7; ++joint) {
      const double velocity = (current[joint] - previous[joint]) / step;
      const double acceleration = (velocity - previous_velocity[joint]) / step;
      const double jerk = (acceleration - previous_acceleration[joint]) / step;
      require(std::abs(velocity) <= command.speed + 1e-6, "Velocity bound exceeded");
      require(std::abs(acceleration) <= 0.30001, "Acceleration bound exceeded");
      require(std::abs(jerk) <= 1.001, "Jerk bound exceeded");
      const double direction = command.target[joint] - start[joint];
      require(direction * velocity >= -1e-10, "Trajectory is not monotonic");
      previous_velocity[joint] = velocity;
      previous_acceleration[joint] = acceleration;
    }
    previous = current;
  }
  auto invalid = command;
  invalid.timeout = 1;
  rejects([&] { fr3::Trajectory too_long(start, invalid); });
  invalid.target[0] = std::numeric_limits<double>::infinity();
  rejects([&] { fr3::validate_joints(invalid.target); });
  invalid.target = command.target;
  invalid.target[0] = fr3::kUpper[0];
  rejects([&] { fr3::validate_joints(invalid.target); });
  require(fr3::max_error(start, command.target) == 0.5, "Measured completion error is wrong");
  auto continuous = command;
  continuous.path = {start, start, command.target, command.target};
  continuous.timeout = 600;
  const fr3::PathTrajectory smooth(start, continuous);
  require(fr3::max_error(smooth.at(0), start) < 1e-12, "PATH start jumped");
  require(fr3::max_error(smooth.at(smooth.duration()), command.target) < 1e-12, "PATH end missed");
  const double dt = smooth.duration()/50000;
  previous = smooth.at(0);
  previous_velocity = {};
  previous_acceleration = {};
  for (int n=1; n<=50000; ++n) {
    const auto q = smooth.at(n*dt);
    fr3::validate_joints(q);
    for (std::size_t j=0;j<7;++j) {
      const double v = (q[j]-previous[j])/dt;
      const double a = (v-previous_velocity[j])/dt;
      const double jerk = (a-previous_acceleration[j])/dt;
      require(std::abs(v) <= continuous.speed+1e-6, "PATH velocity exceeded");
      require(std::abs(a) <= .15001, "PATH acceleration exceeded");
      require(std::abs(jerk) <= .501, "PATH jerk exceeded");
      previous_velocity[j]=v;
      previous_acceleration[j]=a;
    }
    previous=q;
  }
  auto wrong_start=start;
  wrong_start[0]+=.01;
  rejects([&] { fr3::PathTrajectory bad(wrong_start, continuous); });
  require(fr3::parse_command("PATH p .02 .001 600 2 0 0 0 -1 0 1 0 .1 0 0 -1 0 1 0").path.size()==2,
          "PATH wire format failed");
  rejects([&] { fr3::parse_command("PATH p .02 .001 600 22"); });
  rejects([&] { fr3::parse_command("PATH p .02 .001 600 2 0"); });
  std::cout << "FR3 command validation and trajectory bounds passed\n";
}
