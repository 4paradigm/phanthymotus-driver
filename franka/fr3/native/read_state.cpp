// Read-only bridge: deliberately exposes no motion or robot configuration API.
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>

namespace {
volatile std::sig_atomic_t running = 1;
void handle_signal(int) { running = 0; }

void write_array(const char* name, const std::array<double, 7>& values) {
  std::cout << '"' << name << "\":[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) std::cout << ',';
    if (!std::isfinite(values[i])) throw std::runtime_error("Non-finite robot state");
    std::cout << values[i];
  }
  std::cout << ']';
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: franka-state-reader ROBOT_IP\n";
    return 2;
  }
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  std::cout.imbue(std::locale::classic());
  std::cout << std::setprecision(17);
  try {
    // No real-time scheduling is required for observation. The parent enforces
    // connect/read deadlines by terminating this process if the SDK blocks.
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    auto next_sample = std::chrono::steady_clock::now();
    robot.read([&next_sample](const franka::RobotState& state) {
      if (!running) return false;
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_sample) {
        std::cout << '{';
        write_array("q", state.q);
        std::cout << ',';
        write_array("dq", state.dq);
        std::cout << ',';
        write_array("tau_J", state.tau_J);
        std::cout << ",\"robot_mode\":" << static_cast<int>(state.robot_mode) << "}\n";
        std::cout.flush();
        if (!std::cout.good()) return false;
        next_sample = now + std::chrono::microseconds(33333);
      }
      return running != 0;
    });
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
