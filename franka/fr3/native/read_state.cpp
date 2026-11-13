// One connection per robot: idle state reads and joint control never overlap.
// stdin: MOVE id speed_rad_s tolerance_rad timeout_s q1..q7 | STOP id | QUIT
// stdout: newline-delimited state JSON and asynchronous motion/stop events.
#include "joint_motion.h"

#include <franka/control_tools.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void handle_signal(int) { interrupted = 1; }

std::string quoted(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(c);
    else out << c;
  }
  out << '"';
  return out.str();
}

template <size_t N>
void write_array(std::ostream& out, const char* name, const std::array<double, N>& values) {
  out << '"' << name << "\":[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) out << ',';
    if (!std::isfinite(values[i])) throw std::runtime_error("Non-finite robot state");
    out << values[i];
  }
  out << ']';
}

struct RealtimeStatus {
  bool kernel = false, scheduling = false, standard_kernel = false;
  std::string error;
  bool ready() const { return (kernel || standard_kernel) && scheduling; }
};

RealtimeStatus check_realtime() {
  RealtimeStatus result;
  const char* policy = std::getenv("FRANKA_REALTIME_POLICY");
  const std::string mode = policy ? policy : "enforce";
  if (mode != "enforce" && mode != "standard")
    throw std::runtime_error("FRANKA_REALTIME_POLICY must be enforce or standard");
  result.standard_kernel = mode == "standard";
  result.kernel = franka::hasRealtimeKernel();
  std::string detail;
  result.scheduling = franka::setCurrentThreadToHighestSchedulerPriority(&detail);
  if (!result.kernel && !result.standard_kernel) result.error = "PREEMPT_RT kernel required for motion; ";
  if (!result.scheduling) {
    result.error += detail.empty() ? "SCHED_FIFO permission required for motion" : detail;
  }
  return result;
}

void write_realtime(std::ostream& out, const RealtimeStatus& rt) {
  out << "\"realtime_policy\":" << quoted(rt.standard_kernel ? "standard" : "enforce")
      << ",\"rt_kernel\":" << (rt.kernel ? "true" : "false")
      << ",\"sched_fifo\":" << (rt.scheduling ? "true" : "false")
      << ",\"control_ready\":" << (rt.ready() ? "true" : "false")
      << ",\"control_error\":" << (rt.ready() ? "null" : quoted(rt.error));
}

// Only the thread doing SDK read/control may have realtime priority. I/O must
// never inherit the constructor's FIFO priority and contend with its 1 kHz loop.
void normal_priority() {
  sched_param parameters{};
  if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &parameters) != 0)
    throw std::runtime_error("Cannot set helper thread to SCHED_OTHER");
}

std::string canonical_ip(const std::string& value) {
  in_addr v4{};
  in6_addr v6{};
  char rendered[INET6_ADDRSTRLEN]{};
  if (inet_pton(AF_INET, value.c_str(), &v4) == 1) {
    inet_ntop(AF_INET, &v4, rendered, sizeof(rendered));
  } else if (inet_pton(AF_INET6, value.c_str(), &v6) == 1) {
    if (IN6_IS_ADDR_V4MAPPED(&v6)) inet_ntop(AF_INET, &v6.s6_addr[12], rendered, sizeof(rendered));
    else inet_ntop(AF_INET6, &v6, rendered, sizeof(rendered));
  } else {
    throw std::invalid_argument("ROBOT_IP must be an IP literal");
  }
  return rendered;
}

class RobotLock {
 public:
  explicit RobotLock(const std::string& address) {
    const char* configured = std::getenv("FRANKA_LOCK_DIR");
    std::string directory = configured ? configured : "/tmp/phanthymotus-franka";
    if (directory.empty() || directory.front() != '/')
      throw std::invalid_argument("FRANKA_LOCK_DIR must be an absolute directory");
    if (mkdir(directory.c_str(), 0755) != 0 && errno != EEXIST)
      throw std::runtime_error("Cannot create Franka lock directory");
    const auto path = directory + "/" + address + ".lock";
    fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd_ < 0) throw std::runtime_error("Cannot open robot connection lock: " + path);
    if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("Another Franka process owns " + address);
    }
  }
  ~RobotLock() { if (fd_ >= 0) close(fd_); }
 private:
  int fd_ = -1;
};

struct Sample {
  fr3::Joints q{}, q_d{}, dq{}, tau{};
  int mode = 0;
  std::array<double, 16> O_T_EE{};
  std::array<double, 16> F_T_EE{};
};

// Single SDK producer, single stdout consumer. A full queue drops telemetry;
// it never stalls or takes a lock in a realtime callback.
class Samples {
 public:
  void push(const Sample& value) noexcept {
    const auto write = write_.load(std::memory_order_relaxed);
    const auto next = (write + 1) % data_.size();
    if (next == read_.load(std::memory_order_acquire)) return;
    data_[write] = value;
    write_.store(next, std::memory_order_release);
  }
  bool pop(Sample& value) noexcept {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) return false;
    value = data_[read];
    read_.store((read + 1) % data_.size(), std::memory_order_release);
    return true;
  }
 private:
  std::array<Sample, 64> data_{};
  std::atomic<std::size_t> read_{0}, write_{0};
};

class Session {
 public:
  Session(franka::Robot& robot, RealtimeStatus rt) : robot_(robot), rt_(std::move(rt)) {}

  void run() {
    output_ = std::thread([this] { output_loop(); });
    input_ = std::thread([this] { input_loop(); });
    try {
      while (!quitting_ && !interrupted) {
        robot_.read([this](const franka::RobotState& state) {
          sample(state);
          return !quitting_ && !interrupted && !pending_;
        });
        if (pending_) {
          fr3::Command command;
          {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command = command_;
            pending_ = false;
          }
          execute(command);
        }
      }
    } catch (...) {
      finish_threads();
      throw;
    }
    finish_threads();
  }

 private:
  void sample(const franka::RobotState& state) {
    const auto now = Clock::now();
    if (now < next_sample_) return;
    samples_.push({state.q, state.q_d, state.dq, state.tau_J, static_cast<int>(state.robot_mode), state.O_T_EE, state.F_T_EE});
    next_sample_ = now + std::chrono::microseconds(33333);
  }

  void event(const std::string& kind, const std::string& id, const std::string& status,
             const std::string& error = "", const fr3::Joints* target = nullptr,
             std::optional<double> maximum_error = std::nullopt) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << "{\"event\":" << quoted(kind)
        << ",\"command_id\":" << quoted(id) << ",\"status\":" << quoted(status);
    if (!error.empty()) out << ",\"error\":" << quoted(error.substr(0, 2048));
    if (target) { out << ','; write_array(out, "target", *target); }
    if (maximum_error) out << ",\"max_error\":" << *maximum_error;
    out << '}';
    std::lock_guard<std::mutex> lock(event_mutex_);
    if (events_.size() >= 512) { quitting_ = true; cancelled_ = true; return; }
    events_.push_back(out.str());
  }

  void output_loop() {
    try {
      normal_priority();
      for (;;) {
        Sample current;
        while (samples_.pop(current)) {
          std::cout << "{\"type\":\"state\",";
          write_array(std::cout, "q", current.q); std::cout << ',';
          write_array(std::cout, "q_d", current.q_d); std::cout << ',';
          write_array(std::cout, "dq", current.dq); std::cout << ',';
          write_array(std::cout, "tau_J", current.tau);
          std::cout << ',';
          write_array(std::cout, "O_T_EE", current.O_T_EE);
          std::cout << ',';
          write_array(std::cout, "F_T_EE", current.F_T_EE);
          std::cout << ",\"robot_mode\":" << current.mode
                    << ",\"capabilities\":[\"joint_position\"],";
          write_realtime(std::cout, rt_);
          std::cout << "}\n";
        }
        std::deque<std::string> messages;
        {
          std::lock_guard<std::mutex> lock(event_mutex_);
          messages.swap(events_);
        }
        for (const auto& message : messages) std::cout << message << '\n';
        std::cout.flush();
        if (!std::cout.good()) throw std::runtime_error("Parent state pipe closed");
        last_output_ = Clock::now().time_since_epoch().count();
        if (output_done_) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    } catch (const std::exception& error) {
      std::cerr << error.what() << '\n';
      quitting_ = true;
      cancelled_ = true;
    }
  }

  void accept(const std::string& line) {
    fr3::Command incoming;
    try { incoming = fr3::parse_command(line); }
    catch (const std::exception& error) {
      std::istringstream input(line);
      std::string action, id;
      input >> action >> id;
      event("motion", fr3::valid_id(id) ? id : "", "rejected", error.what());
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (incoming.action == "QUIT") { quitting_ = true; cancelled_ = true; return; }
    if (incoming.action == "STOP") {
      if (!busy_ || incoming.id != command_.id) {
        event("stop", incoming.id, "idle");
      } else {
        cancelled_ = true;
      }
      return;
    }
    if (!rt_.ready() || quitting_ || busy_) {
      event("motion", incoming.id, "rejected", !rt_.ready() ? rt_.error :
            (busy_ ? "Robot already has an active motion" : "Session is stopping"));
      return;
    }
    command_ = incoming;
    busy_ = true;
    cancelled_ = false;
    stop_called_ = false;
    stop_error_.clear();
    event("motion", command_.id, "accepted", "", &command_.target);
    pending_ = true;
  }

  void input_loop() {
    try {
      normal_priority();
      std::string pending;
      bool eof = false;
      while (!input_done_) {
        if (interrupted) { quitting_ = true; cancelled_ = true; }
        const auto since_output = Clock::now() - Clock::time_point(Clock::duration(last_output_.load()));
        if (in_control_ && since_output > std::chrono::seconds(1)) {
          quitting_ = true;
          cancelled_ = true;
        }
        // Robot::stop sends StopMove to the real backend from a non-RT thread.
        // in_control becomes true only after the first control callback, so a
        // stop cannot race ahead of startMotion. command_mutex keeps ownership
        // until a stop in flight has returned and prevents stopping a next move.
        if (cancelled_ && in_control_) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          if (in_control_ && !stop_called_) {
            stop_called_ = true;
            try { robot_.stop(); }
            catch (const std::exception& error) {
              stop_error_ = error.what();
              abort_control_ = true;
              event("stop", command_.id, "failed", stop_error_);
              quitting_ = true;
            }
          }
        }
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        const int ready = eof ? 0 : poll(&descriptor, 1, 10);
        if (ready < 0 && errno != EINTR) throw std::runtime_error("Cannot poll command pipe");
        if (ready > 0 && (descriptor.revents & (POLLIN | POLLHUP | POLLERR))) {
          char buffer[2048];
          const auto count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
          if (count == 0) { eof = true; quitting_ = true; cancelled_ = true; }
          else if (count < 0 && errno != EINTR) throw std::runtime_error("Cannot read command pipe");
          else if (count > 0) {
            pending.append(buffer, static_cast<std::size_t>(count));
            std::size_t end;
            while ((end = pending.find('\n')) != std::string::npos) {
              if (end > 4096) throw std::runtime_error("Command exceeds 4096 bytes");
              accept(pending.substr(0, end));
              pending.erase(0, end + 1);
            }
            if (pending.size() > 4096) throw std::runtime_error("Command exceeds 4096 bytes");
          }
        }
        if (eof) std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    } catch (const std::exception& error) {
      std::cerr << error.what() << '\n';
      quitting_ = true;
      cancelled_ = true;
      abort_control_ = true;
      // Malformed input and OS errors must also stop a live motion.
      if (in_control_) {
        try { robot_.stop(); } catch (...) {}
      }
    }
  }

  bool stationary(const franka::RobotState& state) {
    return state.robot_mode == franka::RobotMode::kIdle &&
           !static_cast<bool>(state.current_errors) &&
           std::all_of(state.dq.begin(), state.dq.end(), [](double v) {
             return std::isfinite(v) && std::abs(v) <= 0.02;
           });
  }

  void execute(const fr3::Command& command) {
    std::string failure;
    std::optional<double> error;
    try {
      if (!cancelled_ && !quitting_ && !interrupted) {
        auto initial = robot_.readOnce();
        sample(initial);
        if (!stationary(initial)) throw std::runtime_error("Robot must be idle and stationary before motion");
        // A start based on desired positions avoids the discontinuity caused by
        // commanding a measured position that differs from the SDK's q_d.
        fr3::PathTrajectory trajectory(initial.q_d, command);
        double elapsed = 0.0;
        bool first = true;
        event("motion", command.id, "running", "", &command.target);
        robot_.control([&](const franka::RobotState& state, franka::Duration period) {
          sample(state);
          if (abort_control_)
            throw std::runtime_error("Control aborted because its command/stop channel failed");
          if (first) {
            first = false;
            // The robot may have changed between readOnce and control startup.
            if (!stationary(state) && state.robot_mode != franka::RobotMode::kMove)
              throw std::runtime_error("Robot changed mode before control startup");
            if (fr3::max_error(state.q_d, initial.q_d) > 0.001)
              throw std::runtime_error("Robot position changed before control startup");
            if (cancelled_ || quitting_ || interrupted)
              return franka::MotionFinished(franka::JointPositions(state.q_d));
          }
          in_control_ = true;
          elapsed += period.toSec();
          if (elapsed > command.timeout) throw std::runtime_error("Motion execution timed out");
          franka::JointPositions positions(trajectory.at(elapsed));
          // Continue the smooth trajectory while the non-RT StopMove request
          // performs the manufacturer's controlled stop. Do not jump to q_d.
          if (elapsed >= trajectory.duration() + 0.25)
            return franka::MotionFinished(positions);
          return positions;
        }, franka::ControllerMode::kJointImpedance, true);
      }
    } catch (const franka::ControlException& exception) {
      if (!cancelled_) failure = exception.what();
      // A user stop normally aborts control with ControlException. Validate the
      // measured idle state below before reporting it as cancelled.
    } catch (const std::exception& exception) {
      failure = exception.what();
    }
    in_control_ = false;
    if (abort_control_ && failure.empty())
      failure = "Control aborted because its command/stop channel failed";
    {
      // Join any StopMove call before reading again or releasing ownership.
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!stop_error_.empty()) failure = "Backend stop failed: " + stop_error_;
    }
    try {
      const auto deadline = Clock::now() + std::chrono::seconds(2);
      bool settled = false;
      do {
        const auto state = robot_.readOnce();
        sample(state);
        error = fr3::max_error(state.q, command.target);
        settled = stationary(state);
        if (settled && (cancelled_ || *error <= command.tolerance)) break;
        if (static_cast<bool>(state.current_errors)) {
          std::ostringstream details;
          details << state.current_errors;
          const std::string errors = "Robot motion errors: " + details.str();
          failure = failure.empty() ? errors : failure + "; " + errors;
          break;
        }
      } while (Clock::now() < deadline);
      if (!settled && failure.empty()) failure = "Robot did not confirm an idle, stationary state";
      if (!cancelled_ && failure.empty() && (!error || *error > command.tolerance))
        failure = "Actual joint angles did not reach target tolerance";
    } catch (const std::exception& exception) { failure = exception.what(); }
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!failure.empty()) event("motion", command.id, "failed", failure, &command.target, error);
    else if (cancelled_) {
      event("motion", command.id, "cancelled", "", &command.target, error);
      event("stop", command.id, "stopped");
    } else event("motion", command.id, "succeeded", "", &command.target, error);
    busy_ = false;
  }

  void finish_threads() {
    quitting_ = true;
    cancelled_ = true;
    input_done_ = true;
    if (input_.joinable()) input_.join();
    output_done_ = true;
    if (output_.joinable()) output_.join();
  }

  franka::Robot& robot_;
  RealtimeStatus rt_;
  Samples samples_;
  Clock::time_point next_sample_{};
  std::atomic<bool> quitting_{false}, pending_{false}, cancelled_{false}, in_control_{false};
  std::atomic<bool> abort_control_{false};
  std::atomic<bool> input_done_{false}, output_done_{false};
  std::atomic<Clock::duration::rep> last_output_{Clock::now().time_since_epoch().count()};
  std::mutex command_mutex_, event_mutex_;
  fr3::Command command_;
  bool busy_ = false, stop_called_ = false;
  std::string stop_error_;
  std::deque<std::string> events_;
  std::thread input_, output_;
};
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: franka-state-reader ROBOT_IP | --check-realtime\n";
    return 2;
  }
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  std::signal(SIGPIPE, SIG_IGN);
  std::cout.imbue(std::locale::classic());
  std::cout << std::setprecision(17);
  try {
    const auto rt = check_realtime();
    if (std::string(argv[1]) == "--check-realtime") {
      std::cout << '{'; write_realtime(std::cout, rt); std::cout << "}\n";
      return rt.ready() ? 0 : 1;
    }
    const auto address = canonical_ip(argv[1]);
    RobotLock lock(address);
    // The SDK's kIgnore supports the host's existing kernel. The calling thread
    // retains the FIFO priority established above; helper threads reset theirs.
    // If FIFO setup failed, the session remains available for state reads only.
    franka::Robot robot(address, !rt.standard_kernel && rt.kernel && rt.scheduling
                                    ? franka::RealtimeConfig::kEnforce
                                    : franka::RealtimeConfig::kIgnore);
    Session session(robot, rt);
    session.run();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
