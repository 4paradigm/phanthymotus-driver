// Independent Franka Hand connection; does not acquire the arm's FCI connection.
// stdin: MOVE id speed width timeout | GRASP id speed width force inner outer timeout
//        HOME id timeout | STOP id | QUIT. Distances are metres, speeds m/s, forces N.
// stdout: newline-delimited state JSON and the arm session's motion/stop events.
#include <franka/gripper.h>
#include <franka/gripper_state.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <locale>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>

namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void handle_signal(int) { interrupted = 1; }

std::string json_quoted(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(c);
    else out << c;
  }
  return out.str() + '"';
}

bool valid_id(const std::string& id) {
  return !id.empty() && id.size() <= 64 &&
         std::all_of(id.begin(), id.end(), [](unsigned char c) {
           return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-';
         });
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

class GripperLock {
 public:
  explicit GripperLock(const std::string& address) {
    const char* configured = std::getenv("FRANKA_LOCK_DIR");
    const std::string directory = configured ? configured : "/tmp/phanthymotus-franka";
    if (directory.empty() || directory.front() != '/')
      throw std::invalid_argument("FRANKA_LOCK_DIR must be an absolute directory");
    if (mkdir(directory.c_str(), 0755) != 0 && errno != EEXIST)
      throw std::runtime_error("Cannot create Franka lock directory");
    const auto path = directory + "/franka-gripper-" + address + ".lock";
    fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd_ < 0) throw std::runtime_error("Cannot open gripper connection lock: " + path);
    if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("Another Franka gripper process owns " + address);
    }
  }
  ~GripperLock() { if (fd_ >= 0) close(fd_); }
 private:
  int fd_ = -1;
};

struct Command {
  std::string action, id;
  double speed = 0, width = 0, force = 0, inner = 0, outer = 0, timeout = 0;
};

Command parse(const std::string& line) {
  std::istringstream input(line);
  input.imbue(std::locale::classic());
  Command result;
  input >> result.action;
  if (result.action != "QUIT" && (!(input >> result.id) || !valid_id(result.id)))
    throw std::invalid_argument("Invalid command ID");
  if (result.action == "MOVE" || result.action == "GRASP") {
    if (!(input >> result.speed >> result.width))
      throw std::invalid_argument("Motion requires speed and width in metres");
    if (!std::isfinite(result.speed) || result.speed <= 0 || result.speed > 0.1)
      throw std::invalid_argument("Speed must be positive and at most 0.1 m/s");
    if (!std::isfinite(result.width) || result.width < 0 || result.width > 0.08)
      throw std::invalid_argument("Width must be 0 to 0.08 metres");
    if (result.action == "GRASP") {
      if (!(input >> result.force >> result.inner >> result.outer))
        throw std::invalid_argument("GRASP requires force and inner/outer tolerances");
      if (!std::isfinite(result.force) || result.force < 30 || result.force > 70)
        throw std::invalid_argument("Force must be 30 to 70 newtons");
      if (!std::isfinite(result.inner) || !std::isfinite(result.outer) ||
          result.inner < 0 || result.outer < 0 || result.inner > 0.08 || result.outer > 0.08)
        throw std::invalid_argument("Grasp tolerances must be 0 to 0.08 metres");
    }
  } else if (result.action != "HOME" && result.action != "STOP" && result.action != "QUIT") {
    throw std::invalid_argument("Unknown command");
  }
  if (result.action != "STOP" && result.action != "QUIT") {
    if (!(input >> result.timeout) || !std::isfinite(result.timeout) ||
        result.timeout < 1 || result.timeout > 120)
      throw std::invalid_argument("Timeout must be 1 to 120 seconds");
  }
  std::string extra;
  if (input >> extra) throw std::invalid_argument("Unexpected trailing command data");
  return result;
}

struct Sample {
  franka::GripperState state{};
  Clock::time_point started{}, received{};
};

class Session {
 public:
  explicit Session(franka::Gripper& gripper) : gripper_(gripper) {}

  void run() {
    // A blocked consumer must never prevent the supervisor from stopping motion.
    const int flags = fcntl(STDOUT_FILENO, F_GETFL, 0);
    if (flags < 0 || fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK) < 0)
      throw std::runtime_error("Cannot configure gripper state pipe");
    reader_ = std::thread([this] { read_loop(); });
    const auto reading_started = Clock::now();
    while (!closing_ || active_) {
      if (interrupted) close_session();
      if (!closing_) read_commands();
      else std::this_thread::sleep_for(std::chrono::milliseconds(10));
      const auto snapshot = sample();
      std::string read_error;
      {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        read_error = read_error_;
      }
      if (!read_error.empty() && fatal_.empty()) fail_session(read_error);
      const auto last_read = snapshot ? snapshot->received : reading_started;
      if (Clock::now() - last_read > std::chrono::seconds(1))
        fail_session("Fresh gripper state was not received within one second");
      supervise(snapshot);
      if (snapshot && fatal_.empty() && Clock::now() >= next_sample_ &&
          Clock::now() - snapshot->received <= std::chrono::seconds(1)) {
        publish(*snapshot);
        next_sample_ = Clock::now() + std::chrono::milliseconds(33);
      }
      flush();
      if (active_ && Clock::now() - last_output_ > std::chrono::seconds(1))
        fail_session("Parent gripper state pipe is not being consumed");
    }
    reader_done_ = true;
    if (reader_.joinable()) reader_.join();
    if (worker_.joinable()) worker_.join();
    const auto deadline = Clock::now() + std::chrono::milliseconds(100);
    while (!output_.empty() && !output_broken_ && Clock::now() < deadline) {
      flush();
      if (!output_.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!fatal_.empty()) throw std::runtime_error(fatal_);
  }

 private:
  void close_session() { closing_ = true; cancelled_ = true; }
  void fail_session(const std::string& message) {
    if (fatal_.empty()) fatal_ = message;
    close_session();
  }

  void queue(const std::string& line) {
    if (output_broken_) return;
    if (output_.size() + line.size() > 1024 * 1024) {
      fail_session("Gripper event pipe backlog exceeded");
      return;
    }
    output_ += line + '\n';
  }

  void event(const std::string& kind, const std::string& id, const std::string& status,
             const std::string& error = "") {
    std::string value = "{\"event\":" + json_quoted(kind) + ",\"command_id\":" + json_quoted(id) +
                        ",\"status\":" + json_quoted(status);
    if (!error.empty()) value += ",\"error\":" + json_quoted(error.substr(0, 2048));
    queue(value + '}');
  }

  void flush() {
    while (!output_.empty() && !output_broken_) {
      const auto count = ::write(STDOUT_FILENO, output_.data(), output_.size());
      if (count > 0) {
        output_.erase(0, static_cast<std::size_t>(count));
        last_output_ = Clock::now();
      } else if (count < 0 && errno == EINTR) {
        continue;
      } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      } else {
        output_broken_ = true;
        output_.clear();
        fail_session("Parent gripper state pipe closed");
      }
    }
  }

  void publish(const Sample& value) {
    if (output_.size() > 65536) return;  // Drop telemetry, never motion events.
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << "{\"type\":\"state\",\"width\":" << value.state.width
        << ",\"max_width\":" << value.state.max_width
        << ",\"is_grasped\":" << (value.state.is_grasped ? "true" : "false")
        << ",\"temperature\":" << value.state.temperature
        << ",\"control_ready\":" << (fatal_.empty() && !closing_ ? "true" : "false")
        << ",\"control_error\":" << (fatal_.empty() ? "null" : json_quoted(fatal_)) << '}';
    queue(out.str());
  }

  std::optional<Sample> sample() {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    return sample_;
  }

  void read_loop() {
    try {
      while (!reader_done_) {
        Sample value;
        value.started = Clock::now();
        // This is the sole readOnce caller, including final-state verification.
        value.state = gripper_.readOnce();
        value.received = Clock::now();
        // Match the Python reader: closed fingers can report a signed opening.
        if (!std::isfinite(value.state.width) || !std::isfinite(value.state.max_width) ||
            value.state.max_width < 0)
          throw std::runtime_error("Invalid gripper width in backend state");
        {
          std::lock_guard<std::mutex> lock(sample_mutex_);
          sample_ = value;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> lock(sample_mutex_);
      read_error_ = error.what();
    }
  }

  void accept(const std::string& line) {
    Command incoming;
    try { incoming = parse(line); }
    catch (const std::exception& error) {
      std::istringstream input(line);
      std::string action, id;
      input >> action >> id;
      event(action == "STOP" ? "stop" : "motion", valid_id(id) ? id : "", "rejected", error.what());
      return;
    }
    if (incoming.action == "QUIT") { close_session(); return; }
    if (incoming.action == "STOP") {
      if (!active_) {
        // A completed grasp still applies force. Explicit stopmotion must reach
        // the backend even with no active asynchronous operation to cancel.
        try {
          if (gripper_.stop()) event("stop", incoming.id, "stopped");
          else event("stop", incoming.id, "failed", "Backend stop returned false");
        } catch (const std::exception& error) { event("stop", incoming.id, "failed", error.what()); }
      } else if (incoming.id != command_.id) event("stop", incoming.id, "idle");
      else { cancelled_ = true; stop_requested_ = true; }
      return;
    }
    if (closing_ || active_ || seen_.count(incoming.id)) {
      event("motion", incoming.id, "rejected", closing_ ? "Session is stopping" :
            (active_ ? "Gripper already has an active motion" : "Command ID has already been used"));
      return;
    }
    std::optional<Sample> current;
    std::string read_error;
    {
      std::lock_guard<std::mutex> lock(sample_mutex_);
      current = sample_;
      read_error = read_error_;
    }
    if (!read_error.empty()) {
      event("motion", incoming.id, "rejected", read_error);
      return;
    }
    if (!current || Clock::now() - current->received > std::chrono::seconds(1)) {
      event("motion", incoming.id, "rejected", "Fresh gripper state is required");
      return;
    }
    if (incoming.action != "HOME" && (current->state.max_width <= 0 ||
                                     incoming.width > current->state.max_width)) {
      event("motion", incoming.id, "rejected", current->state.max_width <= 0 ?
            "Gripper requires explicit homing before setting width" : "Width exceeds calibrated maximum");
      return;
    }
    // Keep all accepted IDs for this process lifetime; never silently replay one.
    if (seen_.size() >= 100000) {
      event("motion", incoming.id, "rejected", "Command history exhausted; restart gripper session");
      return;
    }
    seen_.insert(incoming.id);
    command_ = incoming;
    active_ = true;
    cancelled_ = false;
    worker_done_ = false;
    worker_called_ = false;
    worker_success_ = false;
    worker_error_.clear();
    stop_error_.clear();
    stop_ok_ = stop_requested_ = final_stop_done_ = false;
    timed_out_ = false;
    finishing_ = false;
    next_stop_ = Clock::time_point{};
    last_stop_ = Clock::time_point{};
    begun_ = Clock::now();
    event("motion", incoming.id, "accepted");
    event("motion", incoming.id, "running");
    worker_ = std::thread([this, incoming] {
      try {
        if (!cancelled_) {
          worker_called_ = true;
          if (incoming.action == "MOVE")
            worker_success_ = gripper_.move(incoming.width, incoming.speed);
          else if (incoming.action == "GRASP")
            worker_success_ = gripper_.grasp(incoming.width, incoming.speed, incoming.force,
                                            incoming.inner, incoming.outer);
          else worker_success_ = gripper_.homing();
        }
      } catch (const std::exception& error) { worker_error_ = error.what(); }
      worker_finished_ = Clock::now();
      worker_done_.store(true, std::memory_order_release);
    });
  }

  void read_commands() {
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};
    const int ready = poll(&descriptor, 1, 10);
    if (ready < 0) {
      if (errno != EINTR) fail_session("Cannot poll gripper command pipe");
      return;
    }
    if (ready == 0) return;
    if (descriptor.revents & POLLNVAL) { fail_session("Invalid gripper command pipe"); return; }
    if (!(descriptor.revents & (POLLIN | POLLHUP | POLLERR))) return;
    char buffer[2048];
    const auto count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    if (count == 0) { close_session(); return; }
    if (count < 0) {
      if (errno != EINTR) fail_session("Cannot read gripper command pipe");
      return;
    }
    pending_.append(buffer, static_cast<std::size_t>(count));
    std::size_t end;
    while ((end = pending_.find('\n')) != std::string::npos) {
      if (end > 4096) { fail_session("Command exceeds 4096 bytes"); return; }
      accept(pending_.substr(0, end));
      pending_.erase(0, end + 1);
      if (closing_) return;
    }
    if (pending_.size() > 4096) fail_session("Command exceeds 4096 bytes");
  }

  void supervise(const std::optional<Sample>& current) {
    if (!active_) return;
    const auto now = Clock::now();
    const bool done = worker_done_.load(std::memory_order_acquire);
    if (std::chrono::duration<double>((done ? worker_finished_ : now) - begun_).count() >
        command_.timeout) {
      timed_out_ = true;
      cancelled_ = true;
    }
    // stop() may arrive just before the SDK starts move/grasp/homing. Repeat
    // while the worker is alive, retaining ownership so it cannot stop a new job.
    if (cancelled_ && ((!done && now >= next_stop_) ||
                      (done && !final_stop_done_ && (worker_called_ || stop_requested_)))) {
      try {
        if (gripper_.stop()) stop_ok_ = true;
        else stop_error_ = "Backend stop returned false";
      } catch (const std::exception& error) { stop_error_ = error.what(); }
      last_stop_ = Clock::now();
      next_stop_ = Clock::now() + std::chrono::milliseconds(50);
      if (done) final_stop_done_ = true;
    }
    if (!worker_done_.load(std::memory_order_acquire)) return;
    if (!finishing_) {
      worker_.join();
      finishing_ = true;
      final_deadline_ = Clock::now() + std::chrono::seconds(2);
    }
    std::string failure;
    if (!fatal_.empty()) failure = fatal_;
    else if (!stop_error_.empty()) failure = "Backend stop failed: " + stop_error_;
    else if (timed_out_) failure = "Gripper motion execution timed out";
    else if (!worker_error_.empty()) failure = worker_error_;
    else if (!cancelled_ && !worker_success_) failure = "Gripper command returned false";
    // A sample whose read started before the worker completed can be stale.
    const bool fresh = current && current->started > std::max(worker_finished_, last_stop_);
    bool confirmed = fresh;
    if (fresh && !cancelled_) {
      if (command_.action == "HOME") confirmed = current->state.max_width > 0;
      else if (command_.action == "MOVE")
        confirmed = std::abs(current->state.width - command_.width) <= 0.001;
      else confirmed = current->state.is_grasped &&
                       current->state.width >= command_.width - command_.inner &&
                       current->state.width <= command_.width + command_.outer;
    }
    if (!confirmed && Clock::now() < final_deadline_ && fatal_.empty()) return;
    if (!confirmed && failure.empty()) failure = "Gripper did not confirm the final state";
    if (cancelled_ && worker_called_ && !stop_ok_ && !worker_success_ && failure.empty())
      failure = "Gripper cancellation was not confirmed by the backend";
    if (!failure.empty()) {
      event("motion", command_.id, "failed", failure);
      if (stop_requested_) event("stop", command_.id, "failed", failure);
    } else if (cancelled_) {
      event("motion", command_.id, "cancelled");
      if (stop_requested_) event("stop", command_.id, "stopped");
    } else event("motion", command_.id, "succeeded");
    active_ = false;
  }

  franka::Gripper& gripper_;
  std::mutex sample_mutex_;
  std::optional<Sample> sample_;
  std::string read_error_;
  std::atomic<bool> reader_done_{false}, cancelled_{false}, worker_done_{false};
  std::thread reader_, worker_;
  Command command_;
  std::unordered_set<std::string> seen_;
  bool active_ = false, closing_ = false, output_broken_ = false, finishing_ = false;
  bool worker_called_ = false, worker_success_ = false, timed_out_ = false;
  bool stop_ok_ = false, stop_requested_ = false, final_stop_done_ = false;
  std::string pending_, output_, fatal_, worker_error_, stop_error_;
  Clock::time_point begun_{}, worker_finished_{}, next_stop_{}, final_deadline_{}, next_sample_{};
  Clock::time_point last_stop_{};
  Clock::time_point last_output_ = Clock::now();
};
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "Usage: franka-gripper ROBOT_IP\n"; return 2; }
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  std::signal(SIGPIPE, SIG_IGN);
  try {
    const auto address = canonical_ip(argv[1]);
    GripperLock lock(address);
    franka::Gripper gripper(address);
    Session session(gripper);
    session.run();
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  return 0;
}
