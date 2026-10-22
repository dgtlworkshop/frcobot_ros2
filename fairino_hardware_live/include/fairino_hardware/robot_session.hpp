#ifndef FAIRINO_HARDWARE_ROBOT_SESSION_HPP_
#define FAIRINO_HARDWARE_ROBOT_SESSION_HPP_

#include <robot.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace fairino_hardware
{

class RobotBackend
{
public:
  virtual ~RobotBackend() = default;
  virtual int connect(const std::string & ip, const std::vector<RobotState> & states) = 0;
  virtual int receive(ROBOT_STATE_PKG & state) = 0;
  virtual int start_servo() = 0;
  virtual int send(
    const std::array<double, 6> & degrees, const std::array<double, 4> & external) = 0;
  virtual int stop() = 0;
  virtual FRRobot * sdk() { return nullptr; }
};

std::unique_ptr<RobotBackend> make_sdk_backend();

struct RobotSnapshot
{
  ROBOT_STATE_PKG state{};
  bool valid = false;
  bool connected = false;
  bool fresh = false;
  bool fault_latched = false;
  bool claimed = false;
  bool motion_allowed = false;
  double feedback_age = 0.0;
  uint64_t timestamp = 0;
  std::string reason = "Waiting for robot feedback";
};

class RobotSession
{
public:
  using Clock = std::chrono::steady_clock;
  explicit RobotSession(
    std::unique_ptr<RobotBackend> backend = make_sdk_backend(),
    std::chrono::milliseconds feedback_timeout = std::chrono::milliseconds(100),
    std::chrono::milliseconds command_timeout = std::chrono::milliseconds(100));
  ~RobotSession();

  void connect(const std::string & ip, const std::vector<RobotState> & states);
  void start_monitor();
  RobotSnapshot snapshot() const;
  bool claim();
  bool release();
  bool stop_motion();
  bool recover(std::string & reason);
  void command(const std::array<double, 6> & radians);
  void fault(const std::string & reason);
  void poll_once();

  // All legacy service calls share this lock with the communication worker.
  std::mutex & sdk_mutex() { return sdk_mutex_; }
  FRRobot * sdk() { return backend_->sdk(); }

private:
  RobotSnapshot snapshot_locked(Clock::time_point now) const;
  void latch_locked(const std::string & reason);
  static std::string readiness_error(const ROBOT_STATE_PKG & state);

  std::unique_ptr<RobotBackend> backend_;
  const std::chrono::milliseconds feedback_timeout_;
  const std::chrono::milliseconds command_timeout_;
  mutable std::mutex mutex_;
  std::mutex sdk_mutex_;
  RobotSnapshot status_;
  Clock::time_point feedback_time_{};
  Clock::time_point command_time_{};
  std::array<double, 6> command_degrees_{};
  std::array<double, 4> external_positions_{};
  bool command_pending_ = false;
  bool stop_pending_ = false;
  bool configured_ = false;
  std::atomic<bool> running_{false};
  std::thread worker_;
};
}  // namespace fairino_hardware
#endif
