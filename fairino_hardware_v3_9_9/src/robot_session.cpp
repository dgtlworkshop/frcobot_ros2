#include "fairino_hardware/robot_session.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

namespace fairino_hardware
{
namespace
{
constexpr double pi = 3.14159265358979323846;
const auto logger = rclcpp::get_logger("fairino_session");

class SdkBackend : public RobotBackend
{
public:
  ~SdkBackend() override
  {
    if (robot_) {
      const int result = robot_->CloseRPC();
      if (result != 0) {
        RCLCPP_ERROR(logger, "CloseRPC failed: %d", result);
      }
      robot_.reset();
    }
    if (lock_fd_ >= 0) {
      close(lock_fd_);
    }
  }

  int connect(const std::string & ip, const std::vector<RobotState> & states) override
  {
    const auto path = "/tmp/fairino-sdk-" + std::to_string(getuid()) + ".lock";
    lock_fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (lock_fd_ < 0 || flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      throw std::runtime_error(
        "Another Fairino SDK owner is running (or SDK lock cannot be opened). "
        "Use the command service embedded in ros2_control, not a second ros2_cmd_server.");
    }
    robot_ = std::make_unique<FRRobot>();
    int result = robot_->SetReConnectParam(true, 86400000, 500);
    if (result == 0) {
      result = robot_->RPC(ip.c_str());
    }
    if (result == 0) {
      result = robot_->SetRobotRealtimeStateConfig(states, 8);
    }
    return result;
  }

  int receive(ROBOT_STATE_PKG & state) override
  {
    int communication = -1;
    const int result = robot_->GetSDKComState(&communication);
    if (result != 0 || communication != 0) {
      return result != 0 ? result : communication;
    }
    return robot_->GetRobotRealTimeState(&state);
  }
  int start_servo() override { return robot_->ServoMoveStart(); }
  int send(const std::array<double, 6> & degrees, const std::array<double, 4> & external_positions)
    override
  {
    JointPos target{};
    ExaxisPos external{};
    std::copy(degrees.begin(), degrees.end(), target.jPos);
    std::copy(external_positions.begin(), external_positions.end(), external.ePos);
    // UDP avoids an XML-RPC round trip for every 8 ms setpoint.
    return robot_->ServoJ(&target, &external, 0, 0, 0.008, 0, 0, 0, 1);
  }
  int stop() override
  {
    const int motion = robot_->StopMotion();
    const int servo = robot_->ServoMoveEnd();
    if (motion != 0) {
      RCLCPP_ERROR(logger, "StopMotion failed: %d", motion);
    }
    if (servo != 0) {
      RCLCPP_ERROR(logger, "ServoMoveEnd failed: %d", servo);
    }
    return motion != 0 ? motion : servo;
  }
  FRRobot * sdk() override { return robot_.get(); }

private:
  int lock_fd_ = -1;
  std::unique_ptr<FRRobot> robot_;
};

bool same_time(const RobotTime & a, const RobotTime & b)
{
  return a.year == b.year && a.mouth == b.mouth && a.day == b.day && a.hour == b.hour &&
         a.minute == b.minute && a.second == b.second && a.millisecond == b.millisecond;
}
}  // namespace

std::unique_ptr<RobotBackend> make_sdk_backend() { return std::make_unique<SdkBackend>(); }

RobotSession::RobotSession(
  std::unique_ptr<RobotBackend> backend, std::chrono::milliseconds feedback_timeout,
  std::chrono::milliseconds command_timeout)
: backend_(std::move(backend)),
  feedback_timeout_(feedback_timeout),
  command_timeout_(command_timeout)
{
  if (!backend_ || feedback_timeout_.count() <= 0 || command_timeout_.count() <= 0) {
    throw std::invalid_argument("RobotSession requires a backend and positive timeouts");
  }
}

RobotSession::~RobotSession()
{
  running_ = false;
  if (worker_.joinable()) {
    worker_.join();
  }
  release();
}

void RobotSession::connect(const std::string & ip, const std::vector<RobotState> & states)
{
  const int result = backend_->connect(ip, states);
  std::lock_guard<std::mutex> guard(mutex_);
  configured_ = result == 0;
  if (result != 0) {
    latch_locked("SDK connection/feedback configuration failed: " + std::to_string(result));
  }
}

void RobotSession::start_monitor()
{
  if (running_.exchange(true)) {
    return;
  }
  worker_ = std::thread([this]() {
    while (running_) {
      const auto next = Clock::now() + std::chrono::milliseconds(8);
      poll_once();
      std::this_thread::sleep_until(next);
    }
  });
}

std::string RobotSession::readiness_error(const ROBOT_STATE_PKG & s)
{
  if (s.main_code != 0 || s.sub_code != 0) {
    return "Robot fault " + std::to_string(s.main_code) + "/" + std::to_string(s.sub_code);
  }
  if (
    s.EmergencyStop || s.safety_stop0_state || s.safety_stop1_state || s.btnBoxStopSignal ||
    s.collisionState || s.safetyDoorAlarm || s.safetyPlaneAlarm || s.safetyDataState ||
    s.alarmCheckEmergStopBtn || s.ctrlBoxError || s.driverError || s.driverAlarm ||
    s.aliveSlaveNumError || s.cmdPointError || s.outSoftLimitError || s.abnomalStop ||
    s.motonAlarm || s.interfaceAlarm || s.alarmRebootRobot || s.alarm ||
    std::any_of(
      std::begin(s.driverComError), std::end(s.driverComError),
      [](auto value) { return value != 0; }) ||
    std::any_of(std::begin(s.slaveComError), std::end(s.slaveComError), [](auto value) {
      return value != 0;
    })) {
    return "Robot safety stop or controller/drive alarm";
  }
  std::string readiness;
  const auto add_condition = [&readiness](const std::string & condition) {
    if (!readiness.empty()) {
      readiness += "; ";
    }
    readiness += condition;
  };
  if (s.rbtEnableState != 1) {
    add_condition("rbtEnableState=" + std::to_string(s.rbtEnableState) + " (expected 1: enabled)");
  }
  if (s.robot_mode != 0) {
    add_condition("robot_mode=" + std::to_string(s.robot_mode) + " (expected 0: automatic)");
  }
  if (s.robot_state != 1 && s.robot_state != 2) {
    add_condition("robot_state=" + std::to_string(s.robot_state) +
      " (expected 1: stopped or 2: running; 3: paused, 4: drag)");
  }
  if (s.programRunState != 0 && s.programRunState != 200) {
    add_condition("programRunState=" + std::to_string(s.programRunState) +
      " (expected 0: no LUA program running; 200: allowed unknown value)");
  }
  if (!readiness.empty()) {
    return "Robot not ready: " + readiness;
  }
  for (size_t i = 0; i < 6; ++i) {
    if (
      !std::isfinite(s.jt_cur_pos[i]) || !std::isfinite(s.actual_qd[i]) ||
      !std::isfinite(s.jt_cur_tor[i])) {
      return "Non-finite joint feedback";
    }
  }
  for (const auto & axis : s.extAxisStatus) {
    if (!std::isfinite(axis.pos) || axis.errorCode != 0 || axis.alarm != 0) {
      return "External axis feedback invalid or faulted";
    }
  }
  return {};
}

RobotSnapshot RobotSession::snapshot_locked(Clock::time_point now) const
{
  auto result = status_;
  result.feedback_age = status_.valid ? std::chrono::duration<double>(now - feedback_time_).count()
                                      : std::numeric_limits<double>::infinity();
  result.fresh = status_.connected && status_.valid && now - feedback_time_ < feedback_timeout_;
  result.motion_allowed =
    configured_ && result.fresh && !result.fault_latched && readiness_error(result.state).empty();
  if (!result.fresh && !result.fault_latched) {
    result.reason = "Robot feedback missing or stale";
  }
  return result;
}

RobotSnapshot RobotSession::snapshot() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return snapshot_locked(Clock::now());
}

void RobotSession::latch_locked(const std::string & reason)
{
  const bool first_fault = !status_.fault_latched;
  if (first_fault) {
    status_.reason = reason;
    RCLCPP_ERROR(logger, "Motion inhibited: %s", reason.c_str());
  }
  status_.fault_latched = true;
  command_pending_ = false;
  if (first_fault && status_.claimed) {
    stop_pending_ = true;
  }
}

void RobotSession::fault(const std::string & reason)
{
  std::lock_guard<std::mutex> guard(mutex_);
  latch_locked(reason);
}

void RobotSession::poll_once()
{
  // A service must never block the controller-manager thread.
  std::unique_lock<std::mutex> sdk_guard(sdk_mutex_, std::try_to_lock);
  if (!sdk_guard.owns_lock()) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!snapshot_locked(Clock::now()).fresh) {
      latch_locked("Feedback worker blocked by SDK service");
    }
    return;
  }
  ROBOT_STATE_PKG state{};
  const int result = backend_->receive(state);
  bool stop = false;
  bool send = false;
  std::array<double, 6> target{};
  std::array<double, 4> external{};
  {
    std::lock_guard<std::mutex> guard(mutex_);
    const auto now = Clock::now();
    status_.connected = result == 0;
    if (result != 0) {
      latch_locked("SDK feedback/connection failure: " + std::to_string(result));
    } else {
      if (state.programRunState == 200 &&
        (!status_.valid || status_.state.programRunState != 200))
      {
        RCLCPP_WARN(logger,
          "programRunState=200 is unknown in this SDK; allowed by explicit compatibility "
          "exception, not confirmation that no LUA program is running");
      }
      const bool changed = state.robotTime.year != 0 &&
                           (!status_.valid || !same_time(state.robotTime, status_.state.robotTime));
      status_.state = state;
      if (changed) {
        status_.valid = true;
        feedback_time_ = now;
        status_.timestamp =
          static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count());
      }
      if (!snapshot_locked(now).fresh) {
        latch_locked("Robot feedback timestamp is missing or stale");
      } else {
        const auto error = readiness_error(state);
        if (!error.empty()) {
          latch_locked(error);
        } else if (!status_.fault_latched) {
          status_.reason = "Ready";
        }
      }
    }
    if (status_.claimed && now - command_time_ >= command_timeout_) {
      latch_locked("Controller command watchdog expired");
    }
    stop = stop_pending_;
    stop_pending_ = false;
    send = status_.claimed && snapshot_locked(now).motion_allowed && command_pending_;
    if (send) {
      target = command_degrees_;
      external = external_positions_;
      command_pending_ = false;
    }
  }
  if (stop) {
    const int code = backend_->stop();
    if (code != 0) {
      RCLCPP_ERROR(logger, "Best-effort robot stop failed: %d", code);
    }
  } else if (send) {
    const int code = backend_->send(target, external);
    if (code != 0) {
      fault("ServoJ failed: " + std::to_string(code));
    }
  }
}

bool RobotSession::claim()
{
  std::lock_guard<std::mutex> sdk_guard(sdk_mutex_);
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (
      status_.claimed || !snapshot_locked(Clock::now()).motion_allowed ||
      status_.state.robot_state != 1) {
      RCLCPP_ERROR(logger, "Cannot claim servo control: %s", status_.reason.c_str());
      return false;
    }
  }
  const int code = backend_->start_servo();
  std::lock_guard<std::mutex> guard(mutex_);
  if (code != 0 || !snapshot_locked(Clock::now()).motion_allowed) {
    latch_locked("ServoMoveStart failed or feedback expired: " + std::to_string(code));
    stop_pending_ = true;
    return false;
  }
  status_.claimed = true;
  command_time_ = Clock::now();
  // Never replay a target from a previous controller activation.
  std::copy(
    std::begin(status_.state.jt_cur_pos), std::end(status_.state.jt_cur_pos),
    command_degrees_.begin());
  for (size_t i = 0; i < external_positions_.size(); ++i) {
    external_positions_[i] = status_.state.extAxisStatus[i].pos;
  }
  command_pending_ = false;
  return true;
}

bool RobotSession::release()
{
  std::lock_guard<std::mutex> sdk_guard(sdk_mutex_);
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!status_.claimed && !stop_pending_) {
      return true;
    }
    status_.claimed = false;
    command_pending_ = false;
    stop_pending_ = false;
  }
  const int result = backend_->stop();
  if (result != 0) {
    fault("Stopping servo control failed: " + std::to_string(result));
  }
  return result == 0;
}

bool RobotSession::stop_motion()
{
  std::lock_guard<std::mutex> sdk_guard(sdk_mutex_);
  {
    std::lock_guard<std::mutex> guard(mutex_);
    latch_locked("StopMotion requested through command service");
    stop_pending_ = false;
  }
  const int code = backend_->stop();
  if (code != 0) {
    RCLCPP_ERROR(logger, "Requested StopMotion/ServoMoveEnd failed: %d", code);
  }
  return code == 0;
}

bool RobotSession::recover(std::string & reason)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (!configured_) {
    reason = "SDK setup failed; clean up and configure the hardware again";
    return false;
  }
  const auto current = snapshot_locked(Clock::now());
  if (current.claimed || stop_pending_ || !current.fresh) {
    reason = "Deactivate the trajectory controller and wait for fresh connected feedback";
    return false;
  }
  reason = readiness_error(current.state);
  if (!reason.empty()) {
    return false;
  }
  status_.fault_latched = false;
  status_.reason = reason = "Recovered; activate the controller and submit a NEW trajectory";
  return true;
}

void RobotSession::command(const std::array<double, 6> & radians)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (!status_.claimed) {
    return;
  }
  if (!std::all_of(radians.begin(), radians.end(), [](double x) { return std::isfinite(x); })) {
    latch_locked("Non-finite position command");
    return;
  }
  if (!snapshot_locked(Clock::now()).motion_allowed) {
    latch_locked("Motion authority lost before write");
    return;
  }
  std::transform(radians.begin(), radians.end(), command_degrees_.begin(), [](double value) {
    return value * 180.0 / pi;
  });
  if (!std::all_of(command_degrees_.begin(), command_degrees_.end(), [](double value) {
        return std::isfinite(value);
      })) {
    latch_locked("Position command conversion overflow");
    return;
  }
  command_time_ = Clock::now();
  command_pending_ = true;
}
}  // namespace fairino_hardware
