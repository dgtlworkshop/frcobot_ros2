#include "fairino_hardware/fairino_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "fairino_hardware/command_server.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace fairino_hardware
{
namespace
{
const auto logger = rclcpp::get_logger("FairinoHardwareInterface");
constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
}  // namespace

FairinoHardwareInterface::~FairinoHardwareInterface() { close_session(); }

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(logger, "Exactly six joints in SDK axis order are required");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces.front().name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(logger, "Joint %s requires one position command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::set<std::string> states;
    for (const auto & state : joint.state_interfaces) {
      if (
        (state.name != hardware_interface::HW_IF_POSITION &&
         state.name != hardware_interface::HW_IF_VELOCITY &&
         state.name != hardware_interface::HW_IF_EFFORT) ||
        !states.insert(state.name).second) {
        RCLCPP_ERROR(logger, "Unsupported or duplicate state interface on %s", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (!states.count(hardware_interface::HW_IF_POSITION)) {
      RCLCPP_ERROR(logger, "Joint %s requires measured position state", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  const auto parameter = [this](const std::string & name, const std::string & fallback) {
    const auto found = info_.hardware_parameters.find(name);
    return found == info_.hardware_parameters.end() ? fallback : found->second;
  };
  robot_ip_ = parameter("robot_ip", robot_ip_);
  state_prefix_ = parameter("state_prefix", state_prefix_);
  try {
    const auto timeout = [&parameter](const std::string & name) {
      const auto text = parameter(name, "100");
      size_t consumed = 0;
      const int value = std::stoi(text, &consumed);
      if (consumed != text.size() || value < 16 || value > 1000) {
        throw std::invalid_argument(name + " must be an integer in [16, 1000] ms");
      }
      return std::chrono::milliseconds(value);
    };
    feedback_timeout_ = timeout("feedback_timeout_ms");
    command_timeout_ = timeout("command_timeout_ms");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Invalid hardware parameter: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (robot_ip_.empty() || state_prefix_.empty()) {
    RCLCPP_ERROR(logger, "robot_ip and state_prefix must not be empty");
    return hardware_interface::CallbackReturn::ERROR;
  }
  positions_.fill(std::numeric_limits<double>::quiet_NaN());
  velocities_.fill(std::numeric_limits<double>::quiet_NaN());
  efforts_.fill(std::numeric_limits<double>::quiet_NaN());
  commands_.fill(std::numeric_limits<double>::quiet_NaN());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (session_) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  try {
    session_ =
      std::make_shared<RobotSession>(create_backend(), feedback_timeout_, command_timeout_);
    node_ = std::make_shared<robot_command_thread>("fairino_hardware", session_, robot_ip_);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_running_ = true;
    executor_thread_ = std::thread([this]() {
      while (executor_running_ && rclcpp::ok()) {
        executor_->spin_once(std::chrono::milliseconds(10));
      }
    });
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Failed to configure Fairino: %s", error.what());
    close_session();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void FairinoHardwareInterface::update_state()
{
  const auto status = session_->snapshot();
  motion_allowed_ = active_ && status.motion_allowed ? 1.0 : 0.0;
  connected_ = status.connected ? 1.0 : 0.0;
  fault_latched_ = status.fault_latched ? 1.0 : 0.0;
  feedback_age_ = status.feedback_age;
  main_error_ = status.state.main_code;
  sub_error_ = status.state.sub_code;
  if (status.valid && status.fresh) {
    for (size_t i = 0; i < 6; ++i) {
      positions_[i] = status.state.jt_cur_pos[i] * degrees_to_radians;
      velocities_[i] = status.state.actual_qd[i] * degrees_to_radians;
      efforts_[i] = status.state.jt_cur_tor[i];
    }
  }
}

void FairinoHardwareInterface::seed_commands()
{
  update_state();
  commands_ = positions_;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!session_ || !session_->snapshot().motion_allowed) {
    RCLCPP_ERROR(logger, "Activation refused: fresh ready feedback and explicit recovery required");
    return hardware_interface::CallbackReturn::FAILURE;
  }
  active_ = true;
  seed_commands();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  motion_allowed_ = 0.0;
  return !session_ || session_->release() ? hardware_interface::CallbackReturn::SUCCESS
                                          : hardware_interface::CallbackReturn::ERROR;
}

void FairinoHardwareInterface::close_session()
{
  active_ = false;
  motion_allowed_ = 0.0;
  executor_running_ = false;
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  executor_.reset();
  node_.reset();
  session_.reset();
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  close_session();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  close_session();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  motion_allowed_ = 0.0;
  if (session_) {
    session_->fault("Hardware lifecycle error");
    if (!session_->release()) {
      RCLCPP_ERROR(logger, "Failed to stop servo control during lifecycle error");
    }
  }
  // The monitoring node deliberately survives lifecycle errors.
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < 6; ++i) {
    for (const auto & state : info_.joints[i].state_interfaces) {
      double * value = state.name == hardware_interface::HW_IF_POSITION   ? &positions_[i]
                       : state.name == hardware_interface::HW_IF_VELOCITY ? &velocities_[i]
                                                                          : &efforts_[i];
      interfaces.emplace_back(info_.joints[i].name, state.name, value);
    }
  }
  interfaces.emplace_back(state_prefix_, "motion_allowed", &motion_allowed_);
  interfaces.emplace_back(state_prefix_, "connected", &connected_);
  interfaces.emplace_back(state_prefix_, "fault_latched", &fault_latched_);
  interfaces.emplace_back(state_prefix_, "feedback_age", &feedback_age_);
  interfaces.emplace_back(state_prefix_, "main_error_code", &main_error_);
  interfaces.emplace_back(state_prefix_, "sub_error_code", &sub_error_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < 6; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &commands_[i]);
  }
  return interfaces;
}

size_t FairinoHardwareInterface::owned_interfaces(const std::vector<std::string> & interfaces) const
{
  return std::count_if(interfaces.begin(), interfaces.end(), [this](const auto & name) {
    return std::any_of(info_.joints.begin(), info_.joints.end(), [&name](const auto & joint) {
      return name == joint.name + "/" + hardware_interface::HW_IF_POSITION;
    });
  });
}

hardware_interface::return_type FairinoHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  const auto starting = owned_interfaces(start);
  const auto stopping = owned_interfaces(stop);
  if (
    (starting != 0 && starting != 6) || (stopping != 0 && stopping != 6) ||
    (starting && (!active_ || !session_ || !session_->snapshot().motion_allowed))) {
    RCLCPP_ERROR(logger, "Servo control requires all six ready position interfaces");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  if (owned_interfaces(stop) && session_ && !session_->release()) {
    return hardware_interface::return_type::ERROR;
  }
  if (owned_interfaces(start)) {
    if (!session_ || !session_->claim()) {
      return hardware_interface::return_type::ERROR;
    }
    seed_commands();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!session_) {
    RCLCPP_ERROR(logger, "read called without configured SDK session");
    return hardware_interface::return_type::ERROR;
  }
  update_state();
  // Keep interfaces readable on faults so the guarded JTC can abort its action.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!session_) {
    RCLCPP_ERROR(logger, "write called without configured SDK session");
    return hardware_interface::return_type::ERROR;
  }
  if (active_) {
    session_->command(commands_);
  }
  return hardware_interface::return_type::OK;
}
}  // namespace fairino_hardware

PLUGINLIB_EXPORT_CLASS(
  fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
