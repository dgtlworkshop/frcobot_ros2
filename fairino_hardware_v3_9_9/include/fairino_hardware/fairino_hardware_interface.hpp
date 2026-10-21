#ifndef FAIRINO_HARDWARE_INTERFACE_HPP_
#define FAIRINO_HARDWARE_INTERFACE_HPP_

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fairino_hardware/robot_session.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visibility_control.h"

class robot_command_thread;

namespace fairino_hardware
{
class FairinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  FAIRINO_HARDWARE_PUBLIC
  ~FairinoHardwareInterface() override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  virtual std::unique_ptr<RobotBackend> create_backend() { return make_sdk_backend(); }

private:
  void close_session();
  size_t owned_interfaces(const std::vector<std::string> & interfaces) const;
  void update_state();
  void seed_commands();

  std::array<double, 6> positions_{};
  std::array<double, 6> velocities_{};
  std::array<double, 6> efforts_{};
  std::array<double, 6> commands_{};
  double motion_allowed_ = 0.0;
  double connected_ = 0.0;
  double fault_latched_ = 0.0;
  double feedback_age_ = 0.0;
  double main_error_ = 0.0;
  double sub_error_ = 0.0;
  bool active_ = false;
  std::string robot_ip_ = "192.168.58.2";
  std::string state_prefix_ = "fairino";
  std::chrono::milliseconds feedback_timeout_{100};
  std::chrono::milliseconds command_timeout_{100};
  std::shared_ptr<RobotSession> session_;
  std::shared_ptr<robot_command_thread> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::atomic<bool> executor_running_{false};
  std::thread executor_thread_;
};
}  // namespace fairino_hardware
#endif
