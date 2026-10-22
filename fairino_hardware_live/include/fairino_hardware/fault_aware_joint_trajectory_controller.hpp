#ifndef FAIRINO_FAULT_AWARE_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define FAIRINO_FAULT_AWARE_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

namespace fairino_hardware
{
class FaultAwareJointTrajectoryController
: public joint_trajectory_controller::JointTrajectoryController
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string permission_interface_ = "fairino/motion_allowed";
  hardware_interface::LoanedStateInterface * permission_ = nullptr;
  std::atomic<bool> faulted_{false};
  std::shared_ptr<FollowJTrajAction::Result> fault_result_;
  using Switch = controller_manager_msgs::srv::SwitchController;
  rclcpp::Client<Switch>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
  rclcpp::Client<Switch>::SharedFuture switch_future_;
  int64_t switch_request_id_ = 0;
  std::chrono::steady_clock::time_point switch_request_time_{};
  void deactivate_after_fault();
};
}  // namespace fairino_hardware
#endif
