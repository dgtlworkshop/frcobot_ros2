#include "fairino_hardware/fault_aware_joint_trajectory_controller.hpp"

#include <algorithm>

#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_action/create_server.hpp"

namespace fairino_hardware
{
controller_interface::CallbackReturn FaultAwareJointTrajectoryController::on_init()
{
  const auto result = JointTrajectoryController::on_init();
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  auto_declare<std::string>("hardware_state_prefix", "fairino");
  auto_declare<std::string>("controller_manager_name", "/controller_manager");
  return result;
}

controller_interface::CallbackReturn FaultAwareJointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State & state)
{
  const auto prefix = get_node()->get_parameter("hardware_state_prefix").as_string();
  if (prefix.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "hardware_state_prefix must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  permission_interface_ = prefix + "/motion_allowed";
  fault_result_ = std::make_shared<FollowJTrajAction::Result>();
  fault_result_->error_code = FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED;
  fault_result_->error_string =
    "Fairino motion authority lost: robot fault, disconnection, stale feedback or watchdog. "
    "See fairino/diagnostics; explicit recovery and a new trajectory are required.";
  const auto result = JointTrajectoryController::on_configure(state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  // Humble's base action callback checks lifecycle state, not the hardware fault latch.
  action_server_.reset();
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node(), "~/follow_joint_trajectory",
    [this](const auto & uuid, const auto goal) {
      if (faulted_) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      return goal_received_callback(uuid, goal);
    },
    [this](const auto goal) { return goal_cancelled_callback(goal); },
    [this](auto goal) { goal_accepted_callback(goal); });
  const auto manager = get_node()->get_parameter("controller_manager_name").as_string();
  switch_client_ = get_node()->create_client<Switch>(manager + "/switch_controller");
  fault_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(20), [this]() { deactivate_after_fault(); });
  return result;
}

controller_interface::InterfaceConfiguration
FaultAwareJointTrajectoryController::state_interface_configuration() const
{
  auto config = JointTrajectoryController::state_interface_configuration();
  config.names.push_back(permission_interface_);
  return config;
}

controller_interface::CallbackReturn FaultAwareJointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & state)
{
  const auto found = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [this](const auto & interface) { return interface.get_name() == permission_interface_; });
  if (found == state_interfaces_.end() || found->get_value() != 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Fresh, recovered Fairino hardware is required");
    return controller_interface::CallbackReturn::ERROR;
  }
  permission_ = &*found;
  faulted_ = false;
  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type FaultAwareJointTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (faulted_ || !permission_ || permission_->get_value() != 1.0) {
    const bool first_fault = !faulted_.exchange(true);
    if (first_fault) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", fault_result_->error_string.c_str());
    }
    subscriber_is_active_ = false;
    const auto goal = *rt_active_goal_.readFromRT();
    if (goal) {
      goal->setAborted(fault_result_);
    }
    // Never call the upstream sampler again until an explicit deactivate/activate.
    return first_fault ? controller_interface::return_type::ERROR
                       : controller_interface::return_type::OK;
  }

  return JointTrajectoryController::update(time, period);
}

void FaultAwareJointTrajectoryController::deactivate_after_fault()
{
  if (!faulted_ || get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }
  if (switch_future_.valid()) {
    if (switch_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() - switch_request_time_ < std::chrono::seconds(3)) {
        return;
      }
      switch_client_->remove_pending_request(switch_request_id_);
      RCLCPP_ERROR(get_node()->get_logger(), "Fault deactivation response timed out; retrying");
    } else if (!switch_future_.get()->ok) {
      RCLCPP_ERROR(get_node()->get_logger(), "Controller-manager refused fault deactivation");
    }
    switch_future_ = {};
  }
  if (!switch_client_->service_is_ready()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Controller-manager switch service unavailable; motion remains inhibited");
    return;
  }
  auto request = std::make_shared<Switch::Request>();
  request->deactivate_controllers = {get_node()->get_name()};
  request->strictness = Switch::Request::STRICT;
  request->timeout.sec = 2;
  auto pending = switch_client_->async_send_request(request);
  switch_request_id_ = pending.request_id;
  switch_future_ = pending.future.share();
  switch_request_time_ = std::chrono::steady_clock::now();
}

controller_interface::CallbackReturn FaultAwareJointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  if (faulted_) {
    const auto goal = *rt_active_goal_.readFromNonRT();
    if (goal) {
      goal->setAborted(fault_result_);
      // Complete the result before the base class releases the goal and interfaces.
      goal->runNonRealtime();
    }
  }
  permission_ = nullptr;
  return JointTrajectoryController::on_deactivate(state);
}
}  // namespace fairino_hardware

PLUGINLIB_EXPORT_CLASS(
  fairino_hardware::FaultAwareJointTrajectoryController, controller_interface::ControllerInterface)
