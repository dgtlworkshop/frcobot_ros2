#include <gtest/gtest.h>

#include "fairino_hardware/command_server.hpp"
#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "fairino_hardware/fault_aware_joint_trajectory_controller.hpp"
#include "fake_robot.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using fairino_hardware::FaultAwareJointTrajectoryController;
using Action = control_msgs::action::FollowJointTrajectory;

class RosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  template <typename Predicate>
  bool spin_until(Predicate predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }
  rclcpp::executors::SingleThreadedExecutor executor;
};

TEST_F(RosTest, LoadsBothInstalledPluginsWithoutOpeningSdkConnections)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> hardware_loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  auto hardware = hardware_loader.createSharedInstance("fairino_hardware/FairinoHardwareInterface");
  ASSERT_NE(hardware, nullptr);
  pluginlib::ClassLoader<controller_interface::ControllerInterface> controller_loader(
    "controller_interface", "controller_interface::ControllerInterface");
  auto controller =
    controller_loader.createSharedInstance("fairino_hardware/FaultAwareJointTrajectoryController");
  ASSERT_NE(controller, nullptr);
}

TEST_F(RosTest, PublishesBeforeControlAfterFaultAndOnDisconnect)
{
  auto backend = std::make_unique<FakeRobot>();
  auto robot = backend.get();
  auto session = std::make_shared<fairino_hardware::RobotSession>(std::move(backend));
  auto server = std::make_shared<robot_command_thread>("test_fairino", session, "127.0.0.1");
  auto observer = std::make_shared<rclcpp::Node>("test_observer");
  executor.add_node(server);
  executor.add_node(observer);
  fairino_msgs::msg::RobotNonrtState latest;
  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  size_t messages = 0;
  auto sub = observer->create_subscription<fairino_msgs::msg::RobotNonrtState>(
    "nonrt_state_data", rclcpp::QoS(1).transient_local(),
    [&](const fairino_msgs::msg::RobotNonrtState::SharedPtr message) {
      latest = *message;
      ++messages;
    });
  auto diagnostic_sub = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "fairino/diagnostics", rclcpp::QoS(1).transient_local(),
    [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      diagnostics = *message;
    });
  ASSERT_TRUE(spin_until([&]() { return messages > 0 && latest.timestamp != 0; }));
  EXPECT_FALSE(session->snapshot().claimed);
  {
    std::lock_guard<std::mutex> guard(robot->mutex);
    robot->state.main_code = 88;
    robot->state.sub_code = 9;
    robot->state.jt_cur_pos[5] = 42.0;
  }
  ASSERT_TRUE(spin_until([&]() { return latest.main_error_code == 88; }));
  EXPECT_EQ(latest.sub_error_code, 9u);
  EXPECT_DOUBLE_EQ(latest.j6_cur_pos, 42.0);
  ASSERT_TRUE(spin_until([&]() {
    return !diagnostics.status.empty() &&
           diagnostics.status[0].level == diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }));
  {
    std::lock_guard<std::mutex> guard(robot->mutex);
    robot->receive_result = 1;
  }
  ASSERT_TRUE(spin_until([&]() { return latest.reconnect_flag == 1; }));
  EXPECT_EQ(latest.main_error_code, 88u);
  EXPECT_DOUBLE_EQ(latest.j6_cur_pos, 42.0);
  const auto timestamp = latest.timestamp;
  const auto before = messages;
  ASSERT_TRUE(spin_until([&]() { return messages > before + 2; }));
  EXPECT_EQ(latest.timestamp, timestamp);
  auto client = observer->create_client<fairino_msgs::srv::RemoteCmdInterface>(
    "fairino_remote_command_service");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
  request->cmd_str = "MoveJ(1)";
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until([&]() { return future.wait_for(0s) == std::future_status::ready; }));
  EXPECT_EQ(future.get()->cmd_res, "-1");
  executor.remove_node(server);
  executor.remove_node(observer);
}

class ControllerTest : public RosTest
{
protected:
  void SetUp() override
  {
    controller = std::make_unique<FaultAwareJointTrajectoryController>();
    const std::vector<std::string> joints{"j1", "j2", "j3", "j4", "j5", "j6"};
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("joints", joints),
       rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position"}),
       rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position"})});
    ASSERT_EQ(
      controller->init("test_controller", "", options), controller_interface::return_type::OK);
    controller->configure();
    commands.reserve(6);
    states.reserve(7);
    for (const auto & name : joints) {
      commands.emplace_back(name, "position", &command_values[commands.size()]);
      states.emplace_back(name, "position", &state_values[states.size()]);
    }
    states.emplace_back("fairino", "motion_allowed", &permission);
    assign_interfaces();
    controller->get_node()->activate();
    ASSERT_EQ(controller->get_state().label(), "active");
    observer = std::make_shared<rclcpp::Node>("action_observer");
    executor.add_node(observer);
    executor.add_node(controller->get_node()->get_node_base_interface());
    client =
      rclcpp_action::create_client<Action>(observer, "/test_controller/follow_joint_trajectory");
    ASSERT_TRUE(client->wait_for_action_server(1s));
  }
  void TearDown() override
  {
    if (controller->get_state().label() == "active") {
      controller->get_node()->deactivate();
    }
    executor.remove_node(controller->get_node()->get_node_base_interface());
    executor.remove_node(observer);
  }
  void assign_interfaces()
  {
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    for (auto & command : commands) {
      command_loans.emplace_back(command);
    }
    for (auto & state : states) {
      state_loans.emplace_back(state);
    }
    controller->assign_interfaces(std::move(command_loans), std::move(state_loans));
  }
  rclcpp_action::ClientGoalHandle<Action>::SharedPtr send_goal()
  {
    Action::Goal goal;
    goal.trajectory.joint_names = {"j1", "j2", "j3", "j4", "j5", "j6"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = std::vector<double>(6, 0.1);
    point.time_from_start.sec = 10;
    goal.trajectory.points.push_back(point);
    auto future = client->async_send_goal(goal);
    if (!spin_until([&]() { return future.wait_for(0s) == std::future_status::ready; })) {
      ADD_FAILURE() << "Timed out sending goal";
      return nullptr;
    }
    return future.get();
  }
  std::unique_ptr<FaultAwareJointTrajectoryController> controller;
  std::shared_ptr<rclcpp::Node> observer;
  rclcpp_action::Client<Action>::SharedPtr client;
  std::array<double, 6> command_values{};
  std::array<double, 6> state_values{};
  double permission = 1.0;
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
};

TEST_F(ControllerTest, FaultAbortsActionAndCannotResumeOrAcceptNewGoals)
{
  const auto goal = send_goal();
  ASSERT_NE(goal, nullptr);
  ASSERT_EQ(
    controller->update(observer->now(), rclcpp::Duration::from_seconds(0.008)),
    controller_interface::return_type::OK);
  auto result = client->async_get_result(goal);
  permission = 0.0;
  EXPECT_EQ(
    controller->update(observer->now(), rclcpp::Duration::from_seconds(0.008)),
    controller_interface::return_type::ERROR);
  ASSERT_TRUE(spin_until([&]() { return result.wait_for(0s) == std::future_status::ready; }));
  const auto wrapped = result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(wrapped.result->error_code, Action::Result::PATH_TOLERANCE_VIOLATED);
  EXPECT_NE(wrapped.result->error_string.find("motion authority"), std::string::npos);
  const auto last_command = command_values;
  permission = 1.0;
  controller->update(observer->now(), rclcpp::Duration::from_seconds(0.008));
  EXPECT_EQ(command_values, last_command);
  EXPECT_EQ(send_goal(), nullptr);
}

TEST_F(ControllerTest, RequestsStrictDeactivationFromHumbleManager)
{
  bool requested = false;
  auto service = observer->create_service<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller",
    [&](
      const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
      std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response) {
      EXPECT_EQ(request->deactivate_controllers, std::vector<std::string>{"test_controller"});
      EXPECT_EQ(request->strictness, request->STRICT);
      requested = true;
      controller->get_node()->deactivate();
      response->ok = true;
    });
  const auto goal = send_goal();
  ASSERT_NE(goal, nullptr);
  auto result = client->async_get_result(goal);
  permission = 0.0;
  controller->update(observer->now(), rclcpp::Duration::from_seconds(0.008));
  ASSERT_TRUE(spin_until([&]() { return requested; }));
  EXPECT_EQ(controller->get_state().label(), "inactive");
  ASSERT_TRUE(spin_until([&]() { return result.wait_for(0s) == std::future_status::ready; }));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(send_goal(), nullptr);
  permission = 1.0;
  assign_interfaces();
  controller->get_node()->activate();
  EXPECT_EQ(controller->get_state().label(), "active");
  EXPECT_NE(send_goal(), nullptr);
}

TEST_F(ControllerTest, NormalCancellationStillWorks)
{
  const auto goal = send_goal();
  ASSERT_NE(goal, nullptr);
  auto result = client->async_get_result(goal);
  auto cancel = client->async_cancel_goal(goal);
  ASSERT_TRUE(spin_until([&]() { return cancel.wait_for(0s) == std::future_status::ready; }));
  ASSERT_TRUE(spin_until([&]() { return result.wait_for(0s) == std::future_status::ready; }));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(ControllerTest, NormalTrajectoryCompletionStillWorks)
{
  Action::Goal goal;
  goal.trajectory.joint_names = {"j1", "j2", "j3", "j4", "j5", "j6"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = std::vector<double>(6, 0.0);
  point.time_from_start.nanosec = 10000000;
  goal.trajectory.points.push_back(point);
  auto accepted = client->async_send_goal(goal);
  ASSERT_TRUE(spin_until([&]() { return accepted.wait_for(0s) == std::future_status::ready; }));
  const auto handle = accepted.get();
  ASSERT_NE(handle, nullptr);
  auto result = client->async_get_result(handle);
  const auto start = observer->now();
  ASSERT_EQ(
    controller->update(start, rclcpp::Duration::from_seconds(0.008)),
    controller_interface::return_type::OK);
  controller->update(
    start + rclcpp::Duration::from_seconds(0.1), rclcpp::Duration::from_seconds(0.008));
  ASSERT_TRUE(spin_until([&]() { return result.wait_for(0s) == std::future_status::ready; }));
  const auto wrapped = result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(wrapped.result->error_code, Action::Result::SUCCESSFUL);
}

class MockHardware : public fairino_hardware::FairinoHardwareInterface
{
public:
  FakeRobot * robot = nullptr;

protected:
  std::unique_ptr<fairino_hardware::RobotBackend> create_backend() override
  {
    auto backend = std::make_unique<FakeRobot>();
    robot = backend.get();
    for (size_t i = 0; i < 6; ++i) {
      robot->state.jt_cur_pos[i] = 10.0 * (i + 1);
      robot->state.actual_qd[i] = 2.0 * (i + 1);
      robot->state.jt_cur_tor[i] = 3.0 * (i + 1);
    }
    return backend;
  }
};

TEST_F(RosTest, HardwareLifecycleSeedsMeasuredStateAndKeepsMonitoringAfterError)
{
  MockHardware hardware;
  hardware_interface::HardwareInfo info;
  for (int i = 1; i <= 6; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "j" + std::to_string(i);
    for (const auto & name : {"position", "velocity", "effort"}) {
      hardware_interface::InterfaceInfo state;
      state.name = name;
      joint.state_interfaces.push_back(state);
    }
    joint.command_interfaces.push_back(joint.state_interfaces.front());
    info.joints.push_back(joint);
  }
  const rclcpp_lifecycle::State state;
  const auto success = hardware_interface::CallbackReturn::SUCCESS;
  ASSERT_EQ(hardware.on_init(info), success);
  ASSERT_EQ(hardware.on_configure(state), success);
  auto measured = hardware.export_state_interfaces();
  auto commanded = hardware.export_command_interfaces();
  auto observer = std::make_shared<rclcpp::Node>("hardware_observer");
  executor.add_node(observer);
  robot_feedback_msg latest;
  size_t messages = 0;
  auto sub = observer->create_subscription<robot_feedback_msg>(
    "nonrt_state_data", rclcpp::QoS(1).transient_local(),
    [&](const robot_feedback_msg::SharedPtr msg) {
      latest = *msg;
      ++messages;
    });
  ASSERT_TRUE(spin_until([&]() { return messages > 0 && latest.timestamp != 0; }));
  EXPECT_EQ(hardware.robot->starts, 0);
  EXPECT_EQ(hardware.robot->sends, 0);
  ASSERT_EQ(hardware.on_activate(state), success);
  for (size_t i = 0; i < 6; ++i) {
    const double position = (i + 1) * 10.0 * 3.141592653589793 / 180.0;
    EXPECT_NEAR(measured[i * 3].get_value(), position, 1e-10);
    EXPECT_NEAR(measured[i * 3 + 1].get_value(), (i + 1) * 2.0 * 3.141592653589793 / 180.0, 1e-10);
    EXPECT_DOUBLE_EQ(measured[i * 3 + 2].get_value(), (i + 1) * 3.0);
    EXPECT_NEAR(commanded[i].get_value(), position, 1e-10);
  }
  std::vector<std::string> start;
  for (const auto & command : commanded) {
    start.push_back(command.get_name());
  }
  EXPECT_EQ(
    hardware.prepare_command_mode_switch({start[0]}, {}), hardware_interface::return_type::ERROR);
  ASSERT_EQ(hardware.prepare_command_mode_switch(start, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(start, {}), hardware_interface::return_type::OK);
  hardware.write(observer->now(), rclcpp::Duration::from_seconds(0.008));
  ASSERT_TRUE(spin_until([&]() { return hardware.robot->sends > 0; }));
  {
    std::lock_guard<std::mutex> guard(hardware.robot->mutex);
    for (size_t i = 0; i < 6; ++i) {
      EXPECT_NEAR(hardware.robot->last_target[i], (i + 1) * 10.0, 1e-10);
    }
    hardware.robot->state.main_code = 345;
  }
  ASSERT_TRUE(spin_until([&]() { return latest.main_error_code == 345; }));
  hardware.read(observer->now(), rclcpp::Duration::from_seconds(0.008));
  const auto permission = std::find_if(
    measured.begin(), measured.end(),
    [](const auto & interface) { return interface.get_name() == "fairino/motion_allowed"; });
  ASSERT_NE(permission, measured.end());
  EXPECT_DOUBLE_EQ(permission->get_value(), 0.0);
  ASSERT_EQ(hardware.on_error(state), success);
  const auto before = messages;
  ASSERT_TRUE(spin_until([&]() { return messages > before + 2; }));
  EXPECT_EQ(latest.main_error_code, 345u);
  ASSERT_EQ(hardware.on_configure(state), success);
  EXPECT_EQ(hardware.robot->starts, 1);
  ASSERT_EQ(hardware.on_deactivate(state), success);
  const auto inactive_messages = messages;
  ASSERT_TRUE(spin_until([&]() { return messages > inactive_messages + 2; }));
  ASSERT_EQ(hardware.on_cleanup(state), success);
  executor.remove_node(observer);
}
