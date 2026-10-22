#include <gtest/gtest.h>

#include <limits>

#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "fake_robot.hpp"

using fairino_hardware::RobotSession;
using namespace std::chrono_literals;

class SessionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto backend = std::make_unique<FakeRobot>();
    robot = backend.get();
    session = std::make_unique<RobotSession>(std::move(backend), 16ms, 16ms);
    session->connect("unused", {});
    session->poll_once();
    ASSERT_TRUE(session->snapshot().motion_allowed);
  }
  FakeRobot * robot;
  std::unique_ptr<RobotSession> session;
};

TEST_F(SessionTest, DoesNotSendBeforeClaimAndConvertsAllSixAxes)
{
  robot->state.extAxisStatus[0].pos = 321.0;
  session->poll_once();
  const std::array<double, 6> target{0.0, 0.1, -0.2, 0.3, -0.4, 3.141592653589793};
  session->command(target);
  session->poll_once();
  EXPECT_EQ(robot->sends, 0);
  ASSERT_TRUE(session->claim());
  session->poll_once();
  EXPECT_EQ(robot->sends, 0);
  session->command(target);
  session->poll_once();
  EXPECT_EQ(robot->sends, 1);
  EXPECT_DOUBLE_EQ(robot->last_external[0], 321.0);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(robot->last_target[i], target[i] * 180.0 / 3.141592653589793, 1e-10);
  }
}

TEST_F(SessionTest, RefusesToTakeOverAnAlreadyMovingRobot)
{
  robot->state.robot_state = 2;
  session->poll_once();
  EXPECT_FALSE(session->claim());
  EXPECT_EQ(robot->starts, 0);
}

TEST_F(SessionTest, RobotErrorLatchesStopsOnceAndRequiresExplicitRecovery)
{
  ASSERT_TRUE(session->claim());
  robot->state.main_code = 123;
  robot->state.sub_code = 7;
  session->poll_once();
  EXPECT_EQ(robot->stops, 1);
  EXPECT_EQ(session->snapshot().state.main_code, 123);
  EXPECT_EQ(session->snapshot().state.sub_code, 7);
  session->poll_once();
  EXPECT_EQ(robot->stops, 1);
  robot->state.main_code = robot->state.sub_code = 0;
  session->poll_once();
  EXPECT_FALSE(session->snapshot().motion_allowed);
  std::string reason;
  EXPECT_FALSE(session->recover(reason));
  ASSERT_TRUE(session->release());
  ASSERT_TRUE(session->recover(reason));
  ASSERT_TRUE(session->claim());
  session->poll_once();
  EXPECT_EQ(robot->sends, 0);
  session->command({});
  session->poll_once();
  EXPECT_EQ(robot->sends, 1);
}

TEST_F(SessionTest, DisconnectRetainsLastStateAndNeverResumesOnReconnect)
{
  robot->state.jt_cur_pos[5] = 71.0;
  session->poll_once();
  ASSERT_TRUE(session->claim());
  robot->receive_result = 2;
  session->poll_once();
  auto status = session->snapshot();
  EXPECT_FALSE(status.connected);
  EXPECT_FALSE(status.motion_allowed);
  EXPECT_DOUBLE_EQ(status.state.jt_cur_pos[5], 71.0);
  robot->receive_result = 0;
  session->poll_once();
  EXPECT_TRUE(session->snapshot().connected);
  EXPECT_TRUE(session->snapshot().fault_latched);
}

TEST_F(SessionTest, FrozenRobotTimeExpiresEvenWhenSdkReportsSuccess)
{
  robot->frozen = true;
  std::this_thread::sleep_for(20ms);
  EXPECT_FALSE(session->snapshot().motion_allowed);
  session->poll_once();
  EXPECT_TRUE(session->snapshot().fault_latched);
  EXPECT_FALSE(session->snapshot().fresh);
}

TEST_F(SessionTest, CommandWatchdogExpiresWithFreshFeedback)
{
  ASSERT_TRUE(session->claim());
  std::this_thread::sleep_for(20ms);
  session->poll_once();
  EXPECT_TRUE(session->snapshot().fresh);
  EXPECT_TRUE(session->snapshot().fault_latched);
  EXPECT_EQ(robot->stops, 1);
  EXPECT_EQ(robot->sends, 0);
}

TEST_F(SessionTest, SixthNonFiniteCommandAndConversionOverflowAreRejected)
{
  ASSERT_TRUE(session->claim());
  std::array<double, 6> target{};
  target[5] = std::numeric_limits<double>::quiet_NaN();
  session->command(target);
  session->poll_once();
  EXPECT_EQ(robot->sends, 0);
  EXPECT_TRUE(session->snapshot().fault_latched);
  session->release();
  std::string reason;
  ASSERT_TRUE(session->recover(reason));
  ASSERT_TRUE(session->claim());
  target[5] = std::numeric_limits<double>::max();
  session->command(target);
  session->poll_once();
  EXPECT_EQ(robot->sends, 0);
  EXPECT_TRUE(session->snapshot().fault_latched);
}

TEST_F(SessionTest, SdkServoFailuresLatchAndStop)
{
  robot->start_result = 42;
  EXPECT_FALSE(session->claim());
  session->poll_once();
  EXPECT_EQ(robot->stops, 1);
  robot->start_result = 0;
  std::string reason;
  ASSERT_TRUE(session->recover(reason));
  ASSERT_TRUE(session->claim());
  robot->send_result = 43;
  session->command({});
  session->poll_once();
  EXPECT_TRUE(session->snapshot().fault_latched);
  session->poll_once();
  EXPECT_EQ(robot->stops, 2);
}

TEST_F(SessionTest, StopMotionDoesNotReleaseControllerOwnership)
{
  ASSERT_TRUE(session->claim());
  EXPECT_TRUE(session->stop_motion());
  EXPECT_TRUE(session->snapshot().claimed);
  std::string reason;
  EXPECT_FALSE(session->recover(reason));
  session->poll_once();
  EXPECT_EQ(robot->stops, 1);
}

TEST_F(SessionTest, StopFailureIsNotSuccess)
{
  ASSERT_TRUE(session->claim());
  robot->stop_result = 99;
  EXPECT_FALSE(session->release());
  EXPECT_TRUE(session->snapshot().fault_latched);
}

TEST_F(SessionTest, DisconnectDefersStopAndReleaseNeverBlocks)
{
  ASSERT_TRUE(session->claim());
  robot->receive_result = 2;
  session->poll_once();
  EXPECT_EQ(robot->stops, 0);
  EXPECT_TRUE(session->snapshot().stop_pending);
  EXPECT_TRUE(session->release());
  EXPECT_FALSE(session->snapshot().claimed);
  EXPECT_EQ(robot->stops, 0);
  std::string reason;
  EXPECT_FALSE(session->recover(reason));
  robot->receive_result = 0;
  session->poll_once();
  EXPECT_EQ(robot->stops, 1);
  EXPECT_FALSE(session->snapshot().stop_pending);
  EXPECT_TRUE(session->recover(reason));
  ASSERT_TRUE(session->claim());
}

TEST_F(SessionTest, ReleaseWithBusySdkDefersStopAndBlocksReclaim)
{
  ASSERT_TRUE(session->claim());
  {
    std::lock_guard<std::mutex> guard(session->sdk_mutex());
    EXPECT_TRUE(session->release());
    EXPECT_FALSE(session->claim());
  }
  EXPECT_EQ(robot->stops, 0);
  EXPECT_TRUE(session->snapshot().stop_pending);
  EXPECT_FALSE(session->claim());
  session->poll_once();
  EXPECT_EQ(robot->stops, 1);
  std::string reason;
  ASSERT_TRUE(session->recover(reason));
  ASSERT_TRUE(session->claim());
}

TEST_F(SessionTest, AlarmReasonNamesActiveFlags)
{
  robot->state.cmdPointError = 1;
  robot->state.driverComError[2] = 1;
  session->poll_once();
  const auto readiness = session->snapshot().readiness;
  EXPECT_NE(readiness.find("cmdPointError=1"), std::string::npos);
  EXPECT_NE(readiness.find("driverComError[2]=1"), std::string::npos);
  std::string reason;
  EXPECT_FALSE(session->recover(reason));
  EXPECT_NE(reason.find("ResetAllError()"), std::string::npos);
}

TEST_F(SessionTest, RejectsEveryReadinessLoss)
{
  const auto healthy = robot->state;
  const std::vector<std::function<void(ROBOT_STATE_PKG &)>> faults = {
    [](auto & s) { s.main_code = 1; },
    [](auto & s) { s.sub_code = 1; },
    [](auto & s) { s.EmergencyStop = 1; },
    [](auto & s) { s.safety_stop0_state = 1; },
    [](auto & s) { s.safety_stop1_state = 1; },
    [](auto & s) { s.collisionState = 1; },
    [](auto & s) { s.safetyDataState = 1; },
    [](auto & s) { s.driverComError[5] = 1; },
    [](auto & s) { s.slaveComError[7] = 1; },
    [](auto & s) { s.rbtEnableState = 0; },
    [](auto & s) { s.robot_mode = 1; },
    [](auto & s) { s.robot_state = 3; },
    [](auto & s) { s.robot_state = 4; },
    [](auto & s) { s.programRunState = 1; },
    [](auto & s) { s.actual_qd[5] = std::numeric_limits<double>::infinity(); }};
  for (size_t i = 0; i < faults.size(); ++i) {
    SCOPED_TRACE(i);
    robot->state = healthy;
    faults[i](robot->state);
    session->poll_once();
    EXPECT_FALSE(session->snapshot().motion_allowed);
    EXPECT_TRUE(session->snapshot().fault_latched);
    std::string reason;
    EXPECT_FALSE(session->recover(reason));
    robot->state = healthy;
    session->poll_once();
    EXPECT_TRUE(session->recover(reason));
  }
}

TEST_F(SessionTest, BusySdkAndStalledWorkerFailClosed)
{
  std::this_thread::sleep_for(20ms);
  std::lock_guard<std::mutex> guard(session->sdk_mutex());
  session->poll_once();
  EXPECT_TRUE(session->snapshot().fault_latched);
  EXPECT_FALSE(session->snapshot().motion_allowed);
}

TEST_F(SessionTest, RecoveryIdentifiesAllFailedReadinessConditionsAndRawValues)
{
  robot->state.rbtEnableState = 0;
  robot->state.robot_mode = 1;
  robot->state.robot_state = 3;
  robot->state.programRunState = 1;
  session->poll_once();
  std::string reason;
  ASSERT_FALSE(session->recover(reason));
  EXPECT_NE(reason.find("rbtEnableState=0"), std::string::npos);
  EXPECT_NE(reason.find("robot_mode=1"), std::string::npos);
  EXPECT_NE(reason.find("robot_state=3"), std::string::npos);
  EXPECT_NE(reason.find("programRunState=1"), std::string::npos);
  EXPECT_FALSE(session->snapshot().motion_allowed);
}

TEST_F(SessionTest, AllowsUnknownProgramState200ButPreservesOtherFaultGates)
{
  robot->state.programRunState = 200;
  session->poll_once();
  EXPECT_TRUE(session->snapshot().motion_allowed);
  std::string reason;
  ASSERT_TRUE(session->recover(reason));
  ASSERT_TRUE(session->claim());
  session->command({});
  session->poll_once();
  EXPECT_EQ(robot->sends, 1);
  robot->state.main_code = 42;
  session->poll_once();
  EXPECT_FALSE(session->snapshot().motion_allowed);
  EXPECT_FALSE(session->recover(reason));
  EXPECT_EQ(robot->stops, 1);
}

TEST_F(SessionTest, RejectsProgramStatesOtherThanZeroAndExplicitException)
{
  for (const auto value : {1, 2, 199, 201, 255}) {
    SCOPED_TRACE(value);
    robot->state.programRunState = value;
    session->poll_once();
    EXPECT_FALSE(session->snapshot().motion_allowed);
    std::string reason;
    EXPECT_FALSE(session->recover(reason));
    EXPECT_NE(reason.find("programRunState=" + std::to_string(value)), std::string::npos);
    robot->state.programRunState = 0;
    session->poll_once();
    ASSERT_TRUE(session->recover(reason));
  }
}

TEST(SessionSetup, FailedSdkConfigurationCannotBeAcknowledgedAsReady)
{
  auto backend = std::make_unique<FakeRobot>();
  backend->connect_result = 17;
  RobotSession session(std::move(backend));
  session.connect("unused", {});
  session.poll_once();
  EXPECT_TRUE(session.snapshot().fresh);
  EXPECT_TRUE(session.snapshot().fault_latched);
  EXPECT_FALSE(session.snapshot().motion_allowed);
  std::string reason;
  EXPECT_FALSE(session.recover(reason));
  EXPECT_NE(reason.find("configure"), std::string::npos);
}

TEST(HardwareSchema, RejectsWrongJointCountAndUnsupportedInterfaces)
{
  fairino_hardware::FairinoHardwareInterface hardware;
  hardware_interface::HardwareInfo info;
  EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
  for (int i = 0; i < 6; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "j" + std::to_string(i + 1);
    hardware_interface::InterfaceInfo position;
    position.name = "position";
    joint.command_interfaces.push_back(position);
    joint.state_interfaces.push_back(position);
    info.joints.push_back(joint);
  }
  ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.export_command_interfaces().size(), 6u);
  EXPECT_EQ(hardware.export_state_interfaces().size(), 12u);
  info.joints[5].command_interfaces[0].name = "velocity";
  EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
  info.joints[5].command_interfaces[0].name = "position";
  info.hardware_parameters["feedback_timeout_ms"] = "0";
  EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
}
