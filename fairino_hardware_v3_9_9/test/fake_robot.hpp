#ifndef FAIRINO_TEST_FAKE_ROBOT_HPP_
#define FAIRINO_TEST_FAKE_ROBOT_HPP_

#include "fairino_hardware/robot_session.hpp"

class FakeRobot : public fairino_hardware::RobotBackend
{
public:
  FakeRobot()
  {
    state.robotTime.year = 2026;
    state.robot_state = 1;
    state.rbtEnableState = 1;
  }
  int connect(const std::string &, const std::vector<RobotState> &) override
  {
    return connect_result;
  }
  int receive(ROBOT_STATE_PKG & output) override
  {
    std::lock_guard<std::mutex> guard(mutex);
    if (!frozen) {
      state.robotTime.millisecond = (state.robotTime.millisecond + 8) % 1000;
    }
    output = state;
    return receive_result;
  }
  int start_servo() override {++starts; return start_result;}
  int send(
    const std::array<double, 6> & target, const std::array<double, 4> & external) override
  {
    std::lock_guard<std::mutex> guard(mutex);
    ++sends;
    last_target = target;
    last_external = external;
    return send_result;
  }
  int stop() override {++stops; return stop_result;}

  std::mutex mutex;
  ROBOT_STATE_PKG state{};
  bool frozen = false;
  int connect_result = 0;
  int receive_result = 0;
  int start_result = 0;
  int send_result = 0;
  int stop_result = 0;
  std::atomic<int> starts{0};
  std::atomic<int> sends{0};
  std::atomic<int> stops{0};
  std::array<double, 6> last_target{};
  std::array<double, 4> last_external{};
};
#endif
