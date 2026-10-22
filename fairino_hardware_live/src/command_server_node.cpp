#include "fairino_hardware/command_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  //该main函数用于创建简化指令客户端的app
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor mulexecutor;

  //创建用户指令节点
  auto command_server_node = std::make_shared<robot_command_thread>("fr_command_server");
  mulexecutor.add_node(command_server_node);

  mulexecutor.spin();
  rclcpp::shutdown();

  return 0;
}
