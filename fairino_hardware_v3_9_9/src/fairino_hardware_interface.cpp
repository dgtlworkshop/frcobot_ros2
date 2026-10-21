#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware
{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & sysinfo)
{
  if (
    this->hardware_interface::SystemInterface::on_init(sysinfo) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  this->info_ = sysinfo;  //info_是父类中定义的变量

  auto it = this->info_.hardware_parameters.find("robot_ip");
  if (it != this->info_.hardware_parameters.end()) {
    this->_controller_ip = it->second;
    RCLCPP_INFO(
      this->get_logger(), "Found controller IP in URDF: %s", this->_controller_ip.c_str());
  } else {
    RCLCPP_WARN(
      this->get_logger(), "No 'robot_ip' parameter. Falling back to default: %s",
      CONTROLLER_IP_ADDRESS);
    this->_controller_ip = CONTROLLER_IP_ADDRESS;
  }

  for (const hardware_interface::ComponentInfo & joint : this->info_.joints) {
    //指令部分
    if (joint.command_interfaces.size() != 1) {  //开放servoJ
      RCLCPP_FATAL(
        this->get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
    //     RCLCPP_FATAL(this->get_logger(),
    //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
    //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    //关节状态部分
    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        this->get_logger(), "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Joint '%s' have %s state interface as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
    //     RCLCPP_FATAL(this->get_logger(),
    //                 "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
    //                 joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
    //     RCLCPP_FATAL(this->get_logger(),
    //                 "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
    //                 joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}  //end on_init

std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < this->info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->info_.joints[i].name, hardware_interface::HW_IF_POSITION, &this->_jnt_position_state[i])
      );

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state.at(i)));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < this->info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      this->info_.joints[i].name, hardware_interface::HW_IF_POSITION, &this->_jnt_position_command[i]));

    //     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
    //         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  using namespace std::chrono_literals;
  RCLCPP_INFO(this->get_logger(), "Starting ...please wait...");
  //做变量的初始化工作
  this->_ptr_robot = std::make_unique<FRRobot>();  //创建机器人实例
  for (int i = 0; i < 6; i++) {                    //初始化变量
    this->_jnt_position_command[i] = 0;
    this->_jnt_velocity_command[i] = 0;
    this->_jnt_torque_command[i] = 0;
    this->_jnt_position_state[i] = 0;
    this->_jnt_velocity_state[i] = 0;
    this->_jnt_torque_state[i] = 0;
  }
  this->_control_mode = 0;  //默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
  const errno_t rpc_returncode = this->_ptr_robot->RPC(_controller_ip.c_str());  //建立xmlrpc连接
  if (rpc_returncode != 0) {
    RCLCPP_FATAL(
      this->get_logger(), "Failed to intitialize libfairino SDK, code: %d", rpc_returncode);
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(this->get_logger(), "Initialized libfairino SDK");
  }

  rclcpp::sleep_for(200ms);  //等待一段时间让控制器的rpc连接建立完毕

  //做第一步的工作，读取当前状态数据
  JointPos jntpos;
  const auto joint_read_returncode = this->_ptr_robot->GetActualJointPosDegree(0, &jntpos);
  /*
    获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
    因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
    */
  if (joint_read_returncode == 0) {
    for (int j = 0; j < 6; j++) {
      this->_jnt_position_command[j] = jntpos.jPos[j] / 180.0 * M_PI;
    }
    RCLCPP_INFO(
      this->get_logger(), "Read initial joint positions: %f,%f,%f,%f,%f,%f",
      this->_jnt_position_command[0], this->_jnt_position_command[1],
      this->_jnt_position_command[2], this->_jnt_position_command[3],
      this->_jnt_position_command[4], this->_jnt_position_command[5]);
    return hardware_interface::CallbackReturn::SUCCESS;
  } else {
    RCLCPP_FATAL(
      this->get_logger(), "Failed to read initial joint positions. Error code: %d",
      joint_read_returncode);
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Stopping ...please wait...");
  const auto motion_returncode = this->_ptr_robot->StopMotion();  // 停止机器人
  if (motion_returncode != 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to stop robot motion. Error code: %d", motion_returncode);
  }
  const auto rpc_returncode = this->_ptr_robot->CloseRPC();  //销毁实例，连接断开
  if (rpc_returncode != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to stop robot RPC. Error code: %d", motion_returncode);
  }
  this->_ptr_robot.release();

  if (motion_returncode != 0 && rpc_returncode != 0) {
    RCLCPP_INFO(this->get_logger(), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
  } else {
    RCLCPP_WARN(this->get_logger(), "System may not have successfully stopped.");
    return hardware_interface::CallbackReturn::FAILURE;
  }
}

hardware_interface::return_type FairinoHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{  //从RTDE反馈数据中获取所需的位置，速度和扭矩信息
  JointPos state_data;
  error_t returncode = this->_ptr_robot->GetActualJointPosDegree(1, &state_data);
  if (returncode == 0) {
    for (int i = 0; i < 6; i++) {
      this->_jnt_position_state[i] =
        state_data.jPos[i] / 180.0 * M_PI;  //注意单位转换，moveit统一用弧度
      //_jnt_torque_state[i] = state_data.jt_cur_tor[i];//注意单位转换
    }
  } else {
    RCLCPP_FATAL(this->get_logger(), "GetActualJointPosDegree error: %d", returncode);
    return hardware_interface::return_type::ERROR;
  }
  //RCLCPP_INFO(this->get_logger(), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],_jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (_control_mode == 0) {  //位置控制模式
    if (std::any_of(&this->_jnt_position_command[0], &this->_jnt_position_command[5], [](double c) {
          return not std::isfinite(c);
        })) {
      return hardware_interface::return_type::ERROR;
    }
    JointPos cmd;
    ExaxisPos extcmd{0, 0, 0, 0};
    for (auto j = 0; j < 6; j++) {
      cmd.jPos[j] = this->_jnt_position_command[j] / M_PI * 180;  //注意单位转换
    }
    //RCLCPP_INFO(this->get_logger(), "ServoJ下发位置:%f,%f,%f,%f,%f,%f",cmd.jPos[0],cmd.jPos[1],cmd.jPos[2],cmd.jPos[3],cmd.jPos[4],cmd.jPos[5]);
    const auto returncode = this->_ptr_robot->ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);
    if (returncode != 0) {
      RCLCPP_FATAL(this->get_logger(), "ServoJ error: %d", returncode);
      return hardware_interface::return_type::ERROR;
    }
  } else if (_control_mode == 1) {  //扭矩控制模式
    if (std::any_of(&this->_jnt_torque_command[0], &this->_jnt_torque_command[5], [](double c) {
          return not std::isfinite(c);
        })) {
      RCLCPP_ERROR(
        this->get_logger(),
        "ServoJ is using torque mode, however one or more of the torque values are not finite.");
      return hardware_interface::return_type::ERROR;
    }
    //_ptr_robot->write(_jnt_torque_command);//注意单位转换
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Invalid fairino_hardware_interface::_control_mode is %d",
      this->_control_mode);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

rclcpp::Logger FairinoHardwareInterface::get_logger()
{
  return rclcpp::get_logger("FairinoHardwareInterface");
}

}  // namespace fairino_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
