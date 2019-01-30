#include "rtde_control_interface.h"

#include <iostream>

RTDEControlInterface::RTDEControlInterface(std::string hostname, int port) : hostname_(std::move(hostname)), port_(port)
{
  rtde_ = std::make_shared<RTDE>(hostname_);
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  auto controller_version = rtde_->getControllerVersion();
  uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

  double frequency = 125;
  // If e-Series Robot set frequency to 500Hz
  if (major_version > CB3_MAJOR_VERSION)
    frequency = 500;

  // Create a connection to the dashboard server
  db_client_ = std::make_shared<DashboardClient>(hostname_);
  db_client_->connect();

  // Create a connection to the script server
  script_client_ = std::make_shared<ScriptClient>(hostname_);
  script_client_->connect();

  // Setup output
  std::vector<std::string> state_names = {"target_q", "target_qd", "output_int_register_0"};
  rtde_->sendOutputSetup(state_names, frequency);

  // Setup input recipes
  // Recipe 1
  std::vector<std::string> setp_input = {
      "input_int_register_0",    "input_double_register_0", "input_double_register_1",
      "input_double_register_2", "input_double_register_3", "input_double_register_4",
      "input_double_register_5", "input_double_register_6", "input_double_register_7"};
  rtde_->sendInputSetup(setp_input);

  // Recipe 2
  std::vector<std::string> movec_input = {
      "input_int_register_0",     "input_double_register_0",  "input_double_register_1",  "input_double_register_2",
      "input_double_register_3",  "input_double_register_4",  "input_double_register_5",  "input_double_register_6",
      "input_double_register_7",  "input_double_register_8",  "input_double_register_9",  "input_double_register_10",
      "input_double_register_11", "input_double_register_12", "input_double_register_13", "input_int_register_1"};
  rtde_->sendInputSetup(movec_input);

  // Recipe 3
  std::vector<std::string> servoj_input = {
      "input_int_register_0",    "input_double_register_0", "input_double_register_1", "input_double_register_2",
      "input_double_register_3", "input_double_register_4", "input_double_register_5", "input_double_register_6",
      "input_double_register_7", "input_double_register_8", "input_double_register_9", "input_double_register_10"};
  rtde_->sendInputSetup(servoj_input);

  // Recipe 4
  std::vector<std::string> force_mode_input = {
      "input_int_register_0",     "input_int_register_1",     "input_int_register_2",     "input_int_register_3",
      "input_int_register_4",     "input_int_register_5",     "input_int_register_6",     "input_int_register_7",
      "input_double_register_0",  "input_double_register_1",  "input_double_register_2",  "input_double_register_3",
      "input_double_register_4",  "input_double_register_5",  "input_double_register_6",  "input_double_register_7",
      "input_double_register_8",  "input_double_register_9",  "input_double_register_10", "input_double_register_11",
      "input_double_register_12", "input_double_register_13", "input_double_register_14", "input_double_register_15",
      "input_double_register_16", "input_double_register_17"};
  rtde_->sendInputSetup(force_mode_input);

  // Recipe 5
  std::vector<std::string> no_cmd_input = {"input_int_register_0"};
  rtde_->sendInputSetup(no_cmd_input);

  // Start RTDE data synchronization
  rtde_->sendStart();

  // Signal the controller to get ready for commands
  RTDE::RobotCommand clear_cmd;
  clear_cmd.type_ = RTDE::RobotCommand::NO_CMD;
  clear_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  rtde_->send(clear_cmd);

  // Send script to the UR Controller
  script_client_->sendScript("../scripts/rtde_control.script");

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>();
}

RTDEControlInterface::~RTDEControlInterface()
{
  if (rtde_ != nullptr)
  {
    if (rtde_->isConnected())
      rtde_->disconnect();
  }

  if (script_client_ != nullptr)
  {
    if (script_client_->isConnected())
      script_client_->disconnect();
  }

  if (db_client_ != nullptr)
  {
    if (db_client_->isConnected())
      db_client_->disconnect();
  }
}

void RTDEControlInterface::stopRobot()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  pushCommand(robot_cmd);
  sendCommand();
}

void RTDEControlInterface::verifyValueIsWithin(const double& value, const double& min, const double& max)
{
  if (std::isnan(min) || std::isnan(max))
  {
    throw std::invalid_argument("Make sure both min and max are not NaN's");
  }
  else if (std::isnan(value))
  {
    throw std::invalid_argument("The value is considered NaN");
  }
  else if (!(std::isgreaterequal(value, min) && std::islessequal(value, max)))
  {
    std::ostringstream oss;
    oss << "The value is not within [" << min << ";" << max << "]";
    throw std::range_error(oss.str());
  }
}

std::string RTDEControlInterface::prepareCmdScript(const std::vector<std::vector<double>>& path, const std::string& cmd)
{
  std::string cmd_str;
  std::stringstream ss;
  cmd_str += "def motions():\n";
  cmd_str += "\twrite_output_integer_register(0, 0)\n";
  for (const auto& pose : path)
  {
    verifyValueIsWithin(pose[6], UR_VELOCITY_MIN, UR_VELOCITY_MAX);
    verifyValueIsWithin(pose[7], UR_ACCELERATION_MIN, UR_ACCELERATION_MAX);
    verifyValueIsWithin(pose[8], UR_BLEND_MIN, UR_BLEND_MAX);
    ss << "\t" << cmd << "[" << pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << ","
       << pose[5] << "],"
       << "a=" << pose[7] << ",v=" << pose[6] << ",r=" << pose[8] << ")\n";
  }
  cmd_str += ss.str();

  // Signal when motions are finished
  cmd_str += "\twrite_output_integer_register(0, 1)\n";
  cmd_str += "end\n";
  return cmd_str;
}

void RTDEControlInterface::moveJ(const std::vector<std::vector<double>>& path)
{
  // First stop the running RTDE control script
  stopRobot();

  // Send motions
  script_client_->sendScriptCommand(prepareCmdScript(path, "movej("));

  rtde_->receiveData(robot_state_);

  // Wait until the controller has finished executing motions
  while (robot_state_->getOutput_int_register_0() == 0)
  {
    // Receive RobotState
    rtde_->receiveData(robot_state_);
  }

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript("../scripts/rtde_control.script");
}

void RTDEControlInterface::moveJ(const std::vector<double>& joints, double speed, double acceleration)
{
  verifyValueIsWithin(speed, UR_VELOCITY_MIN, UR_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_ACCELERATION_MIN, UR_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = joints;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  pushCommand(robot_cmd);
  sendCommand();
}

void RTDEControlInterface::moveJ_IK(const std::vector<double>& transform, double speed, double acceleration)
{
  verifyValueIsWithin(speed, UR_VELOCITY_MIN, UR_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_ACCELERATION_MIN, UR_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEJ_IK;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = transform;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  pushCommand(robot_cmd);
  sendCommand();
}

void RTDEControlInterface::moveL(const std::vector<std::vector<double>>& path)
{
  // First stop the running RTDE control script
  stopRobot();

  // Send motions
  script_client_->sendScriptCommand(prepareCmdScript(path, "movel(p"));

  rtde_->receiveData(robot_state_);

  // Wait until the controller has finished executing motions
  while (robot_state_->getOutput_int_register_0() == 0)
  {
    // Receive RobotState
    rtde_->receiveData(robot_state_);
  }

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript("../scripts/rtde_control.script");
}

void RTDEControlInterface::moveL(const std::vector<double>& transform, double speed, double acceleration)
{
  verifyValueIsWithin(speed, UR_VELOCITY_MIN, UR_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_ACCELERATION_MIN, UR_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = transform;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  pushCommand(robot_cmd);
  sendCommand();
}

void RTDEControlInterface::moveL_FK(const std::vector<double>& joints, double speed, double acceleration)
{
  verifyValueIsWithin(speed, UR_VELOCITY_MIN, UR_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_ACCELERATION_MIN, UR_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEL_FK;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = joints;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  pushCommand(robot_cmd);
  sendCommand();
}

void RTDEControlInterface::moveC(const std::vector<double>& pose_via, const std::vector<double>& pose_to, double speed,
                                 double acceleration, int mode)
{
  verifyValueIsWithin(speed, UR_VELOCITY_MIN, UR_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_ACCELERATION_MIN, UR_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEC;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_2;
  robot_cmd.val_ = pose_via;
  for (const auto& val : pose_to)
    robot_cmd.val_.push_back(val);

  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.movec_mode_ = mode;
  pushCommand(robot_cmd);
  sendCommand();
}

void RTDEControlInterface::forceModeStart(const std::vector<double>& task_frame,
                                          const std::vector<int>& selection_vector, const std::vector<double>& wrench,
                                          int type, const std::vector<double>& limits)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_START;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  robot_cmd.val_ = task_frame;
  for (const auto& val : wrench)
    robot_cmd.val_.push_back(val);

  for (const auto& val : limits)
    robot_cmd.val_.push_back(val);

  robot_cmd.selection_vector_ = selection_vector;
  robot_cmd.force_mode_type_ = type;
  pushCommand(robot_cmd);
  sendCommand();
}

void RTDEControlInterface::forceModeUpdate(const std::vector<double>& wrench)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_UPDATE;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = wrench;
  robot_cmd.val_.push_back(0.0);
  robot_cmd.val_.push_back(0.0);
  pushCommand(robot_cmd);
  sendCommand();
}

void RTDEControlInterface::forceModeStop()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  pushCommand(robot_cmd);
  sendCommand();
}

void RTDEControlInterface::sendCommand()
{
  // Check if any commands are available in the command queue
  if (!command_queue_.empty())
  {
    // Receive RobotState
    rtde_->receiveData(robot_state_);

    // Wait until the controller is ready for a command
    while (robot_state_->getOutput_int_register_0() == 0)
    {
      // Receive RobotState
      rtde_->receiveData(robot_state_);
    }

    RTDE::RobotCommand cmd = popCommand();
    rtde_->send(cmd);

    // Wait until the controller has finished executing
    while (robot_state_->getOutput_int_register_0() != 0)
    {
      // Receive RobotState
      rtde_->receiveData(robot_state_);
    }

    RTDE::RobotCommand clear_cmd;
    clear_cmd.type_ = RTDE::RobotCommand::NO_CMD;
    clear_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
    // Tell the controller to clear executing status, since there is no command yet
    rtde_->send(clear_cmd);
  }
}