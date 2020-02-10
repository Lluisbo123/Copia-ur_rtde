#include <ur_rtde/rtde_control_interface.h>
#include <iostream>
#include <bitset>
#include <chrono>

namespace ur_rtde
{
RTDEControlInterface::RTDEControlInterface(std::string hostname, int port) : hostname_(std::move(hostname)), port_(port)
{
  rtde_ = std::make_shared<RTDE>(hostname_);
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  auto controller_version = rtde_->getControllerVersion();
  uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);
  uint32_t minor_version = std::get<MINOR_VERSION>(controller_version);

  double frequency = 125;
  // If e-Series Robot set frequency to 500Hz
  if (major_version > CB3_MAJOR_VERSION)
    frequency = 500;

  // Create a connection to the dashboard server
  db_client_ = std::make_shared<DashboardClient>(hostname_);
  db_client_->connect();

  // Create a connection to the script server
  script_client_ = std::make_shared<ScriptClient>(hostname_, major_version, minor_version);
  script_client_->connect();

  // Setup output
  std::vector<std::string> state_names = {
      "robot_status_bits",        "output_int_register_0",    "output_int_register_1",
      "output_double_register_0", "output_double_register_1", "output_double_register_2",
      "output_double_register_3", "output_double_register_4", "output_double_register_5"};
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

  // Recipe 6
  std::vector<std::string> servoc_input = {
      "input_int_register_0",    "input_double_register_0", "input_double_register_1", "input_double_register_2",
      "input_double_register_3", "input_double_register_4", "input_double_register_5", "input_double_register_6",
      "input_double_register_7", "input_double_register_8"};
  rtde_->sendInputSetup(servoc_input);

  // Recipe 7
  std::vector<std::string> wrench_input = {
      "input_int_register_0",    "input_double_register_0", "input_double_register_1", "input_double_register_2",
      "input_double_register_3", "input_double_register_4", "input_double_register_5"};
  rtde_->sendInputSetup(wrench_input);

  // Recipe 8
  std::vector<std::string> set_payload_input = {"input_int_register_0", "input_double_register_0",
                                                "input_double_register_1", "input_double_register_2",
                                                "input_double_register_3"};
  rtde_->sendInputSetup(set_payload_input);

  // Recipe 9
  std::vector<std::string> force_mode_parameters_input = {"input_int_register_0", "input_double_register_0"};
  rtde_->sendInputSetup(force_mode_parameters_input);

  // Recipe 10
  std::vector<std::string> get_actual_joint_positions_history_input = {"input_int_register_0", "input_int_register_1"};
  rtde_->sendInputSetup(get_actual_joint_positions_history_input);

  // Recipe 11
  std::vector<std::string> get_inverse_kin_input = {
      "input_int_register_0", "input_double_register_0", "input_double_register_1", "input_double_register_2",
      "input_double_register_3", "input_double_register_4", "input_double_register_5", "input_double_register_6",
      "input_double_register_7", "input_double_register_8", "input_double_register_9", "input_double_register_10",
      "input_double_register_11", "input_double_register_12", "input_double_register_13"};
  rtde_->sendInputSetup(get_inverse_kin_input);

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>();

  // Wait until RTDE data synchronization has started.
  std::cout << "Waiting for RTDE data synchronization to start..." << std::endl;
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  // Start RTDE data synchronization
  rtde_->sendStart();

  while(!rtde_->isStarted())
  {
    // Wait until RTDE data synchronization has started or timeout
    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > RTDE_START_SYNCHRONIZATION_TIMEOUT)
    {
      break;
    }
  }

  if (!rtde_->isStarted())
    throw std::logic_error("Failed to start RTDE data synchronization, before timeout");

  // Start executing receiveCallback
  th_ = std::make_shared<boost::thread>(boost::bind(&RTDEControlInterface::receiveCallback, this));

  // Wait until the first robot state has been received
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Clear command register
  sendClearCommand();

  if (!isProgramRunning())
  {
    // Send script to the UR Controller
    script_client_->sendScript();
  }
  else
  {
    std::cout << "A script was running on the controller, killing it!" << std::endl;
    // Stop the running script first
    stopRobot();
    db_client_->stop();

    // Wait until terminated
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Send script to the UR Controller
    script_client_->sendScript();
  }
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

  // Stop the receive callback function
  stop_thread = true;
  th_->interrupt();
  th_->join();
}

bool RTDEControlInterface::isConnected()
{
  return rtde_->isConnected();
}

bool RTDEControlInterface::reconnect()
{
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  auto controller_version = rtde_->getControllerVersion();
  uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

  double frequency = 125;
  // If e-Series Robot set frequency to 500Hz
  if (major_version > CB3_MAJOR_VERSION)
    frequency = 500;

  // Setup output
  std::vector<std::string> state_names = {
      "robot_status_bits",        "output_int_register_0",    "output_int_register_1",
      "output_double_register_0", "output_double_register_1", "output_double_register_2",
      "output_double_register_3", "output_double_register_4", "output_double_register_5"};
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

  // Recipe 6
  std::vector<std::string> servoc_input = {
      "input_int_register_0",    "input_double_register_0", "input_double_register_1", "input_double_register_2",
      "input_double_register_3", "input_double_register_4", "input_double_register_5", "input_double_register_6",
      "input_double_register_7", "input_double_register_8"};
  rtde_->sendInputSetup(servoc_input);

  // Recipe 7
  std::vector<std::string> wrench_input = {
      "input_int_register_0",    "input_double_register_0", "input_double_register_1", "input_double_register_2",
      "input_double_register_3", "input_double_register_4", "input_double_register_5"};
  rtde_->sendInputSetup(wrench_input);

  // Recipe 8
  std::vector<std::string> set_payload_input = {"input_int_register_0", "input_double_register_0",
                                                "input_double_register_1", "input_double_register_2",
                                                "input_double_register_3"};
  rtde_->sendInputSetup(set_payload_input);

  // Recipe 9
  std::vector<std::string> force_mode_parameters_input = {"input_int_register_0", "input_double_register_0"};
  rtde_->sendInputSetup(force_mode_parameters_input);

  // Recipe 10
  std::vector<std::string> get_actual_joint_positions_history_input = {"input_int_register_0", "input_int_register_1"};
  rtde_->sendInputSetup(get_actual_joint_positions_history_input);

  // Recipe 11
  std::vector<std::string> get_inverse_kin_input = {
      "input_int_register_0", "input_double_register_0", "input_double_register_1", "input_double_register_2",
      "input_double_register_3", "input_double_register_4", "input_double_register_5", "input_double_register_6",
      "input_double_register_7", "input_double_register_8", "input_double_register_9", "input_double_register_10",
      "input_double_register_11", "input_double_register_12", "input_double_register_13"};
  rtde_->sendInputSetup(get_inverse_kin_input);

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>();

  // Wait until RTDE data synchronization has started.
  std::cout << "Waiting for RTDE data synchronization to start..." << std::endl;
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  // Start RTDE data synchronization
  rtde_->sendStart();

  while(!rtde_->isStarted())
  {
    // Wait until RTDE data synchronization has started or timeout
    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > RTDE_START_SYNCHRONIZATION_TIMEOUT)
    {
      break;
    }
  }

  if (!rtde_->isStarted())
    throw std::logic_error("Failed to start RTDE data synchronization, before timeout");

  // Start executing receiveCallback
  th_ = std::make_shared<boost::thread>(boost::bind(&RTDEControlInterface::receiveCallback, this));

  // Wait until the first robot state has been received
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Clear command register
  sendClearCommand();

  if (!isProgramRunning())
  {
    // Send script to the UR Controller
    script_client_->sendScript();
  }
  else
  {
    std::cout << "A script was running on the controller, killing it!" << std::endl;
    // Stop the running script first
    stopRobot();
    db_client_->stop();

    // Wait until terminated
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Send script to the UR Controller
    script_client_->sendScript();
  }

  return true;
}

void RTDEControlInterface::receiveCallback()
{
  while (!stop_thread)
  {
    // Receive and update the robot state
    try
    {
      rtde_->receiveData(robot_state_);
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      if (rtde_->isConnected())
        rtde_->disconnect();
      stop_thread = true;
      th_->interrupt();
      th_->join();
    }
  }
}

void RTDEControlInterface::stopRobot()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  sendCommand(robot_cmd);
}

bool RTDEControlInterface::reuploadScript()
{
  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();
  db_client_->popup("The RTDE Control script has been re-uploaded due to an error.");
  return true;
}

bool RTDEControlInterface::sendCustomScriptFunction(const std::string &function_name, const std::string &script)
{
  std::string cmd_str;
  std::string line;
  std::stringstream ss(script);
  cmd_str += "def " + function_name + "():\n";
  cmd_str += "\twrite_output_integer_register(0, 0)\n";

  while (std::getline(ss, line))
  {
    cmd_str += "\t" + line + "\n";
  }

  // Signal when motions are finished
  cmd_str += "\twrite_output_integer_register(0, 1)\n";
  cmd_str += "end\n";

  // First stop the running RTDE control script
  stopRobot();

  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  // Send custom script function
  script_client_->sendScriptCommand(cmd_str);

  while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
  {
    // Wait until the controller is done with command
    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > UR_PATH_EXECUTION_TIMEOUT)
      return false;
  }

  sendClearCommand();

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();

  return true;
}

bool RTDEControlInterface::sendCustomScriptFile(const std::string &file_path)
{
  // First stop the running RTDE control script
  stopRobot();

  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  // Send custom script file
  script_client_->sendScript(file_path);

  while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
  {
    // Wait until the controller is done with command
    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > UR_PATH_EXECUTION_TIMEOUT)
      return false;
  }

  sendClearCommand();

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();
  return true;
}

void RTDEControlInterface::verifyValueIsWithin(const double &value, const double &min, const double &max)
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

std::string RTDEControlInterface::prepareCmdScript(const std::vector<std::vector<double>> &path, const std::string &cmd)
{
  std::string cmd_str;
  std::stringstream ss;
  cmd_str += "def motions():\n";
  cmd_str += "\twrite_output_integer_register(0, 0)\n";
  for (const auto &pose : path)
  {
    if(cmd =="movej(")
    {
      verifyValueIsWithin(pose[6], UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
      verifyValueIsWithin(pose[7], UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);
      verifyValueIsWithin(pose[8], UR_BLEND_MIN, UR_BLEND_MAX);
    }
    else if(cmd =="movel(p")
    {
      verifyValueIsWithin(pose[6], UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
      verifyValueIsWithin(pose[7], UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);
      verifyValueIsWithin(pose[8], UR_BLEND_MIN, UR_BLEND_MAX);
    }
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

bool RTDEControlInterface::moveJ(const std::vector<std::vector<double>> &path)
{
  // First stop the running RTDE control script
  stopRobot();

  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  // Send motions
  script_client_->sendScriptCommand(prepareCmdScript(path, "movej("));

  while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
  {
    // Wait until the controller is done with command
    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > UR_PATH_EXECUTION_TIMEOUT)
      return false;
  }

  sendClearCommand();

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();
  return true;
}

bool RTDEControlInterface::moveJ(const std::vector<double> &q, double speed, double acceleration)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = q;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveJ_IK(const std::vector<double> &transform, double speed, double acceleration)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEJ_IK;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = transform;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveL(const std::vector<std::vector<double>> &path)
{
  // First stop the running RTDE control script
  stopRobot();

  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
  // Send motions
  script_client_->sendScriptCommand(prepareCmdScript(path, "movel(p"));

  while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
  {
    // Wait until the controller is done with command or timeout
    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > UR_PATH_EXECUTION_TIMEOUT)
      return false;
  }

  sendClearCommand();

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();
  return true;
}

bool RTDEControlInterface::moveL(const std::vector<double> &transform, double speed, double acceleration)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = transform;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveL_FK(const std::vector<double> &q, double speed, double acceleration)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEL_FK;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = q;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveC(const std::vector<double> &pose_via, const std::vector<double> &pose_to, double speed,
                                 double acceleration, int mode)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEC;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_2;
  robot_cmd.val_ = pose_via;
  for (const auto &val : pose_to)
    robot_cmd.val_.push_back(val);

  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.movec_mode_ = mode;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceModeStart(const std::vector<double> &task_frame,
                                          const std::vector<int> &selection_vector, const std::vector<double> &wrench,
                                          int type, const std::vector<double> &limits)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_START;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  robot_cmd.val_ = task_frame;
  for (const auto &val : wrench)
    robot_cmd.val_.push_back(val);

  for (const auto &val : limits)
    robot_cmd.val_.push_back(val);

  robot_cmd.selection_vector_ = selection_vector;
  robot_cmd.force_mode_type_ = type;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceModeUpdate(const std::vector<double> &wrench)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_UPDATE;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_7;
  robot_cmd.val_ = wrench;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceModeStop()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::zeroFtSensor()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::ZERO_FT_SENSOR;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::speedJ(const std::vector<double> &qd, double acceleration, double time)
{
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SPEEDJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = qd;
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(time);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::speedL(const std::vector<double> &xd, double acceleration, double time)
{
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SPEEDL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = xd;
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(time);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::servoJ(const std::vector<double> &q, double speed, double acceleration, double time,
                                  double lookahead_time, double gain)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);
  verifyValueIsWithin(lookahead_time, UR_SERVO_LOOKAHEAD_TIME_MIN, UR_SERVO_LOOKAHEAD_TIME_MAX);
  verifyValueIsWithin(gain, UR_SERVO_GAIN_MIN, UR_SERVO_GAIN_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SERVOJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_3;
  robot_cmd.val_ = q;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(time);
  robot_cmd.val_.push_back(lookahead_time);
  robot_cmd.val_.push_back(gain);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::servoL(const std::vector<double> &pose, double speed, double acceleration, double time,
                                  double lookahead_time, double gain)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);
  verifyValueIsWithin(lookahead_time, UR_SERVO_LOOKAHEAD_TIME_MIN, UR_SERVO_LOOKAHEAD_TIME_MAX);
  verifyValueIsWithin(gain, UR_SERVO_GAIN_MIN, UR_SERVO_GAIN_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SERVOL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_3;
  robot_cmd.val_ = pose;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(time);
  robot_cmd.val_.push_back(lookahead_time);
  robot_cmd.val_.push_back(gain);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::speedStop()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SPEED_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::servoStop()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SERVO_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::servoC(const std::vector<double> &pose, double speed, double acceleration, double blend)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);
  verifyValueIsWithin(blend, UR_BLEND_MIN, UR_BLEND_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SERVOC;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_6;
  robot_cmd.val_ = pose;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(blend);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::setPayload(double mass, const std::vector<double> &cog)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_PAYLOAD;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_8;
  robot_cmd.val_.push_back(mass);
  if (!cog.empty())
  {
    for (const auto &val : cog)
      robot_cmd.val_.push_back(val);
  }
  else
  {
    robot_cmd.val_.push_back(0);
    robot_cmd.val_.push_back(0);
    robot_cmd.val_.push_back(0);
  }
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::teachMode()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::TEACH_MODE;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::endTeachMode()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::END_TEACH_MODE;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceModeSetDamping(double damping)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_SET_DAMPING;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_9;
  robot_cmd.val_.push_back(damping);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceModeSetGainScaling(double scaling)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_SET_GAIN_SCALING;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_9;
  robot_cmd.val_.push_back(scaling);
  return sendCommand(robot_cmd);
}

int RTDEControlInterface::toolContact(const std::vector<double> &direction)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::TOOL_CONTACT;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_7;
  robot_cmd.val_ = direction;
  if (sendCommand(robot_cmd))
  {
    return getToolContactValue();
  }
  else
  {
    return 0;
  }
}

double RTDEControlInterface::getStepTime()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_STEPTIME;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  if (sendCommand(robot_cmd))
  {
    return getStepTimeValue();
  }
  else
  {
    return 0;
  }
}

std::vector<double> RTDEControlInterface::getActualJointPositionsHistory(int steps)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_ACTUAL_JOINT_POSITIONS_HISTORY;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_10;
  robot_cmd.steps_ = steps;
  if (sendCommand(robot_cmd))
  {
    return getActualJointPositionsHistoryValue();
  }
  else
  {
    return std::vector<double>();
  }
}

std::vector<double> RTDEControlInterface::getTargetWaypoint()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_TARGET_WAYPOINT;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  if (sendCommand(robot_cmd))
  {
    return getTargetWaypointValue();
  }
  else
  {
    return std::vector<double>();
  }
}

bool RTDEControlInterface::isProgramRunning()
{
  if (robot_state_ != nullptr)
  {
    // Read Bits 0-3: Is power on(1) | Is program running(2) | Is teach button pressed(4) | Is power button pressed(8)
    std::bitset<sizeof(uint32_t)> status_bits(robot_state_->getRobot_status());
    return status_bits.test(RobotStatus::ROBOT_STATUS_PROGRAM_RUNNING);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

double RTDEControlInterface::getStepTimeValue()
{
  if (robot_state_ != nullptr)
  {
    return robot_state_->getOutput_double_register_0();
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

int RTDEControlInterface::getToolContactValue()
{
  if (robot_state_ != nullptr)
  {
    return robot_state_->getOutput_int_register_1();
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

std::vector<double> RTDEControlInterface::getTargetWaypointValue()
{
  if (robot_state_ != nullptr)
  {
    std::vector<double> target_waypoint = {
        robot_state_->getOutput_double_register_0(), robot_state_->getOutput_double_register_1(),
        robot_state_->getOutput_double_register_2(), robot_state_->getOutput_double_register_3(),
        robot_state_->getOutput_double_register_4(), robot_state_->getOutput_double_register_5()};
    return target_waypoint;
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

std::vector<double> RTDEControlInterface::getActualJointPositionsHistoryValue()
{
  if (robot_state_ != nullptr)
  {
    std::vector<double> actual_joint_positions_history = {
        robot_state_->getOutput_double_register_0(), robot_state_->getOutput_double_register_1(),
        robot_state_->getOutput_double_register_2(), robot_state_->getOutput_double_register_3(),
        robot_state_->getOutput_double_register_4(), robot_state_->getOutput_double_register_5()};
    return actual_joint_positions_history;
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

std::vector<double> RTDEControlInterface::getInverseKinematicsValue()
{
  if (robot_state_ != nullptr)
  {
    std::vector<double> q = {
        robot_state_->getOutput_double_register_0(), robot_state_->getOutput_double_register_1(),
        robot_state_->getOutput_double_register_2(), robot_state_->getOutput_double_register_3(),
        robot_state_->getOutput_double_register_4(), robot_state_->getOutput_double_register_5()};
    return q;
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTDEControlInterface::setTcp(const std::vector<double> &tcp_offset)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_TCP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_7;
  robot_cmd.val_ = tcp_offset;
  return sendCommand(robot_cmd);
}

std::vector<double> RTDEControlInterface::getInverseKinematics(const std::vector<double> &x,
                                                const std::vector<double> &qnear,
                                                double max_position_error,
                                                double max_orientation_error){
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_INVERSE_KINEMATICS;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_11;
  robot_cmd.val_ = x;
  robot_cmd.val_.insert(robot_cmd.val_.end(), qnear.begin(), qnear.end());
  robot_cmd.val_.push_back(max_position_error);
  robot_cmd.val_.push_back(max_orientation_error);
  if (sendCommand(robot_cmd))
  {
    return getInverseKinematicsValue();
  }
  else
  {
    return std::vector<double>();
  }
}

int RTDEControlInterface::getControlScriptState()
{
  if (robot_state_ != nullptr)
  {
    return robot_state_->getOutput_int_register_0();
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTDEControlInterface::triggerProtectiveStop()
{
    RTDE::RobotCommand robot_cmd;
    robot_cmd.type_ = RTDE::RobotCommand::Type::PROTECTIVE_STOP;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
    return sendCommand(robot_cmd);
}

bool RTDEControlInterface::sendCommand(const RTDE::RobotCommand &cmd)
{
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  try
  {
    while (getControlScriptState() != UR_CONTROLLER_RDY_FOR_CMD)
    {
      // Wait until the controller is ready for a command or timeout
      std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
      if (duration > UR_GET_READY_TIMEOUT)
        return false;
    }

    // Send command to the controller
    rtde_->send(cmd);

    start_time = std::chrono::high_resolution_clock::now();
    while (getControlScriptState() != UR_CONTROLLER_CMD_RECEIVED)
    {
      // Wait until the controller has received the command or timeout
      std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
      if (duration > UR_CMD_RECEIVE_TIMEOUT)
        return false;
    }

    // Command has been received stop sending it.
    sendClearCommand();

    if (cmd.type_ != RTDE::RobotCommand::Type::STOP)
    {
      start_time = std::chrono::high_resolution_clock::now();
      while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
      {
        // Wait until the controller has finished executing or timeout
        std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (duration > UR_EXECUTION_TIMEOUT)
          return false;
      }
    }

    // Command has been received stop sending it.
    sendClearCommand();
    return true;
  }
  catch (std::exception &e)
  {
    std::cout << "RTDEControlInterface: Lost connection to robot..." << std::endl;
    std::cerr << e.what() << std::endl;
    if (rtde_ != nullptr)
    {
      if (rtde_->isConnected())
        rtde_->disconnect();
    }
  }

  if (!rtde_->isConnected())
  {
    std::cout << "RTDEControlInterface: Robot is disconnected, reconnecting..." << std::endl;
    reconnect();
    return sendCommand(cmd);
  }
  return false; 
}

void RTDEControlInterface::sendClearCommand()
{
  RTDE::RobotCommand clear_cmd;
  clear_cmd.type_ = RTDE::RobotCommand::Type::NO_CMD;
  clear_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  rtde_->send(clear_cmd);
}

}  // namespace ur_rtde
