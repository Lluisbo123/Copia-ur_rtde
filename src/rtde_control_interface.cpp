#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/robot_state.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/script_client.h>

#include <bitset>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <iostream>
#include <thread>

namespace ur_rtde
{
RTDEControlInterface::RTDEControlInterface(std::string hostname, int port, bool verbose)
    : hostname_(std::move(hostname)), port_(port), verbose_(verbose)
{
  // Create a connection to the dashboard server
  db_client_ = std::make_shared<DashboardClient>(hostname_);
  db_client_->connect();

  // Only check if in remote on real robot.
  if (hostname_ != "localhost" && hostname_ != "127.0.0.1")
  {
    PolyScopeVersion polyscope_version(db_client_->polyscopeVersion());
    if (polyscope_version.major == 5 && polyscope_version.minor > 5)
    {
      // Check if robot is in remote control
      if (!db_client_->isInRemoteControl())
      {
        throw std::logic_error("ur_rtde: Please enable remote control on the robot!");
      }
    }
  }

  rtde_ = std::make_shared<RTDE>(hostname_, port_, verbose_);
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  auto controller_version = rtde_->getControllerVersion();
  uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);
  uint32_t minor_version = std::get<MINOR_VERSION>(controller_version);

  frequency_ = 125;
  // If e-Series Robot set frequency to 500Hz
  if (major_version > CB3_MAJOR_VERSION)
    frequency_ = 500;

  // Set delta time to be used by receiveCallback
  delta_time_ = 1 / frequency_;

  // Create a connection to the script server
  script_client_ = std::make_shared<ScriptClient>(hostname_, major_version, minor_version);
  script_client_->connect();

  // Setup default recipes
  setupRecipes(frequency_);

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>();

  // Wait until RTDE data synchronization has started
  if (verbose_)
    std::cout << "Waiting for RTDE data synchronization to start..." << std::endl;
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  // Start RTDE data synchronization
  rtde_->sendStart();

  while (!rtde_->isStarted())
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
    if (verbose_)
      std::cout << "A script was running on the controller, killing it!" << std::endl;
    // Stop the running script first
    stopScript();
    db_client_->stop();

    // Wait until terminated
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Send script to the UR Controller
    script_client_->sendScript();
  }
}

RTDEControlInterface::~RTDEControlInterface()
{
  disconnect();
}

void RTDEControlInterface::disconnect()
{
  // Stop the receive callback function
  stop_thread_ = true;
  th_->interrupt();
  th_->join();

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

  // Wait until everything has disconnected
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

bool RTDEControlInterface::isConnected()
{
  return rtde_->isConnected();
}

bool RTDEControlInterface::reconnect()
{
  db_client_->connect();
  script_client_->connect();
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  auto controller_version = rtde_->getControllerVersion();
  uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

  frequency_ = 125;
  // If e-Series Robot set frequency to 500Hz
  if (major_version > CB3_MAJOR_VERSION)
    frequency_ = 500;

  // Set delta time to be used by receiveCallback
  delta_time_ = 1 / frequency_;

  // Setup default recipes
  setupRecipes(frequency_);

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>();

  // Wait until RTDE data synchronization has started.
  if (verbose_)
    std::cout << "Waiting for RTDE data synchronization to start..." << std::endl;
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  // Start RTDE data synchronization
  rtde_->sendStart();

  while (!rtde_->isStarted())
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
  stop_thread_ = false;
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
    if (verbose_)
      std::cout << "A script was running on the controller, killing it!" << std::endl;
    // Stop the running script first
    stopScript();
    db_client_->stop();

    // Wait until terminated
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Send script to the UR Controller
    script_client_->sendScript();
  }

  return true;
}

bool RTDEControlInterface::setupRecipes(const double &frequency)
{
  // Setup output
  std::vector<std::string> state_names = {
      "robot_status_bits",        "safety_status_bits",       "output_int_register_0",    "output_int_register_1",
      "output_double_register_0", "output_double_register_1", "output_double_register_2", "output_double_register_3",
      "output_double_register_4", "output_double_register_5"};
  rtde_->sendOutputSetup(state_names, frequency);

  // Setup input recipes
  // Recipe 1
  std::vector<std::string> async_setp_input = {
      "input_int_register_0",    "input_double_register_0", "input_double_register_1", "input_double_register_2",
      "input_double_register_3", "input_double_register_4", "input_double_register_5", "input_double_register_6",
      "input_double_register_7", "input_int_register_1"};
  rtde_->sendInputSetup(async_setp_input);

  // Recipe 2
  std::vector<std::string> movec_input = {
      "input_int_register_0",     "input_double_register_0",  "input_double_register_1",  "input_double_register_2",
      "input_double_register_3",  "input_double_register_4",  "input_double_register_5",  "input_double_register_6",
      "input_double_register_7",  "input_double_register_8",  "input_double_register_9",  "input_double_register_10",
      "input_double_register_11", "input_double_register_12", "input_double_register_13", "input_double_register_14",
      "input_int_register_1"};
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
      "input_int_register_0",     "input_double_register_0",  "input_double_register_1", "input_double_register_2",
      "input_double_register_3",  "input_double_register_4",  "input_double_register_5", "input_double_register_6",
      "input_double_register_7",  "input_double_register_8",  "input_double_register_9", "input_double_register_10",
      "input_double_register_11", "input_double_register_12", "input_double_register_13"};
  rtde_->sendInputSetup(get_inverse_kin_input);

  // Recipe 12
  std::vector<std::string> watchdog_input = {"input_int_register_0"};
  rtde_->sendInputSetup(watchdog_input);

  // Recipe 13
  std::vector<std::string> pose_trans_input = {
      "input_int_register_0",    "input_double_register_0", "input_double_register_1", "input_double_register_2",
      "input_double_register_3", "input_double_register_4", "input_double_register_5", "input_double_register_6",
      "input_double_register_7", "input_double_register_8", "input_double_register_9", "input_double_register_10",
      "input_double_register_11"};
  rtde_->sendInputSetup(pose_trans_input);

  // Recipe 14
  std::vector<std::string> setp_input = {
      "input_int_register_0",    "input_double_register_0", "input_double_register_1",
      "input_double_register_2", "input_double_register_3", "input_double_register_4",
      "input_double_register_5", "input_double_register_6", "input_double_register_7"};
  rtde_->sendInputSetup(setp_input);

  // Recipe 15
  std::vector<std::string> jog_input = {"input_int_register_0",    "input_double_register_0", "input_double_register_1",
                                        "input_double_register_2", "input_double_register_3", "input_double_register_4",
                                        "input_double_register_5", "input_double_register_6"};
  rtde_->sendInputSetup(jog_input);

  // Recipe 16
  std::vector<std::string> async_path_input = {"input_int_register_0", "input_int_register_1"};
  rtde_->sendInputSetup(async_path_input);

  return true;
}

void RTDEControlInterface::receiveCallback()
{
  while (!stop_thread_)
  {
    // Receive and update the robot state
    try
    {
      rtde_->receiveData(robot_state_);
      // temporary hack to fix synchronization problems on windows.
#ifndef _WIN32
      std::this_thread::sleep_for(std::chrono::microseconds(100));
#endif
    }
    catch (std::exception &e)
    {
      std::cerr << "RTDEControlInterface: Could not receive data from robot..." << std::endl;
      std::cerr << e.what() << std::endl;
      if (rtde_ != nullptr)
      {
        if (rtde_->isConnected())
          rtde_->disconnect();

        if (!rtde_->isConnected())
        {
          std::cerr << "RTDEControlInterface: Robot is disconnected, reconnecting..." << std::endl;
          reconnect();
        }

        if (rtde_->isConnected())
          std::cout << "RTDEControlInterface: Successfully reconnected!" << std::endl;
        else
          throw std::runtime_error("Could not recover from losing connection to robot!");
      }
    }
  }
}

void RTDEControlInterface::stopScript()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::STOP_SCRIPT;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  sendCommand(robot_cmd);
}

void RTDEControlInterface::stopL(double a)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::STOPL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_9;
  robot_cmd.val_.push_back(a);
  sendCommand(robot_cmd);
}

void RTDEControlInterface::stopJ(double a)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::STOPJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_9;
  robot_cmd.val_.push_back(a);
  sendCommand(robot_cmd);
}

bool RTDEControlInterface::reuploadScript()
{
  if (isProgramRunning())
  {
    if (verbose_)
      std::cout << "A script was running on the controller, killing it!" << std::endl;

    // Stop the running script first
    stopScript();
    db_client_->stop();

    // Wait until terminated
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Re-upload RTDE script to the UR Controller
  if (script_client_->sendScript())
  {
    db_client_->popup("The RTDE Control script has been re-uploaded due to an error.");
    if (verbose_)
      std::cout << "The RTDE Control script has been re-uploaded." << std::endl;
    return true;
  }
  else
  {
    return false;
  }
}

bool RTDEControlInterface::sendCustomScriptFunction(const std::string &function_name, const std::string &script)
{
  std::string cmd_str;
  std::string line;
  std::stringstream ss(script);
  cmd_str += "def " + function_name + "():\n";
  cmd_str += "\twrite_output_integer_register(0, 1)\n";

  while (std::getline(ss, line))
  {
    cmd_str += "\t" + line + "\n";
  }

  // Signal when motions are finished
  cmd_str += "\twrite_output_integer_register(0, 2)\n";
  cmd_str += "end\n";

  return sendCustomScript(cmd_str);
}

bool RTDEControlInterface::sendCustomScript(const std::string &script)
{
  // First stop the running RTDE control script
  stopScript();

  auto start_time = std::chrono::high_resolution_clock::now();

  // Send custom script function
  script_client_->sendScriptCommand(script);

  while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
  {
    // Wait until the controller is done with command
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > UR_PATH_EXECUTION_TIMEOUT)
      return false;
    // Sleep to avoid high CPU load
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  sendClearCommand();

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();

  return true;
}

bool RTDEControlInterface::sendCustomScriptFile(const std::string &file_path)
{
  // First stop the running RTDE control script
  stopScript();

  auto start_time = std::chrono::high_resolution_clock::now();

  // Send custom script file
  script_client_->sendScript(file_path);

  while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
  {
    // Wait until the controller is done with command
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > UR_PATH_EXECUTION_TIMEOUT)
      return false;
    // Sleep to avoid high CPU load
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  sendClearCommand();

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();
  return true;
}

RTDE_EXPORT void RTDEControlInterface::setCustomScriptFile(const std::string &file_path)
{
  script_client_->setScriptFile(file_path);
  reuploadScript();
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

std::string RTDEControlInterface::buildPathScriptCode(const std::vector<std::vector<double>> &path,
                                                      const std::string &cmd)
{
  std::stringstream ss;
  for (const auto &pose : path)
  {
    if (cmd == "movej(")
    {
      verifyValueIsWithin(pose[6], UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
      verifyValueIsWithin(pose[7], UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);
      verifyValueIsWithin(pose[8], UR_BLEND_MIN, UR_BLEND_MAX);
    }
    else if (cmd == "movel(p")
    {
      verifyValueIsWithin(pose[6], UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
      verifyValueIsWithin(pose[7], UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);
      verifyValueIsWithin(pose[8], UR_BLEND_MIN, UR_BLEND_MAX);
    }
    ss << "\t" << cmd << "[" << pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << ","
       << pose[5] << "],"
       << "a=" << pose[7] << ",v=" << pose[6] << ",r=" << pose[8] << ")\n";
  }
  return ss.str();
}

bool RTDEControlInterface::moveJ(const std::vector<std::vector<double>> &path, bool async)
{
  // This is the first step because it may throw an exception
  auto PathScript = buildPathScriptCode(path, "movej(");
  // stop the running RTDE control script
  stopScript();
  // now inject the movej path into the main UR script
  script_client_->setScriptInjection("# inject movej path\n", PathScript);
  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();

  // Now send the command
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEJ_PATH;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_16;
  robot_cmd.async_ = async ? 1 : 0;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveJ(const std::vector<double> &q, double speed, double acceleration, bool async)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  if (async)
    robot_cmd.async_ = 1;
  else
    robot_cmd.async_ = 0;
  robot_cmd.val_ = q;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveJ_IK(const std::vector<double> &transform, double speed, double acceleration, bool async)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEJ_IK;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  if (async)
    robot_cmd.async_ = 1;
  else
    robot_cmd.async_ = 0;
  robot_cmd.val_ = transform;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveL(const std::vector<std::vector<double>> &path, bool async)
{
  // This is the first step because it may throw an exception
  auto PathScript = buildPathScriptCode(path, "movel(p");
  // stop the running RTDE control script
  stopScript();
  // now inject the movel path into the main UR script
  script_client_->setScriptInjection("# inject movel path\n", PathScript);
  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();

  // Now send the command
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEL_PATH;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_16;
  robot_cmd.async_ = async ? 1 : 0;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveL(const std::vector<double> &transform, double speed, double acceleration, bool async)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  if (async)
    robot_cmd.async_ = 1;
  else
    robot_cmd.async_ = 0;
  robot_cmd.val_ = transform;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::jogStart(const std::vector<double> &speeds, int feature)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::JOG_START;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_15;
  robot_cmd.val_ = speeds;
  robot_cmd.val_.push_back(feature);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::jogStop()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::JOG_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveL_FK(const std::vector<double> &q, double speed, double acceleration, bool async)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEL_FK;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  if (async)
    robot_cmd.async_ = 1;
  else
    robot_cmd.async_ = 0;
  robot_cmd.val_ = q;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveC(const std::vector<double> &pose_via, const std::vector<double> &pose_to, double speed,
                                 double acceleration, double blend, int mode)
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
  robot_cmd.val_.push_back(blend);
  robot_cmd.movec_mode_ = mode;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveP(const std::vector<double> &pose, double speed, double acceleration, double blend)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);
  verifyValueIsWithin(blend, UR_BLEND_MIN, UR_BLEND_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_6;
  robot_cmd.val_ = pose;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(blend);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceMode(const std::vector<double> &task_frame, const std::vector<int> &selection_vector,
                                     const std::vector<double> &wrench, int type, const std::vector<double> &limits)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE;
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
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_14;
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
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_14;
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
    std::bitset<32> status_bits(robot_state_->getRobot_status());
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
    std::vector<double> q = {robot_state_->getOutput_double_register_0(), robot_state_->getOutput_double_register_1(),
                             robot_state_->getOutput_double_register_2(), robot_state_->getOutput_double_register_3(),
                             robot_state_->getOutput_double_register_4(), robot_state_->getOutput_double_register_5()};
    return q;
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

std::vector<double> RTDEControlInterface::poseTransValue()
{
  if (robot_state_ != nullptr)
  {
    std::vector<double> pose = {
        robot_state_->getOutput_double_register_0(), robot_state_->getOutput_double_register_1(),
        robot_state_->getOutput_double_register_2(), robot_state_->getOutput_double_register_3(),
        robot_state_->getOutput_double_register_4(), robot_state_->getOutput_double_register_5()};
    return pose;
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
                                                               double max_position_error, double max_orientation_error)
{
  RTDE::RobotCommand robot_cmd;
  if (!qnear.empty())
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_INVERSE_KINEMATICS_ARGS;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_11;
    robot_cmd.val_ = x;
    robot_cmd.val_.insert(robot_cmd.val_.end(), qnear.begin(), qnear.end());
    robot_cmd.val_.push_back(max_position_error);
    robot_cmd.val_.push_back(max_orientation_error);
  }
  else
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_INVERSE_KINEMATICS_DEFAULT;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_7;
    robot_cmd.val_ = x;
  }

  if (sendCommand(robot_cmd))
  {
    return getInverseKinematicsValue();
  }
  else
  {
    return std::vector<double>();
  }
}

std::vector<double> RTDEControlInterface::poseTrans(const std::vector<double> &p_from,
                                                    const std::vector<double> &p_from_to)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::POSE_TRANS;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_13;
  robot_cmd.val_ = p_from;
  robot_cmd.val_.insert(robot_cmd.val_.end(), p_from_to.begin(), p_from_to.end());
  if (sendCommand(robot_cmd))
  {
    return poseTransValue();
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

bool RTDEControlInterface::isProtectiveStopped()
{
  if (robot_state_ != nullptr)
  {
    std::bitset<32> safety_status_bits(robot_state_->getSafety_status_bits());
    return safety_status_bits.test(SafetyStatus::IS_PROTECTIVE_STOPPED);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTDEControlInterface::isEmergencyStopped()
{
  if (robot_state_ != nullptr)
  {
    std::bitset<32> safety_status_bits(robot_state_->getSafety_status_bits());
    return safety_status_bits.test(SafetyStatus::IS_EMERGENCY_STOPPED);
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

bool RTDEControlInterface::setWatchdog(double min_frequency)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_WATCHDOG;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_9;
  robot_cmd.val_.push_back(min_frequency);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::kickWatchdog()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::NO_CMD;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_12;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::isPoseWithinSafetyLimits(const std::vector<double> &pose)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::IS_POSE_WITHIN_SAFETY_LIMITS;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_7;
  robot_cmd.val_ = pose;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      return robot_state_->getOutput_int_register_1() == 1;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return false;
  }
}

bool RTDEControlInterface::isJointsWithinSafetyLimits(const std::vector<double> &q)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::IS_JOINTS_WITHIN_SAFETY_LIMITS;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_7;
  robot_cmd.val_ = q;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      return robot_state_->getOutput_int_register_1() == 1;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return false;
  }
}

std::vector<double> RTDEControlInterface::getJointTorques()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_JOINT_TORQUES;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      std::vector<double> torques = {
          robot_state_->getOutput_double_register_0(), robot_state_->getOutput_double_register_1(),
          robot_state_->getOutput_double_register_2(), robot_state_->getOutput_double_register_3(),
          robot_state_->getOutput_double_register_4(), robot_state_->getOutput_double_register_5()};
      return torques;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return std::vector<double>();
  }
}

std::vector<double> RTDEControlInterface::getTCPOffset()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_TCP_OFFSET;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      std::vector<double> tcp_offset = {
          robot_state_->getOutput_double_register_0(), robot_state_->getOutput_double_register_1(),
          robot_state_->getOutput_double_register_2(), robot_state_->getOutput_double_register_3(),
          robot_state_->getOutput_double_register_4(), robot_state_->getOutput_double_register_5()};
      return tcp_offset;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return std::vector<double>();
  }
}

std::vector<double> RTDEControlInterface::getForwardKinematics(const std::vector<double> &q,
                                                               const std::vector<double> &tcp_offset)
{
  RTDE::RobotCommand robot_cmd;
  if (q.empty() && tcp_offset.empty())
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_FORWARD_KINEMATICS_DEFAULT;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  }
  else if (tcp_offset.empty() && !q.empty())
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_FORWARD_KINEMATICS_ARGS;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_7;
    robot_cmd.val_ = q;
  }
  else
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_FORWARD_KINEMATICS_ARGS;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_13;
    robot_cmd.val_ = q;
    robot_cmd.val_.insert(robot_cmd.val_.end(), tcp_offset.begin(), tcp_offset.end());
  }

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      std::vector<double> forward_kin = {
          robot_state_->getOutput_double_register_0(), robot_state_->getOutput_double_register_1(),
          robot_state_->getOutput_double_register_2(), robot_state_->getOutput_double_register_3(),
          robot_state_->getOutput_double_register_4(), robot_state_->getOutput_double_register_5()};
      return forward_kin;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return std::vector<double>();
  }
}

bool RTDEControlInterface::isSteady()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::IS_STEADY;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      return robot_state_->getOutput_int_register_1() == 1;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return false;
  }
}

bool RTDEControlInterface::sendCommand(const RTDE::RobotCommand &cmd)
{
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  try
  {
    while (getControlScriptState() != UR_CONTROLLER_RDY_FOR_CMD)
    {
      // If robot is in an emergency or protective stop return false
      if (isProtectiveStopped() || isEmergencyStopped())
      {
        sendClearCommand();
        return false;
      }

      // Wait until the controller is ready for a command or timeout
      std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
      if (duration > UR_GET_READY_TIMEOUT)
      {
        sendClearCommand();
        return false;
      }
    }

    if (cmd.type_ == RTDE::RobotCommand::Type::SERVOJ || cmd.type_ == RTDE::RobotCommand::Type::SERVOL ||
        cmd.type_ == RTDE::RobotCommand::Type::SERVOC || cmd.type_ == RTDE::RobotCommand::Type::SPEEDJ ||
        cmd.type_ == RTDE::RobotCommand::Type::SPEEDL || cmd.type_ == RTDE::RobotCommand::Type::FORCE_MODE)
    {
      // Send command to the controller
      rtde_->send(cmd);

      // We do not wait for 'continuous' commands to finish.

      // Make controller ready for next command
      // sendClearCommand();

      return true;
    }
    else
    {
      // Send command to the controller
      rtde_->send(cmd);

      if (cmd.type_ != RTDE::RobotCommand::Type::STOP_SCRIPT)
      {
        start_time = std::chrono::high_resolution_clock::now();
        while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
        {
          // If robot is in an emergency or protective stop return false
          if (isProtectiveStopped() || isEmergencyStopped())
          {
            sendClearCommand();
            return false;
          }

          // Wait until the controller has finished executing or timeout
          auto current_time = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
          if (duration > UR_EXECUTION_TIMEOUT)
          {
            sendClearCommand();
            return false;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
      else
      {
        while (isProgramRunning())
        {
          // If robot is in an emergency or protective stop return false
          if (isProtectiveStopped() || isEmergencyStopped())
          {
            sendClearCommand();
            return false;
          }

          // Wait for program to stop running or timeout
          auto current_time = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
          if (duration > UR_EXECUTION_TIMEOUT)
          {
            sendClearCommand();
            return false;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }

      // Make controller ready for next command
      sendClearCommand();

      return true;
    }
  }
  catch (std::exception &e)
  {
    std::cerr << "RTDEControlInterface: Lost connection to robot..." << std::endl;
    std::cerr << e.what() << std::endl;
    if (rtde_ != nullptr)
    {
      if (rtde_->isConnected())
        rtde_->disconnect();
    }
  }

  if (!rtde_->isConnected())
  {
    std::cerr << "RTDEControlInterface: Robot is disconnected, reconnecting..." << std::endl;
    reconnect();
    return sendCommand(cmd);
  }
  sendClearCommand();
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
