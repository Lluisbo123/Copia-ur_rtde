#include <ur_rtde/robot_state.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_utility.h>

#include <boost/asio/connect.hpp>
#include <boost/asio/detail/socket_option.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/write.hpp>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>

const unsigned HEADER_SIZE = 3;
#define RTDE_PROTOCOL_VERSION 2
#define DEBUG_OUTPUT false

#if DEBUG_OUTPUT
#define DEBUG(a)                                                \
  {                                                             \
    std::cout << "RTDE:" << __LINE__ << ": " << a << std::endl; \
  }
#else
#define DEBUG(a) \
  {              \
  }
#endif

using boost::asio::ip::tcp;

namespace ur_rtde
{
RTDE::RTDE(const std::string hostname, int port)
    : hostname_(std::move(hostname)), port_(port), conn_state_(ConnectionState::DISCONNECTED)
{
  setupCallbacks();
}

RTDE::~RTDE() = default;

void RTDE::connect()
{
  try
  {
    io_service_ = std::make_shared<boost::asio::io_service>();
    socket_ = std::make_shared<boost::asio::ip::tcp::socket>(*io_service_);
    socket_->open(boost::asio::ip::tcp::v4());
    boost::asio::ip::tcp::no_delay no_delay_option(true);
    boost::asio::socket_base::reuse_address sol_reuse_option(true);
    socket_->set_option(no_delay_option);
    socket_->set_option(sol_reuse_option);
    resolver_ = std::make_shared<boost::asio::ip::tcp::resolver>(*io_service_);
    boost::asio::ip::tcp::resolver::query query(hostname_, std::to_string(port_));
    boost::asio::connect(*socket_, resolver_->resolve(query));
    conn_state_ = ConnectionState::CONNECTED;
    std::cout << "Connected successfully to: " << hostname_ << " at " << port_ << std::endl;
  }
  catch (boost::system::system_error const &e)
  {
    std::cout << "Warning: Could not connect to: " << hostname_ << " at " << port_ << ", verify the IP" << std::endl;
    throw;
  }
}

void RTDE::disconnect()
{
  // We rely on the socket_ destructor to do its job.
  conn_state_ = ConnectionState::DISCONNECTED;
  std::cout << "RTDE - Socket disconnected" << std::endl;
}

bool RTDE::isConnected()
{
  return conn_state_ == ConnectionState::CONNECTED || conn_state_ == ConnectionState::STARTED;
}

bool RTDE::isStarted()
{
  return conn_state_ == ConnectionState::STARTED;
}

bool RTDE::negotiateProtocolVersion()
{
  std::uint8_t cmd = RTDE_REQUEST_PROTOCOL_VERSION;
  // Pack RTDE_PROTOCOL_VERSION into payload
  uint8_t null_byte = 0;
  uint8_t version = RTDE_PROTOCOL_VERSION;
  std::vector<char> buffer;
  buffer.push_back(null_byte);
  buffer.push_back(version);
  std::string payload(buffer.begin(), buffer.end());
  sendAll(cmd, payload);
  DEBUG("Done sending RTDE_REQUEST_PROTOCOL_VERSION");
  receive();
  return true;
}

bool RTDE::sendInputSetup(const std::vector<std::string> &input_names)
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_SETUP_INPUTS;
  // Concatenate input_names to a single string
  std::string input_names_str;
  for (const auto &input_name : input_names)
    input_names_str += input_name + ",";
  sendAll(cmd, input_names_str);
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_SETUP_INPUTS");
  receive();
  return true;
}

bool RTDE::sendOutputSetup(const std::vector<std::string> &output_names, double frequency)
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS;

  // First save the output_names for use in the receiveData function
  output_names_ = output_names;

  std::string freq_as_hexstr = RTDEUtility::double2hexstr(frequency);
  std::vector<char> freq_packed = RTDEUtility::hexToBytes(freq_as_hexstr);
  // Concatenate output_names to a single string
  std::string output_names_str;
  for (const auto &output_name : output_names)
    output_names_str += output_name + ",";

  std::copy(output_names_str.begin(), output_names_str.end(), std::back_inserter(freq_packed));
  std::string payload(std::begin(freq_packed), std::end(freq_packed));
  sendAll(cmd, payload);
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS");
  receive();
  return true;
}

void RTDE::send(const RobotCommand &robot_cmd)
{
  std::uint8_t command = RTDE_DATA_PACKAGE;
  std::vector<char> cmd_packed = RTDEUtility::packInt32(robot_cmd.type_);

  if (robot_cmd.type_ == RobotCommand::FORCE_MODE_START)
  {
    std::vector<char> force_mode_type_packed = RTDEUtility::packInt32(robot_cmd.force_mode_type_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(force_mode_type_packed.begin()),
                      std::make_move_iterator(force_mode_type_packed.end()));

    std::vector<char> sel_vector_packed = RTDEUtility::packVectorNInt32(robot_cmd.selection_vector_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(sel_vector_packed.begin()),
                      std::make_move_iterator(sel_vector_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::GET_ACTUAL_JOINT_POSITIONS_HISTORY)
  {
    std::vector<char> actual_joint_positions_history_packed = RTDEUtility::packUInt32(robot_cmd.steps_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(actual_joint_positions_history_packed.begin()),
                      std::make_move_iterator(actual_joint_positions_history_packed.end()));
  }

  if (!robot_cmd.val_.empty())
  {
    std::vector<char> vector_nd_packed = RTDEUtility::packVectorNd(robot_cmd.val_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(vector_nd_packed.begin()),
                      std::make_move_iterator(vector_nd_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::MOVEC)
  {
    std::vector<char> movec_mode_packed = RTDEUtility::packInt32(robot_cmd.movec_mode_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(movec_mode_packed.begin()),
                      std::make_move_iterator(movec_mode_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_STD_DIGITAL_OUT)
  {
    cmd_packed.push_back(robot_cmd.std_digital_out_mask_);
    cmd_packed.push_back(robot_cmd.std_digital_out_);
  }

  if (robot_cmd.type_ == RobotCommand::SET_TOOL_DIGITAL_OUT)
  {
    cmd_packed.push_back(robot_cmd.std_tool_out_mask_);
    cmd_packed.push_back(robot_cmd.std_tool_out_);
  }

  if (robot_cmd.type_ == RobotCommand::SET_SPEED_SLIDER)
  {
    std::vector<char> speed_slider_mask_packed = RTDEUtility::packInt32(robot_cmd.speed_slider_mask_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(speed_slider_mask_packed.begin()),
                      std::make_move_iterator(speed_slider_mask_packed.end()));

    std::vector<char> speed_slider_fraction_packed = RTDEUtility::packDouble(robot_cmd.speed_slider_fraction_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(speed_slider_fraction_packed.begin()),
                      std::make_move_iterator(speed_slider_fraction_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_STD_ANALOG_OUT)
  {
    cmd_packed.push_back(robot_cmd.std_analog_output_mask_);
    cmd_packed.push_back(robot_cmd.std_analog_output_type_);
    std::vector<char> std_analog_output_0_packed = RTDEUtility::packDouble(robot_cmd.std_analog_output_0_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_analog_output_0_packed.begin()),
                      std::make_move_iterator(std_analog_output_0_packed.end()));
    std::vector<char> std_analog_output_1_packed = RTDEUtility::packDouble(robot_cmd.std_analog_output_1_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_analog_output_1_packed.begin()),
                      std::make_move_iterator(std_analog_output_1_packed.end()));
  }

  cmd_packed.insert(cmd_packed.begin(), robot_cmd.recipe_id_);
  std::string sent(cmd_packed.begin(), cmd_packed.end());

  sendAll(command, sent);
  DEBUG("Done sending RTDE_DATA_PACKAGE");
}

void RTDE::sendAll(const std::uint8_t &command, std::string payload)
{
  DEBUG("Payload size is: " << payload.size());
  // Pack size and command into header
  uint16_t size = htons(HEADER_SIZE + payload.size());
  uint8_t type = command;

  char buffer[3];
  memcpy(buffer + 0, &size, sizeof(size));
  memcpy(buffer + 2, &type, sizeof(type));

  // Create vector<char> that includes the header
  std::vector<char> header_packed;
  std::copy(buffer, buffer + sizeof(buffer), std::back_inserter(header_packed));

  // Add the payload to the header_packed vector
  std::copy(payload.begin(), payload.end(), std::back_inserter(header_packed));

  std::string sent(header_packed.begin(), header_packed.end());
  DEBUG("SENDING buf containing: " << sent << " with len: " << sent.size());

  boost::asio::write(*socket_, boost::asio::buffer(header_packed, header_packed.size()));
}

void RTDE::sendStart()
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_START;
  sendAll(cmd, "");
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_START");
  receive();
}

void RTDE::sendPause()
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_PAUSE;
  sendAll(cmd, "");
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_PAUSE");
  receive();
}

void RTDE::receive()
{
  DEBUG("Receiving...");
  // Read Header
  std::vector<char> data(HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));
  // DEBUG("Reply length is: " << reply_length);
  uint32_t message_offset = 0;
  uint16_t msg_size = RTDEUtility::getUInt16(data, message_offset);
  uint8_t msg_cmd = data.at(2);

  DEBUG("ControlHeader: ");
  DEBUG("size is: " << msg_size);
  DEBUG("command is: " << static_cast<int>(msg_cmd));

  // Read Body
  data.resize(msg_size - HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));

  switch (msg_cmd)
  {
    case RTDE_TEXT_MESSAGE:
    {
      uint8_t msg_length = data.at(0);
      for (int i = 1; i < msg_length; i++)
      {
        std::cout << data[i];
      }
      break;
    }

    case RTDE_REQUEST_PROTOCOL_VERSION:
    {
      break;
    }

    case RTDE_GET_URCONTROL_VERSION:
    {
      DEBUG("ControlVersion: ");
      // std::uint32_t message_offset = 0;
      // std::uint32_t v_major = RTDEUtility::getUInt32(data, message_offset);
      // std::uint32_t v_minor = RTDEUtility::getUInt32(data, message_offset);
      // std::uint32_t v_bugfix = RTDEUtility::getUInt32(data, message_offset);
      // std::uint32_t v_build = RTDEUtility::getUInt32(data, message_offset);
      // DEBUG(v_major << "." << v_minor << "." << v_bugfix << "." << v_build);
      break;
    }

    case RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
    {
      // char id = data.at(0);
      // DEBUG("ID:" << (int)id);
      std::string datatypes(std::begin(data) + 1, std::end(data));
      DEBUG("Datatype:" << datatypes);
      break;
    }

    case RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
    {
      // char id = data.at(0);
      // DEBUG("ID:" << id);
      std::string datatypes(std::begin(data) + 1, std::end(data));
      DEBUG("Datatype:" << datatypes);
      output_types_ = RTDEUtility::split(datatypes, ',');
      break;
    }

    case RTDE_CONTROL_PACKAGE_START:
    {
      char success = data.at(0);
      DEBUG("success: " << static_cast<bool>(success));
      auto rtde_success = static_cast<bool>(success);
      if (rtde_success)
      {
        conn_state_ = ConnectionState::STARTED;
        std::cout << "RTDE synchronization started" << std::endl;
      }
      else
        std::cerr << "Unable to start synchronization" << std::endl;
      break;
    }

    case RTDE_CONTROL_PACKAGE_PAUSE:
    {
      char success = data.at(0);
      auto pause_success = static_cast<bool>(success);
      DEBUG("success: " << pause_success);
      if (pause_success)
      {
        conn_state_ = ConnectionState::PAUSED;
        std::cout << "RTDE synchronization paused!" << std::endl;
      }
      else
        std::cerr << "Unable to pause synchronization" << std::endl;
      break;
    }

      // TODO: Handle IN_USE and NOT_FOUND case

    default:
      std::cout << "Unknown Command: " << static_cast<int>(msg_cmd) << std::endl;
      break;
  }
}

void RTDE::receiveData(std::shared_ptr<RobotState> &robot_state)
{
  DEBUG("Receiving...");
  // Read Header
  std::vector<char> data(HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));
  // DEBUG("Reply length is: " << reply_length);
  uint32_t message_offset = 0;
  uint16_t msg_size = RTDEUtility::getUInt16(data, message_offset);
  uint8_t msg_cmd = data.at(2);

  DEBUG("ControlHeader: ");
  DEBUG("size is: " << msg_size);
  DEBUG("command is: " << static_cast<int>(msg_cmd));

  // Read Body
  data.resize(msg_size - HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));

  switch (msg_cmd)
  {
    case RTDE_TEXT_MESSAGE:
    {
      uint8_t msg_length = data.at(0);
      for (int i = 1; i < msg_length; i++)
      {
        std::cout << data[i];
      }
      break;
    }

    case RTDE_DATA_PACKAGE:
    {
      // Read ID
      message_offset = 0;

      RTDEUtility::getUChar(data, message_offset);

      // Read all the variables specified by the user.
      for (const auto &output_name : output_names_)
      {
        // check if key exists
        if (cb_map_.count(output_name) > 0)
        {
          // call handling function
          cb_map_[output_name](robot_state, data, message_offset);
        }
        else
        {
          DEBUG("Unknown variable name: " << output_name << " please verify the output setup!");
        }
      }

      // TODO: Handle IN_USE and NOT_FOUND case

      break;
    }

    default:
      std::cout << "Unknown Command: " << static_cast<int>(msg_cmd) << std::endl;
      break;
  }
}

std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t> RTDE::getControllerVersion()
{
  std::uint8_t cmd = RTDE_GET_URCONTROL_VERSION;
  sendAll(cmd, "");
  DEBUG("Done sending RTDE_GET_URCONTROL_VERSION");
  std::vector<char> data(HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));
  uint32_t message_offset = 0;
  uint16_t msg_size = RTDEUtility::getUInt16(data, message_offset);
  uint8_t msg_cmd = data.at(2);
  // Read Body
  data.resize(msg_size - HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));

  if (msg_cmd == RTDE_GET_URCONTROL_VERSION)
  {
    message_offset = 0;
    std::uint32_t v_major = RTDEUtility::getUInt32(data, message_offset);
    std::uint32_t v_minor = RTDEUtility::getUInt32(data, message_offset);
    std::uint32_t v_bugfix = RTDEUtility::getUInt32(data, message_offset);
    std::uint32_t v_build = RTDEUtility::getUInt32(data, message_offset);
    DEBUG(v_major << "." << v_minor << "." << v_bugfix << "." << v_build);
    return std::make_tuple(v_major, v_minor, v_bugfix, v_build);
  }
  else
  {
    std::uint32_t v_major = 0;
    std::uint32_t v_minor = 0;
    std::uint32_t v_bugfix = 0;
    std::uint32_t v_build = 0;
    return std::make_tuple(v_major, v_minor, v_bugfix, v_build);
  }
}

namespace details
{
/*! @brief This function creates a callback map entry for a given key
  @tparam T Fully qualified type of the signature of the function to be called
  @tparam S Return type of the parsing function, should be of type T with equal or less qualifiers
  @param map A reference to the callback map
  @param key The key of the function callback
  @param fun A pointer to the robot state function that shall be called for the given key with data value T
  @param parse_fun A pointer to the parsing function, which will parse data and msg_offset to the data value S */
template <class T, class S>
void setupCallback(ur_rtde::details::cb_map &map, const std::string &key, void (ur_rtde::RobotState::*fun)(T),
                   S (*parse_fun)(const std::vector<char> &, uint32_t &))
{
  map.emplace(key, [fun, parse_fun](std::shared_ptr<ur_rtde::RobotState> state_ptr, const std::vector<char> &data,
                                    uint32_t &msg_offset) {
    // calls robot_state->setVarFun(RTDEUtility::parseVarFun(data,offset))
    (*state_ptr.*fun)((*parse_fun)(data, msg_offset));
  });
}

// helper makros to reduce the manually written code for registration of callbacks for output_registers
#define NUMBERED_REGISTER_NAME(type, num) "output_" #type "_register_" #num
#define NUMBERED_REGISTER_FUN(type, num) setOutput_##type##_register_##num

#define OUTPUT_REGISTER_CALLBACK(num)                                                                             \
  setupCallback(cb_map_, NUMBERED_REGISTER_NAME(int, num), &ur_rtde::RobotState::NUMBERED_REGISTER_FUN(int, num), \
                &RTDEUtility::getInt32);                                                                          \
  setupCallback(cb_map_, NUMBERED_REGISTER_NAME(double, num),                                                     \
                &ur_rtde::RobotState::NUMBERED_REGISTER_FUN(double, num), &RTDEUtility::getDouble);
}  // namespace details

void RTDE::setupCallbacks()
{
  using namespace ur_rtde::details;

  // general
  setupCallback(cb_map_, "timestamp", &ur_rtde::RobotState::setTimestamp, &RTDEUtility::getDouble);
  setupCallback(cb_map_, "actual_execution_time", &ur_rtde::RobotState::setActual_execution_time,
                &RTDEUtility::getDouble);
  setupCallback(cb_map_, "robot_mode", &ur_rtde::RobotState::setRobot_mode, &RTDEUtility::getInt32);
  setupCallback(cb_map_, "joint_mode", &ur_rtde::RobotState::setJoint_mode, &RTDEUtility::unpackVector6Int32);
  setupCallback(cb_map_, "safety_mode", &ur_rtde::RobotState::setSafety_mode, &RTDEUtility::getInt32);
  setupCallback(cb_map_, "runtime_state", &ur_rtde::RobotState::setRuntime_state, &RTDEUtility::getUInt32);

  // joint space
  setupCallback(cb_map_, "target_q", &ur_rtde::RobotState::setTarget_q, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "target_qd", &ur_rtde::RobotState::setTarget_qd, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "target_qdd", &ur_rtde::RobotState::setTarget_qdd, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "actual_q", &ur_rtde::RobotState::setActual_q, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "actual_qd", &ur_rtde::RobotState::setActual_qd, &RTDEUtility::unpackVector6d);

  // cartesian space
  setupCallback(cb_map_, "actual_TCP_pose", &ur_rtde::RobotState::setActual_TCP_pose, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "actual_TCP_speed", &ur_rtde::RobotState::setActual_TCP_speed, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "target_TCP_pose", &ur_rtde::RobotState::setTarget_TCP_pose, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "target_TCP_speed", &ur_rtde::RobotState::setTarget_TCP_speed, &RTDEUtility::unpackVector6d);

  // drives and control
  setupCallback(cb_map_, "joint_control_output", &ur_rtde::RobotState::setJoint_control_output,
                &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "joint_temperatures", &ur_rtde::RobotState::setJoint_temperatures,
                &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "speed_scaling", &ur_rtde::RobotState::setSpeed_scaling, &RTDEUtility::getDouble);
  setupCallback(cb_map_, "target_speed_fraction", &ur_rtde::RobotState::setTarget_speed_fraction,
                &RTDEUtility::getDouble);

  // currents and torque
  setupCallback(cb_map_, "target_current", &ur_rtde::RobotState::setTarget_current, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "actual_current", &ur_rtde::RobotState::setActual_current, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "target_moment", &ur_rtde::RobotState::setTarget_moment, &RTDEUtility::unpackVector6d);
  setupCallback(cb_map_, "actual_momentum", &ur_rtde::RobotState::setActual_momentum, &RTDEUtility::getDouble);
  setupCallback(cb_map_, "actual_main_voltage", &ur_rtde::RobotState::setActual_main_voltage, &RTDEUtility::getDouble);
  setupCallback(cb_map_, "actual_robot_voltage", &ur_rtde::RobotState::setActual_robot_voltage,
                &RTDEUtility::getDouble);
  setupCallback(cb_map_, "actual_robot_current", &ur_rtde::RobotState::setActual_robot_current,
                &RTDEUtility::getDouble);
  setupCallback(cb_map_, "actual_joint_voltage", &ur_rtde::RobotState::setActual_joint_voltage,
                &RTDEUtility::unpackVector6d);

  /* actual_tool_acc is the only function relying on unpackVec3 which can not be differentiated from unpackVec6
    by the templates of setupCallback() as both are type vec<double>. Therefore pass the parsing function manually (4th
    arg) Long term fix would be to change vec6 to arr6 and and vec3 to arr3 which makes them different types */
  setupCallback(cb_map_, "actual_tool_accelerometer", &ur_rtde::RobotState::setActual_tool_accelerometer,
                &RTDEUtility::unpackVector3d);

  // I/O
  setupCallback(cb_map_, "actual_digital_input_bits", &ur_rtde::RobotState::setActual_digital_input_bits,
                &RTDEUtility::getUInt64);
  setupCallback(cb_map_, "actual_digital_output_bits", &ur_rtde::RobotState::setActual_digital_output_bits,
                &RTDEUtility::getUInt64);
  setupCallback(cb_map_, "robot_status_bits", &ur_rtde::RobotState::setRobot_status, &RTDEUtility::getUInt32);
  setupCallback(cb_map_, "safety_status_bits", &ur_rtde::RobotState::setSafety_status_bits, &RTDEUtility::getUInt32);

  // io registers
  setupCallback(cb_map_, "standard_analog_input0", &ur_rtde::RobotState::setStandard_analog_input_0,
                &RTDEUtility::getDouble);
  setupCallback(cb_map_, "standard_analog_input1", &ur_rtde::RobotState::setStandard_analog_input_1,
                &RTDEUtility::getDouble);
  setupCallback(cb_map_, "standard_analog_output0", &ur_rtde::RobotState::setStandard_analog_output_0,
                &RTDEUtility::getDouble);
  setupCallback(cb_map_, "standard_analog_output1", &ur_rtde::RobotState::setStandard_analog_output_1,
                &RTDEUtility::getDouble);

  OUTPUT_REGISTER_CALLBACK(0)
  OUTPUT_REGISTER_CALLBACK(1)
  OUTPUT_REGISTER_CALLBACK(2)
  OUTPUT_REGISTER_CALLBACK(3)
  OUTPUT_REGISTER_CALLBACK(4)
  OUTPUT_REGISTER_CALLBACK(5)
  OUTPUT_REGISTER_CALLBACK(6)
  OUTPUT_REGISTER_CALLBACK(7)

  OUTPUT_REGISTER_CALLBACK(8)
  OUTPUT_REGISTER_CALLBACK(9)
  OUTPUT_REGISTER_CALLBACK(10)
  OUTPUT_REGISTER_CALLBACK(11)
  OUTPUT_REGISTER_CALLBACK(12)
  OUTPUT_REGISTER_CALLBACK(13)
  OUTPUT_REGISTER_CALLBACK(14)
  OUTPUT_REGISTER_CALLBACK(15)

  OUTPUT_REGISTER_CALLBACK(16)
  OUTPUT_REGISTER_CALLBACK(17)
  OUTPUT_REGISTER_CALLBACK(18)
  OUTPUT_REGISTER_CALLBACK(19)
  OUTPUT_REGISTER_CALLBACK(20)
  OUTPUT_REGISTER_CALLBACK(21)
  OUTPUT_REGISTER_CALLBACK(22)
  OUTPUT_REGISTER_CALLBACK(23)

  OUTPUT_REGISTER_CALLBACK(24)
  OUTPUT_REGISTER_CALLBACK(25)
  OUTPUT_REGISTER_CALLBACK(26)
  OUTPUT_REGISTER_CALLBACK(27)
  OUTPUT_REGISTER_CALLBACK(28)
  OUTPUT_REGISTER_CALLBACK(29)
  OUTPUT_REGISTER_CALLBACK(30)
  OUTPUT_REGISTER_CALLBACK(31)

  OUTPUT_REGISTER_CALLBACK(32)
  OUTPUT_REGISTER_CALLBACK(33)
  OUTPUT_REGISTER_CALLBACK(34)
  OUTPUT_REGISTER_CALLBACK(35)
  OUTPUT_REGISTER_CALLBACK(36)
  OUTPUT_REGISTER_CALLBACK(37)
  OUTPUT_REGISTER_CALLBACK(38)
  OUTPUT_REGISTER_CALLBACK(39)

  OUTPUT_REGISTER_CALLBACK(40)
  OUTPUT_REGISTER_CALLBACK(41)
  OUTPUT_REGISTER_CALLBACK(42)
  OUTPUT_REGISTER_CALLBACK(43)
  OUTPUT_REGISTER_CALLBACK(44)
  OUTPUT_REGISTER_CALLBACK(45)
  OUTPUT_REGISTER_CALLBACK(46)
  OUTPUT_REGISTER_CALLBACK(47)
}
}  // namespace ur_rtde
