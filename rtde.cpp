#include "rtde.h"
#include "rtde_utility.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <cstdint>
#include <cstdio>
#include <chrono>

#include <boost/numeric/conversion/cast.hpp>
#include <boost/asio.hpp>

const unsigned HEADER_SIZE = 3;
#define RTDE_PROTOCOL_VERSION 2
#define DEBUG_OUTPUT true

#if DEBUG_OUTPUT
#define DEBUG(a) {std::cout << "RTDE:" << __LINE__ << ": " << a << std::endl;}
#else
#define DEBUG(a) {}
#endif

using boost::asio::ip::tcp;

RTDE::RTDE(const std::string hostname, int port)
    : hostname_(std::move(hostname)), port_(port), conn_state_(ConnectionState::DISCONNECTED)
{
}

RTDE::~RTDE() = default;

void RTDE::connect()
{
  io_service_ = std::make_shared<boost::asio::io_service>();
  socket_ = std::make_shared<tcp::socket>(*io_service_);
  socket_->open(boost::asio::ip::tcp::v4());
  boost::asio::ip::tcp::no_delay no_delay_option(true);
  boost::asio::socket_base::reuse_address sol_reuse_option(true);
  socket_->set_option(no_delay_option);
  socket_->set_option(sol_reuse_option);
  resolver_ = std::make_shared<tcp::resolver>(*io_service_);
  tcp::resolver::query query(hostname_, std::to_string(port_));
  boost::asio::connect(*socket_, resolver_->resolve(query));
  conn_state_ = ConnectionState::CONNECTED;
  std::cout << "Connected successfully to: " << hostname_ << " at " << port_ << std::endl;
}

void RTDE::disconnect()
{
  // Close socket
  socket_->close();
  conn_state_ = ConnectionState::DISCONNECTED;
  std::cout << "RTDE -Socket disconnected" << std::endl;
}

bool RTDE::isConnected()
{
  if (conn_state_ != ConnectionState::DISCONNECTED)
    return true;
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
}

bool RTDE::sendInputSetup(const std::vector<std::string>& input_names)
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_SETUP_INPUTS;
  // Concatenate input_names to a single string
  std::string input_names_str;
  for (const auto &input_name : input_names)
    input_names_str += input_name+",";
  sendAll(cmd, input_names_str);
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_SETUP_INPUTS");
  receive();
}

bool RTDE::sendOutputSetup(const std::vector<std::string>& output_names, double frequency)
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS;

  // First save the output_names for use in the receiveData function
  output_names_ = output_names;

  std::string freq_as_hexstr = RTDEUtility::double2hexstr(frequency);
  std::vector<char> freq_packed = RTDEUtility::hexToBytes(freq_as_hexstr);
  // Concatenate output_names to a single string
  std::string output_names_str;
  for (const auto &output_name : output_names)
    output_names_str += output_name+",";

  std::copy(output_names_str.begin(), output_names_str.end(), std::back_inserter(freq_packed));
  std::string payload(std::begin(freq_packed), std::end(freq_packed));
  sendAll(cmd, payload);
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS");
  receive();
}

void RTDE::send(std::vector<double> vector_6d)
{
  std::uint8_t command = RTDE_DATA_PACKAGE;
  std::uint8_t recipe_id = 1;
  std::vector<char> vector_6d_packed = RTDEUtility::packVector6d(vector_6d);
  vector_6d_packed.insert(vector_6d_packed.begin(), recipe_id);
  // Pack size and command into header
  //uint16_t size = htons(HEADER_SIZE + vector_6d_packed.size());
  //uint8_t type = command;
  std::string sent(vector_6d_packed.begin(), vector_6d_packed.end());
  //RTDEUtility::hexDump(std::cout, sent.data(), sent.size());
  //boost::asio::write(*socket_, boost::asio::buffer(vector_6d_packed, vector_6d_packed.size()));
  sendAll(command, sent);
  DEBUG("Done sending RTDE_DATA_PACKAGE");
}

void RTDE::sendAll(const std::uint8_t& command, std::string payload)
{
  DEBUG("Payload size is: " << payload.size());
  // Pack size and command into header
  uint16_t size = htons(HEADER_SIZE + payload.size());
  uint8_t type = command;

  char buffer[3];
  memcpy(buffer+0, &size, sizeof(size));
  memcpy(buffer+2, &type, sizeof(type));

  // Create vector<char> that includes the header
  std::vector<char> header_packed;
  std::copy(buffer, buffer+sizeof(buffer), std::back_inserter(header_packed));

  // Add the payload to the header_packed vector
  std::copy(payload.begin(), payload.end(), std::back_inserter(header_packed));

  std::string sent(header_packed.begin(), header_packed.end());
  //hex_dump(std::cout, sent.data(), sent.size());
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
  size_t reply_length = boost::asio::read(*socket_, boost::asio::buffer(data));
  DEBUG("Reply length is: " << reply_length);
  uint32_t message_offset = 0;
  uint16_t msg_size = RTDEUtility::getUInt16(data, message_offset);
  uint8_t msg_cmd = data.at(2);

  DEBUG("ControlHeader: ");
  DEBUG("size is: " << msg_size);
  DEBUG("command is: " << static_cast<int>(msg_cmd));

  // Read Body
  data.resize(msg_size-HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));

  switch(msg_cmd)
  {
    case RTDE_REQUEST_PROTOCOL_VERSION:
    {

      break;
    }

    case RTDE_TEXT_MESSAGE:
    {
      uint8_t msg_length = data.at(0);
      for (int i = 1; i < msg_length; i++)
      {
        std::cout << data[i];
      }
      break;
    }

    case RTDE_GET_URCONTROL_VERSION:
    {
      DEBUG("ControlVersion: ");
      std::uint32_t message_offset = 0;
      std::uint32_t v_major = RTDEUtility::getUInt32(data, message_offset);
      std::uint32_t v_minor = RTDEUtility::getUInt32(data, message_offset);
      std::uint32_t v_bugfix = RTDEUtility::getUInt32(data, message_offset);
      std::uint32_t v_build = RTDEUtility::getUInt32(data, message_offset);
      DEBUG(v_major << "." << v_minor << "." << v_bugfix << "." << v_build);
      break;
    }

    case RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
    {
      char id = data.at(0);
      DEBUG("ID:" << (int)id);
      std::string datatypes(std::begin(data)+1, std::end(data));
      DEBUG("Datatype:" << datatypes);
      break;
    }

    case RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
    {
      char id = data.at(0);
      DEBUG("ID:" << id);
      std::string datatypes(std::begin(data)+1, std::end(data));
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

    /*case RTDE_DATA_PACKAGE:
    {
      // Read ID
      std::uint32_t message_offset = 0;
      unsigned char id = RTDEUtility::getUChar(data, message_offset);

      // Read all the datatypes reported by the controller.
      for (const auto &datatype : output_types_)
      {
        if(datatype == "VECTOR6D")
        {
          std::vector<double> vector_6d = RTDEUtility::unpackVector6d(data, message_offset);
          std::cout << datatype << ": ";
          for(const auto &d : vector_6d)
            std::cout << d << " ";
          std::cout << std::endl;
        }
        else if(datatype == "VECTOR6INT32")
        {
          std::vector<int32_t> vector_6_int32 = RTDEUtility::unpackVector6Int32(data, message_offset);
          std::cout << datatype << ": ";
          for(const auto &d : vector_6_int32)
            std::cout << d << " ";
          std::cout << std::endl;
        }
        else if (datatype == "VECTOR3D")
        {
          std::vector<double> vector_3d = RTDEUtility::unpackVector3d(data, message_offset);
          std::cout << datatype << ": ";
          for(const auto &d : vector_3d)
            std::cout << d << " ";
          std::cout << std::endl;
        }
        else if (datatype == "UINT32")
        {
          uint32_t uint32_value = RTDEUtility::getUInt32(data, message_offset);
          std::cout << datatype << ": " << uint32_value << std::endl;
        }
        else if (datatype == "INT32")
        {
          int32_t int32_value = RTDEUtility::getInt32(data, message_offset);
          std::cout << datatype << ": " << int32_value << std::endl;
        }
        else if (datatype == "UINT64")
        {
          uint64_t uint64_value = RTDEUtility::getUInt64(data, message_offset);
          std::cout << datatype << ": " << uint64_value << std::endl;
        }
        else if (datatype == "DOUBLE")
        {
          double double_value = RTDEUtility::getDouble(data, message_offset);
          std::cout << datatype << ": " << double_value << std::endl;
        }
        else
        {
          // TODO: Handle IN_USE and NOT_FOUND case
          std::cerr << "Unknown datatype: " << datatype << std::endl;
        }
      }

      break;
    }*/

    default:
      std::cout << "Unknown Command: " << static_cast<int>(msg_cmd) << std::endl;
      break;
  }

}

void RTDE::receiveData(RobotState &robot_state)
{
  DEBUG("Receiving...");
  // Read Header
  std::vector<char> data(HEADER_SIZE);
  size_t reply_length = boost::asio::read(*socket_, boost::asio::buffer(data));
  DEBUG("Reply length is: " << reply_length);
  uint32_t message_offset = 0;
  uint16_t msg_size = RTDEUtility::getUInt16(data, message_offset);
  uint8_t msg_cmd = data.at(2);

  DEBUG("ControlHeader: ");
  DEBUG("size is: " << msg_size);
  DEBUG("command is: " << static_cast<int>(msg_cmd));

  // Read Body
  data.resize(msg_size-HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));

  switch(msg_cmd)
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
      std::uint32_t message_offset = 0;
      unsigned char id = RTDEUtility::getUChar(data, message_offset);

      // Read all the variables specified by the user.
      for (const auto &output_name : output_names_)
      {
        if(output_name == "timestamp")
          robot_state.timestamp = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "target_q")
          robot_state.target_q = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "target_qd")
          robot_state.target_qd = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "target_qdd")
          robot_state.target_qdd = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "target_current")
          robot_state.target_current = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "target_moment")
          robot_state.target_moment = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "actual_q")
          robot_state.actual_q = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "actual_qd")
          robot_state.actual_qd = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "actual_current")
          robot_state.actual_current = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "joint_control_output")
          robot_state.joint_control_output = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "actual_TCP_pose")
          robot_state.actual_TCP_pose = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "actual_TCP_speed")
          robot_state.actual_TCP_speed = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "actual_TCP_force")
          robot_state.actual_TCP_force = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "target_TCP_pose")
          robot_state.target_TCP_pose = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "target_TCP_speed")
          robot_state.target_TCP_speed = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "actual_digital_input_bits")
          robot_state.actual_digital_input_bits = RTDEUtility::getUInt64(data, message_offset);
        else if(output_name == "joint_temperatures")
          robot_state.joint_temperatures = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "actual_execution_time")
          robot_state.actual_execution_time = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "robot_mode")
          robot_state.robot_mode = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "joint_mode")
          robot_state.joint_mode = RTDEUtility::unpackVector6Int32(data, message_offset);
        else if(output_name == "safety_mode")
          robot_state.safety_mode = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "actual_tool_accelerometer")
          robot_state.actual_tool_accelerometer = RTDEUtility::unpackVector3d(data, message_offset);
        else if(output_name == "speed_scaling")
          robot_state.speed_scaling = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "target_speed_fraction")
          robot_state.target_speed_fraction = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "actual_momentum")
          robot_state.actual_momentum = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "actual_main_voltage")
          robot_state.actual_main_voltage = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "actual_robot_voltage")
          robot_state.actual_robot_voltage = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "actual_robot_current")
          robot_state.actual_robot_current = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "actual_joint_voltage")
          robot_state.actual_joint_voltage = RTDEUtility::unpackVector6d(data, message_offset);
        else if(output_name == "actual_digital_output_bits")
          robot_state.actual_digital_output_bits = RTDEUtility::getUInt64(data, message_offset);
        else if(output_name == "runtime_state")
          robot_state.runtime_state = RTDEUtility::getUInt32(data, message_offset);
        else if(output_name == "output_int_register_0")
          robot_state.output_int_register_0 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_1")
          robot_state.output_int_register_1 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_2")
          robot_state.output_int_register_2 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_3")
          robot_state.output_int_register_3 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_4")
          robot_state.output_int_register_4 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_5")
          robot_state.output_int_register_5 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_6")
          robot_state.output_int_register_6 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_7")
          robot_state.output_int_register_7 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_8")
          robot_state.output_int_register_8 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_9")
          robot_state.output_int_register_9 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_10")
          robot_state.output_int_register_10 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_11")
          robot_state.output_int_register_11 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_12")
          robot_state.output_int_register_12 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_13")
          robot_state.output_int_register_13 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_14")
          robot_state.output_int_register_14 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_15")
          robot_state.output_int_register_15 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_16")
          robot_state.output_int_register_16 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_17")
          robot_state.output_int_register_17 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_18")
          robot_state.output_int_register_18 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_19")
          robot_state.output_int_register_19 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_20")
          robot_state.output_int_register_20 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_21")
          robot_state.output_int_register_21 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_22")
          robot_state.output_int_register_22 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_int_register_23")
          robot_state.output_int_register_23 = RTDEUtility::getInt32(data, message_offset);
        else if(output_name == "output_double_register_0")
          robot_state.output_double_register_0 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_1")
          robot_state.output_double_register_1 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_2")
          robot_state.output_double_register_2 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_3")
          robot_state.output_double_register_3 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_4")
          robot_state.output_double_register_4 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_5")
          robot_state.output_double_register_5 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_6")
          robot_state.output_double_register_6 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_7")
          robot_state.output_double_register_7 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_8")
          robot_state.output_double_register_8 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_9")
          robot_state.output_double_register_9 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_10")
          robot_state.output_double_register_10 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_11")
          robot_state.output_double_register_11 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_12")
          robot_state.output_double_register_12 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_13")
          robot_state.output_double_register_13 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_14")
          robot_state.output_double_register_14 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_15")
          robot_state.output_double_register_15 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_16")
          robot_state.output_double_register_16 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_17")
          robot_state.output_double_register_17 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_18")
          robot_state.output_double_register_18 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_19")
          robot_state.output_double_register_19 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_20")
          robot_state.output_double_register_20 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_21")
          robot_state.output_double_register_21 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_22")
          robot_state.output_double_register_22 = RTDEUtility::getDouble(data, message_offset);
        else if(output_name == "output_double_register_23")
          robot_state.output_double_register_23 = RTDEUtility::getDouble(data, message_offset);
        else
          DEBUG("Unknown variable name: " << output_name << " please verify the output setup!");
      }

      // Read all the datatypes reported by the controller.
      /*for (const auto &datatype : output_types_)
      {
        if(datatype == "VECTOR6D")
        {
          std::vector<double> vector_6d = RTDEUtility::unpackVector6d(data, message_offset);
          std::cout << datatype << ": ";
          for(const auto &d : vector_6d)
            std::cout << d << " ";
          std::cout << std::endl;
        }
        else if(datatype == "VECTOR6INT32")
        {
          std::vector<int32_t> vector_6_int32 = RTDEUtility::unpackVector6Int32(data, message_offset);
          std::cout << datatype << ": ";
          for(const auto &d : vector_6_int32)
            std::cout << d << " ";
          std::cout << std::endl;
        }
        else if (datatype == "VECTOR3D")
        {
          std::vector<double> vector_3d = RTDEUtility::unpackVector3d(data, message_offset);
          std::cout << datatype << ": ";
          for(const auto &d : vector_3d)
            std::cout << d << " ";
          std::cout << std::endl;
        }
        else if (datatype == "UINT32")
        {
          uint32_t uint32_value = RTDEUtility::getUInt32(data, message_offset);
          std::cout << datatype << ": " << uint32_value << std::endl;
        }
        else if (datatype == "INT32")
        {
          int32_t int32_value = RTDEUtility::getInt32(data, message_offset);
          std::cout << datatype << ": " << int32_value << std::endl;
        }
        else if (datatype == "UINT64")
        {
          uint64_t uint64_value = RTDEUtility::getUInt64(data, message_offset);
          std::cout << datatype << ": " << uint64_value << std::endl;
        }
        else if (datatype == "DOUBLE")
        {
          double double_value = RTDEUtility::getDouble(data, message_offset);
          std::cout << datatype << ": " << double_value << std::endl;
        }
        else
        {
          // TODO: Handle IN_USE and NOT_FOUND case
          std::cerr << "Unknown datatype: " << datatype << std::endl;
        }
      }*/

      break;
    }

    default:
      std::cout << "Unknown Command: " << static_cast<int>(msg_cmd) << std::endl;
      break;
  }

}

void RTDE::getControllerVersion()
{
  std::uint8_t cmd = RTDE_GET_URCONTROL_VERSION;
  sendAll(cmd, "");
  DEBUG("Done sending RTDE_GET_URCONTROL_VERSION");
  receive();
}