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

RTDE::RTDE(std::string hostname, int port)
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

bool RTDE::sendOutputSetup(std::string output_names, double frequency)
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS;

  std::string freq_as_hexstr = RTDEUtility::double2hexstr(frequency);
  std::vector<char> freq_packed = RTDEUtility::hexToBytes(freq_as_hexstr);
  std::copy(output_names.begin(), output_names.end(), std::back_inserter(freq_packed));
  std::string payload(std::begin(freq_packed), std::end(freq_packed));
  sendAll(cmd, payload);
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS");
  receive();
}

void RTDE::sendAll(std::uint8_t command, std::string payload)
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

    case RTDE_DATA_PACKAGE:
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
      }

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