#include "rtde.h"
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <tuple>
#include <iostream>
#include <string>
#include <stdexcept>
#include "cppystruct.h"

using boost::asio::ip::tcp;
const unsigned HEADER_SIZE = 3;
#define RTDE_PROTOCOL_VERSION 2
#define DEBUG_OUTPUT false

#if DEBUG_OUTPUT
#define DEBUG(a) {std::cerr << "RTDE:" << __LINE__ << ": " << a << std::endl;}
#else
#define DEBUG(a) {}
#endif

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

template <typename T, std::size_t N1, std::size_t N2>
constexpr std::array<T, N1 + N2> concat(std::array<T, N1> lhs, std::array<T, N2> rhs)
{
  std::array<T, N1 + N2> result{};
  std::size_t index = 0;

  for (auto& el : lhs) {
    result[index] = std::move(el);
    ++index;
  }
  for (auto& el : rhs) {
    result[index] = std::move(el);
    ++index;
  }

  return result;
}

bool RTDE::negotiateProtocolVersion()
{
  std::uint8_t cmd = RTDE_REQUEST_PROTOCOL_VERSION;
  auto cmd_packed = pystruct::pack(PY_STRING(">HB"), pystruct::calcsize(PY_STRING(">HBH")), cmd);
  auto payload_packed = pystruct::pack(PY_STRING(">H"), RTDE_PROTOCOL_VERSION);
  std::array<char, cmd_packed.size()+payload_packed.size()> all = concat(cmd_packed, payload_packed);
  boost::asio::write(*socket_, boost::asio::buffer(all, sizeof(all)));
  DEBUG("Done sending RTDE_REQUEST_PROTOCOL_VERSION");
  receive();
}

bool RTDE::sendOutputSetup(std::vector<std::string> output_names, std::vector<std::string> output_types, int frequency)
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS;
  auto freq_packed = pystruct::pack(PY_STRING(">d"), frequency);
  std::vector<char> var_vec(freq_packed.begin(), freq_packed.end());
  std::string variables = "actual_q";
  std::copy(variables.begin(), variables.end(), std::back_inserter(var_vec));
  std::string var_vec_str(std::begin(var_vec), std::end(var_vec));
  sendAll(cmd, var_vec_str);
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS");
  receive();
}

void RTDE::sendAll(std::uint8_t command, std::string payload)
{
  auto cmd_packed = pystruct::pack(PY_STRING(">HB"), pystruct::calcsize(PY_STRING(">HB")) + payload.length(), command);

  std::vector<char> cmd_packed_vec(cmd_packed.begin(), cmd_packed.end());
  std::copy(payload.begin(), payload.end(), std::back_inserter(cmd_packed_vec));
  boost::asio::write(*socket_, boost::asio::buffer(cmd_packed_vec, sizeof(cmd_packed_vec)));
}

void RTDE::sendStart()
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_START;
  sendAll(cmd, "");
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_START");
  receive();
}

void RTDE::receiveData()
{
  DEBUG("Receiving Data...");
  // Read Header
  std::vector<char> data(4096);
  size_t reply_length = boost::asio::read(*socket_, boost::asio::buffer(data));
  DEBUG("Reply length is: " << reply_length);
  auto [msg_size, msg_cmd] = pystruct::unpack(PY_STRING(">HB"), data);
  DEBUG("ControlHeader: ");
  DEBUG("size is: " << msg_size);
  DEBUG("command is: " << static_cast<int>(msg_cmd));
}

void RTDE::receive()
{
  DEBUG("Receiving...");
  // Read Header
  std::vector<char> data(HEADER_SIZE);
  size_t reply_length = boost::asio::read(*socket_, boost::asio::buffer(data));

  DEBUG("Reply length is: " << reply_length);
  auto [msg_size, msg_cmd] = pystruct::unpack(PY_STRING(">HB"), data);
  DEBUG("ControlHeader: ");
  DEBUG("size is: " << msg_size);
  DEBUG("command is: " << static_cast<int>(msg_cmd));

  // Read Body
  data.resize(msg_size-HEADER_SIZE);

  boost::asio::read(*socket_, boost::asio::buffer(data));

  auto cmd = static_cast<uint8_t>(msg_cmd);

  switch(cmd)
  {
    case RTDE_REQUEST_PROTOCOL_VERSION:
    {

      break;
    }

    case RTDE_TEXT_MESSAGE:
    {
      int offset = 0;
      auto [msg_length] = pystruct::unpack(PY_STRING(">B"), data);
      for (int i = 1; i < msg_length; i++)
      {
        std::cout << data[i];
      }
      break;
    }

    case RTDE_GET_URCONTROL_VERSION:
    {
      DEBUG("ControlVersion: ");
      auto [v_major, v_minor, v_bugfix, v_build] = pystruct::unpack(PY_STRING(">IIII"), data);
      DEBUG(v_major << "." << v_minor << "." << v_bugfix << "." << v_build);
      break;
    }

    case RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
    {
      auto [id] = pystruct::unpack(PY_STRING(">B"), data);

      DEBUG("ID:" << id);
      std::string datatype(std::begin(data), std::end(data));
      DEBUG("Datatype:" << datatype);

      break;
    }

    case RTDE_CONTROL_PACKAGE_START:
    {
      auto [success] = pystruct::unpack(PY_STRING(">B"), data);
      DEBUG("success: " << static_cast<bool>(success));
      bool rtde_success = static_cast<bool>(success);
      if (rtde_success)
      {
        conn_state_ = ConnectionState::STARTED;
        std::cout << "RTDE synchronization started" << std::endl;
      }
      else
        std::cerr << "Unable to start synchronization" << std::endl;
      break;
    }

    case RTDE_DATA_PACKAGE:
    {
      auto [id, q1, q2, q3, q4, q5, q6] = pystruct::unpack(PY_STRING(">Bdddddd"), data);
      DEBUG("ID is: " << id);
      std::cout << q1 << " " << q2 << " " << q3 << " " << q4 << " " << q5 << " " << q6 << std::endl;
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