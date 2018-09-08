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

RTDE::RTDE(std::string hostname, int port)
    : hostname_(std::move(hostname)), port_(port), conn_state_(ConnectionState::DISCONNECTED)
{
}

RTDE::~RTDE() = default;

void RTDE::connect()
{
  io_service_ = std::make_shared<boost::asio::io_service>();
  socket_ = std::make_shared<tcp::socket>(*io_service_);
  resolver_ = std::make_shared<tcp::resolver>(*io_service_);
  tcp::resolver::query query(hostname_, std::to_string(port_));
  boost::asio::connect(*socket_, resolver_->resolve(query));
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

template <int N, typename T, typename U>
static auto constexpr get(std::pair<T, U> const& pair)
-> typename std::tuple_element<N, decltype(pair)>::type
{
  return N == 0 ? pair.first : pair.second;
}


bool RTDE::sendOutputSetup(std::vector<std::string> output_names, std::vector<std::string> output_types, int frequency)
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS;
  auto payload = pystruct::pack(PY_STRING(">d"), frequency);
  std::vector<char> pay_vec(payload.begin(), payload.end());
  std::string comma = ",target_q";
  for(unsigned int i=0; i<comma.size(); i++)
  {
    char c = comma.c_str()[i];
    pay_vec.push_back(c);
  }
  std::string str(std::begin(pay_vec), std::end(pay_vec));
  std::cout << str << std::endl;
  sendAll(cmd, str);
  std::cout << "Done sending" << std::endl;
  receive();
}

void RTDE::sendAll(std::uint8_t command, std::string payload)
{
  auto cmd_packed = pystruct::pack(PY_STRING(">HB"), pystruct::calcsize(PY_STRING(">HB")), command);
  std::vector<char> cmd_packed_vec(cmd_packed.begin(), cmd_packed.end());
  if (!payload.empty())
  {
    for(unsigned int i=0; i<payload.size(); i++)
    {
      char c = payload.c_str()[i];
      cmd_packed_vec.push_back(c);
    }
  }

  std::cout << cmd_packed_vec.data() << std::endl;

  boost::asio::write(*socket_, boost::asio::buffer(cmd_packed, sizeof(cmd_packed)));
}

/*void RTDE::sendAll(std::uint8_t command)
{
  auto runtimePacked = pystruct::pack(PY_STRING(">HB"), pystruct::calcsize(PY_STRING(">HB")), command);

  boost::asio::write(*socket_, boost::asio::buffer(runtimePacked, sizeof(runtimePacked)));
}*/

void RTDE::receive()
{
  // Read Header
  std::vector<char> data(HEADER_SIZE);
  size_t reply_length = boost::asio::read(*socket_, boost::asio::buffer(data));
  std::cout << "Reply length is: " << reply_length << std::endl;
  auto [msg_size, msg_cmd] = pystruct::unpack(PY_STRING(">HB"), data);
  std::cout << "ControlHeader: " << std::endl;
  std::cout << "size is: " << msg_size << std::endl;
  std::cout << "command is: " << static_cast<int>(msg_cmd) << std::endl;

  // Read Body
  data.resize(msg_size-HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));

  auto cmd = static_cast<uint8_t>(msg_cmd);

  switch(cmd)
  {
    case RTDE_GET_URCONTROL_VERSION:
    {
      std::cout << "ControlVersion: " << std::endl;
      auto [v_major, v_minor, v_bugfix, v_build] = pystruct::unpack(PY_STRING(">IIII"), data);
      std::cout << v_major << "." << v_minor << "." << v_bugfix << "." << v_build << "\n";
      break;
    }

    case RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
    {
      //auto [id] = pystruct::unpack(PY_STRING(">B"), data);
      //std::cout << "id: " << typeid(id).name() << std::endl;
      auto [id, q1, q2, q3, q4, q5, q6] = pystruct::unpack(PY_STRING(">Bdddddd"), data);
      std::cout << q1 << " " << q2 << " " << q3 << " " << q4 << std::endl;
      break;
    }

    default:
      break;
  }


}

void RTDE::getControllerVersion()
{
  std::uint8_t cmd = RTDE_GET_URCONTROL_VERSION;
  sendAll(cmd);
  std::cout << "Done sending" << std::endl;
  receive();
}