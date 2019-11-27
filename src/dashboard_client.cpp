#include <ur_rtde/dashboard_client.h>
#include <boost/asio.hpp>
#include <cstring>
#include <iostream>
#include <regex>

using boost::asio::ip::tcp;

namespace ur_rtde
{
DashboardClient::DashboardClient(std::string hostname, int port)
    : hostname_(std::move(hostname)), port_(port), conn_state_(ConnectionState::DISCONNECTED)
{
}

DashboardClient::~DashboardClient() = default;

void DashboardClient::connect()
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
  receive();
  std::cout << "Connected successfully to UR dashboard server: " << hostname_ << " at " << port_ << std::endl;
}

bool DashboardClient::isConnected()
{
  return conn_state_ == ConnectionState::CONNECTED;
}

void DashboardClient::disconnect()
{
  // Close socket
  socket_->close();
  conn_state_ = ConnectionState::DISCONNECTED;
  std::cout << "Dashboard Client - Socket disconnected" << std::endl;
}

void DashboardClient::send(const std::string &str)
{
  boost::asio::write(*socket_, boost::asio::buffer(str));
}

void DashboardClient::loadURP(const std::string &urp_name)
{
  std::string load_urp = "load " + urp_name + "\n";
  send(load_urp);
  receive();
}

void DashboardClient::play()
{
  std::string play = "play\n";
  send(play);
  receive();
}

void DashboardClient::stop()
{
  std::string stop = "stop\n";
  send(stop);
  receive();
}

void DashboardClient::pause()
{
  std::string pause = "pause\n";
  send(pause);
  receive();
}

void DashboardClient::quit()
{
  std::string quit = "quit\n";
  send(quit);
  receive();
}
void DashboardClient::shutdown()
{
  std::string shutdown = "shutdown\n";
  send(shutdown);
  receive();
}

bool DashboardClient::running()
{
  std::string message = "running\n";
  send(message);
  auto str = receive();
  std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::tolower(c); });
  if (strstr(str.c_str(), "true") != nullptr)
    return true;
  return false;
}

void DashboardClient::popup(const std::string &message)
{
  std::string popup = "popup " + message + "\n";
  send(popup);
  receive();
}

void DashboardClient::closePopup()
{
  std::string close_popup = "close popup\n";
  send(close_popup);
  receive();
}

std::string DashboardClient::polyscopeVersion()
{
  std::string close_popup = "PolyscopeVersion\n";
  send(close_popup);
  auto str = receive();
  const std::regex base_regex("\\d+.\\d+.\\d+.\\d+");
  std::smatch base_match;
  std::regex_search(str, base_match, base_regex);
  if (!base_match.empty())
    return std::string(base_match[0]);
  else
    return str;
}

std::string DashboardClient::programState()
{
  std::string close_popup = "programState\n";
  send(close_popup);
  auto state_str = receive();
  return state_str;
}

void DashboardClient::powerOn()
{
  std::string power_on = "power on\n";
  send(power_on);
  receive();
}

void DashboardClient::powerOff()
{
  std::string power_off = "power off\n";
  send(power_off);
  receive();
}

void DashboardClient::brakeRelease()
{
  std::string brake_release = "brake release\n";
  send(brake_release);
  receive();
}

void DashboardClient::unlockProtectiveStop()
{
  std::string unlock_p_stop = "unlock protective stop\n";
  send(unlock_p_stop);
  receive();
}

std::string DashboardClient::receive()
{
  boost::array<char, 1024> recv_buffer_;
  boost::system::error_code error_;
  size_t buflen = socket_->read_some(boost::asio::buffer(recv_buffer_), error_);
  return std::string(recv_buffer_.elems, buflen - 1);  // -1 is removing newline
}

std::string DashboardClient::robotmode()
{
  std::string robotmode = "robotmode\n";
  send(robotmode);
  auto state_str = receive();
  return state_str;
}

std::string DashboardClient::getLoadedProgram()
{
  std::string get_loaded_program = "get loaded program\n";
  send(get_loaded_program);
  auto state_str = receive();
  return state_str;
}

void DashboardClient::addToLog(const std::string &message)
{
  std::string add_to_lof = "addToLog " + message + "\n";
  send(add_to_lof);
  receive();
}

bool DashboardClient::isProgramSaved()
{
  std::string is_program_saved = "isProgramSaved\n";
  send(is_program_saved);
  auto str = receive();
  if (strstr(str.c_str(), "True") != nullptr)
    return true;
  return false;
}

void DashboardClient::setUserRole(const UserRole &role)
{
  std::string message;
  switch (role)
  {
    case UserRole::LOCKED:
      message = "locked";
    case UserRole::PROGRAMMER:
      message = "programmer";
    case UserRole::OPERATOR:
      message = "operator";
    case UserRole::NONE:
      message = "none";
    case UserRole::RESTRICTED:
      message = "restricted";
  }
  send("setUserRole " + message + "\n");
  receive();
}

std::string DashboardClient::safetymode()
{
  std::string safetymode = "safetymode\n";
  send(safetymode);
  return receive();
}

std::string DashboardClient::safetystatus()
{
  std::string safetystatus = "safetystatus\n";
  send(safetystatus);
  return receive();
}

void DashboardClient::closeSafetyPopup()
{
  std::string str = "close safety popup\n";
  send(str);
  receive();
}

void DashboardClient::restartSafety()
{
  std::string str = "restart safety\n";
  send(str);
  receive();
}

}  // namespace ur_rtde
