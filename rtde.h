#ifndef RTDE_LIBRARY_H
#define RTDE_LIBRARY_H

#include <cstdint>
#include <string>
#include <utility>
#include <boost/asio.hpp>

class RTDE
{
 public:
  explicit RTDE(std::string hostname, int port = 30004);

  virtual ~RTDE();

  enum Command
  {
    RTDE_REQUEST_PROTOCOL_VERSION = 86,       // ascii V
    RTDE_GET_URCONTROL_VERSION = 118,         // ascii v
    RTDE_TEXT_MESSAGE = 77,                   // ascii M
    RTDE_DATA_PACKAGE = 85,                   // ascii U
    RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79,  // ascii O
    RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73,   // ascii I
    RTDE_CONTROL_PACKAGE_START = 83,          // ascii S
    RTDE_CONTROL_PACKAGE_PAUSE = 80           // ascii P
  };

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
    STARTED = 2,
    PAUSED = 3
  };

 public:
  void connect();
  void disconnect();
  bool isConnected();

  bool negotiateProtocolVersion();
  void getControllerVersion();
  void receive();
  void sendAll(std::uint8_t command, std::string payload="");
  void sendStart();
  void sendPause();
  bool sendOutputSetup(std::string output_names, double frequency);

 private:
  std::string hostname_;
  int port_;
  ConnectionState conn_state_;
  std::vector<std::string> output_types_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
};

#endif