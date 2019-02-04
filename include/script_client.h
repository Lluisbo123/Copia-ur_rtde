#ifndef RTDE_SCRIPT_CLIENT_H
#define RTDE_SCRIPT_CLIENT_H

#include <string>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

class ScriptClient
{
 public:
  explicit ScriptClient(std::string hostname, int port = 30002);

  virtual ~ScriptClient();

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
  };

 public:
  void connect();
  void disconnect();
  bool isConnected();
  bool sendScript(const std::string& str);
  bool sendScriptCommand(const std::string &cmd_str);

 private:
  std::string hostname_;
  int port_;
  ConnectionState conn_state_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
};

#endif //RTDE_SCRIPT_CLIENT_H
