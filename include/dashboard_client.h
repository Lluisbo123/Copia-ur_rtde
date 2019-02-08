#ifndef RTDE_DASHBOARD_CLIENT_H
#define RTDE_DASHBOARD_CLIENT_H

#include <rtde_export.h>
#include <string>
#include <boost/asio.hpp>

class RTDE_EXPORT DashboardClient
{
 public:
  explicit DashboardClient(std::string hostname, int port = 29999);

  virtual ~DashboardClient();

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
  };

 public:
  void connect();
  bool isConnected();
  void disconnect();
  void send(const std::string& str);
  void loadURP(const std::string& urp_name);
  void play();
  void stop();
  void pause();
  void quit();
  void shutdown();
  bool running();
  void popup(const std::string& message);
  void closePopup();
  std::string programState();
  void powerOn();
  void powerOff();
  void brakeRelease();
  void unlockProtectiveStop();


 private:
  std::string hostname_;
  int port_;
  ConnectionState conn_state_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
};

#endif  // RTDE_DASHBOARD_CLIENT_H
