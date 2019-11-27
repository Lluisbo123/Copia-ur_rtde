#ifndef RTDE_DASHBOARD_CLIENT_H
#define RTDE_DASHBOARD_CLIENT_H

#include <ur_rtde/rtde_export.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <string>
#include <ur_rtde/dashboard_enums.h>

namespace ur_rtde
{
class DashboardClient
{
 public:

  RTDE_EXPORT explicit DashboardClient(std::string hostname, int port = 29999);

  RTDE_EXPORT virtual ~DashboardClient();

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
  };

 public:
  RTDE_EXPORT void connect();
  RTDE_EXPORT bool isConnected();
  RTDE_EXPORT void disconnect();
  RTDE_EXPORT void send(const std::string &str);
  RTDE_EXPORT std::string receive();
  RTDE_EXPORT void loadURP(const std::string &urp_name);
  RTDE_EXPORT void play();
  RTDE_EXPORT void stop();
  RTDE_EXPORT void pause();
  RTDE_EXPORT void quit();
  RTDE_EXPORT void shutdown();
  RTDE_EXPORT bool running();
  RTDE_EXPORT void popup(const std::string &message);
  RTDE_EXPORT void closePopup();
  RTDE_EXPORT void closeSafetyPopup();
  RTDE_EXPORT void powerOn();
  RTDE_EXPORT void powerOff();
  RTDE_EXPORT void brakeRelease();
  RTDE_EXPORT void unlockProtectiveStop();
  RTDE_EXPORT void restartSafety();
  RTDE_EXPORT std::string polyscopeVersion();
  RTDE_EXPORT std::string programState();
  RTDE_EXPORT std::string robotmode();
  RTDE_EXPORT std::string getLoadedProgram();
  RTDE_EXPORT std::string safetymode();
  RTDE_EXPORT std::string safetystatus();
  RTDE_EXPORT void addToLog(const std::string &message);
  RTDE_EXPORT bool isProgramSaved();
  RTDE_EXPORT void setUserRole(const UserRole &role);

 private:
  std::string hostname_;
  int port_;
  ConnectionState conn_state_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
};

}  // namespace ur_rtde

#endif  // RTDE_DASHBOARD_CLIENT_H
