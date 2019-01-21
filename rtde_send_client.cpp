#include "rtde.h"
#include "dashboard_client.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <numeric>

using namespace std::chrono;

int main(int argc, char *argv[])
{
  double frequency = 500;
  RTDE rtde("127.0.0.1");
  rtde.connect();
  rtde.negotiateProtocolVersion();
  rtde.getControllerVersion();

  // Create a connection to the dashboard server
  DashboardClient db_client("127.0.0.1");
  db_client.connect();

  // Data to be sent
  std::vector<double> tcp_pose1 = {-0.12, -0.43, 0.14, 0, 3.11, 0.04};
  std::vector<double> tcp_pose2 = {-0.12, -0.51, 0.21, 0, 3.11, 0.04};

  // Setup output
  std::vector<std::string> state_names = {"target_q", "target_qd", "output_int_register_0"};
  rtde.sendOutputSetup(state_names, frequency);

  // Setup input
  std::vector<std::string> setp_names = {"input_double_register_0", "input_double_register_1",
                                         "input_double_register_2", "input_double_register_3",
                                         "input_double_register_4", "input_double_register_5"};
  rtde.sendInputSetup(setp_names);

  // std::string watchdog_names = "input_int_register_0";
  // rtde.sendInputSetup(watchdog_names);

  rtde.sendStart();

  // Clear registers with tcp pose of zeroes
  std::vector<double> tcp_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rtde.send(tcp_pose);

  // Execute script on UR Controller through the dashboard server
  db_client.play();

  RobotState robot_state;

  bool keep_running = true;
  while (keep_running)
  {
    // Receive data
    rtde.receiveData(robot_state);

    if (robot_state.output_int_register_0 != 0)
    {
      if (tcp_pose != tcp_pose1)
        tcp_pose = tcp_pose1;
      else if (tcp_pose != tcp_pose2)
        tcp_pose = tcp_pose2;
      // Send data
      rtde.send(tcp_pose);
    }
  }

  rtde.sendPause();
  rtde.disconnect();
  db_client.disconnect();
  return 0;
}