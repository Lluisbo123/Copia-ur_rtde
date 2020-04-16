#include <ur_rtde/rtde_control_interface.h>
#include <thread>
#include <chrono>
#include <iostream>

using namespace ur_rtde;
using namespace std::chrono; 

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("127.0.0.1");

  // Data to be sent
  double velocity = 0.5;
  double acceleration = 0.5;
  //int movec_mode = 0;
  std::vector<double> tcp_pose1 = {-0.143, -0.435, 0.20, -0.001, 3.12, 0.04};
  std::vector<double> tcp_pose2 = {-0.143, -0.51, 0.21, -0.001, 3.12, 0.04};

  std::vector<double> circ_pose_via = {-0.143, -0.435, 0.20, -0.001, 3.12, 0.04};
  std::vector<double> circ_pose_to = {-0.343, -0.435, 0.20, -0.001, 3.12, 0.04};

  std::vector<double> joint_q1 = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};
  std::vector<double> joint_q2 = {-0.69, -2.37, -1.79, -0.37, 1.93, 0.87};
  std::vector<double> joint_q3 = {-1.30, -1.36, -1.90, -1.05, 1.50, 0.03};

  std::vector<double> path_pose1 = {-0.143, -0.435, 0.20, -0.001, 3.12, 0.04, velocity, acceleration, 0};
  std::vector<double> path_pose2 = {-0.143, -0.51, 0.21, -0.001, 3.12, 0.04, velocity, acceleration, 0.02};
  std::vector<double> path_pose3 = {-0.32, -0.61, 0.31, -0.001, 3.12, 0.04, velocity, acceleration, 0};

  std::vector<std::vector<double>> path;
  path.push_back(path_pose1);
  path.push_back(path_pose2);
  path.push_back(path_pose3);

  // Send a linear path with blending in between - (currently uses separate script)
  // rtde_control.moveL(path);

  // Send a linear movement
  /*rtde_control.moveL(tcp_pose1, velocity, acceleration);

  // Send joint movements
  rtde_control.moveJ(joint_q1, velocity, acceleration);
  rtde_control.moveJ(joint_q2, velocity, acceleration);
  rtde_control.moveJ(joint_q1, velocity, acceleration);

  // Use the inverse kinematics of the robot to perform joint movements from a pose
  rtde_control.moveJ_IK(tcp_pose1, velocity, acceleration);
  rtde_control.moveJ_IK(tcp_pose2, velocity, acceleration);
  rtde_control.moveJ_IK(tcp_pose1, velocity, acceleration);

  // Use forward kinematics of the robot to perform linear movements from a joint configuration
  rtde_control.moveL_FK(joint_q1, velocity, acceleration);
  rtde_control.moveL_FK(joint_q2, velocity, acceleration);
  rtde_control.moveL_FK(joint_q1, velocity, acceleration);

  // Send a linear movement
  rtde_control.moveL(tcp_pose1, velocity, acceleration);

  // Send a circular movement
  //rtde_control.moveC(circ_pose_via, circ_pose_to, velocity, acceleration, movec_mode);*/

  // Test force mode
  std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
  std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
  std::vector<double> wrench_down = {0, 0, -20, 0, 0, 0};
  std::vector<double> wrench_up = {0, 0, 20, 0, 0, 0};
  int force_type = 2;
  std::vector<double> limits = {2, 2, 1.5, 1, 1, 1};

  rtde_control.forceModeStart(task_frame, selection_vector, wrench_down, force_type, limits);
  std::cout << std::endl << "Going Down!" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << std::endl << "Going Up!" << std::endl << std::endl;
  rtde_control.forceModeUpdate(wrench_up);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  rtde_control.forceModeStop();

  /*std::vector<double> joint_speed = {0.2, 0.3, 0.1, 0.05, 0, 0};
  std::vector<double> tool_speed = {0.5, 0.4, 0.0, 1.57, 0, 0};
  double time = 0.5;

  for (unsigned int i=0; i<10; i++)
  {
    rtde_control.speedJ(joint_speed, acceleration);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  rtde_control.speedStop();*/

  // Test servoJ
  /*double time = 0.002;
  double lookahead_time = 0.1;
  double gain = 300;
  // First move to q2
  rtde_control.moveJ(joint_q2);
  // Move back to q1
  rtde_control.moveJ(joint_q1);
  rtde_control.servoJ(joint_q1, velocity, acceleration, time, lookahead_time, gain);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  for (unsigned int i=0; i<1000; i++)
  {
    joint_q1[0] += 0.001;
    joint_q1[1] += 0.001;
    auto start = high_resolution_clock::now(); 
    rtde_control.servoJ(joint_q1, velocity, acceleration, time, lookahead_time, gain);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "Time taken by servoJ: "
         << duration.count() << " microseconds" << std::endl; 
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  
  rtde_control.servoStop();*/

  // Test moveP and moveC
  /*std::vector<double> waypoint_1 = {-0.300, -0.300, 0.100, -2.695, 1.605, -0.036};
  std::vector<double> waypoint_2 = {-0.399, -0.199, 0.099, -2.694, 1.606, -0.037};
  std::vector<double> waypoint_3 = {-0.500, -0.299, 0.099, -2.695, 1.606, -0.038};
  std::vector<double> waypoint_4 = {-0.399, -0.400, 0.100, -2.695, 1.605, -0.038};
  std::vector<double> waypoint_5 = {-0.300, -0.300, 0.100, -2.696, 1.605, -0.036};

  // Move to init pose
  rtde_control.moveL({-0.300, -0.300, 0.100, -2.695, 1.605, -0.036});

  // Perform circular motion
  for (unsigned int i=0; i<5; i++)
  {
    rtde_control.moveP(waypoint_1, 0.25, 1.2, 0.1);
    rtde_control.moveC(waypoint_2, waypoint_3, 0.25, 1.2, 0.1);
    rtde_control.moveC(waypoint_4, waypoint_5, 0.25, 1.2, 0.1);
  }
  rtde_control.stopScript();*/

  /*rtde_control.moveJ(joint_q1, velocity, acceleration);
  rtde_control.moveJ(joint_q2, velocity, acceleration);
  rtde_control.moveJ(joint_q3, velocity, acceleration);
  rtde_control.moveJ(joint_q1, velocity, acceleration);

  for (int step = 0; step < 200; step++)
  {
    for (const auto &d : rtde_control.getActualJointPositionsHistory(step))
      std::cout << d << " ";

    std::cout << std::endl;
  }*/

  // rtde_control.teachMode();
  // rtde_control.endTeachMode();

  // rtde_control.forceModeSetDamping(0.025);
  // rtde_control.forceModeSetGainScaling(0.5);

  // rtde_control.setSpeedSlider(0.3);

  // std::vector<double> cog = {0, 0, 0};
  // rtde_control.setPayload(1.3);
  // rtde_control.setPayload(1.2, cog);

  // rtde_control.setAnalogOutputCurrent(0, 0.5);
  // rtde_control.setAnalogOutputCurrent(1, 0.25);

  // Test servoC
  // rtde_control.servoC(tcp_pose1, velocity, acceleration, 0.01);*/

  // Test standard and tool digital out
  // rtde_control.setStandardDigitalOut(2, true);
  // rtde_control.setToolDigitalOut(1, true);

  rtde_control.stopScript();

  return 0;
}
