#ifndef RTDE_RECEIVE_INTERFACE_H
#define RTDE_RECEIVE_INTERFACE_H

#include <rtde.h>
#include <dashboard_client.h>
#include <script_client.h>
#include <thread>
#include <future>
#include <chrono>
#include <sstream>
#include <iostream>

#define MAJOR_VERSION 0
#define CB3_MAJOR_VERSION 3

class RTDEReceiveInterface
{
 public:

  explicit RTDEReceiveInterface(std::vector<std::string> variables, std::string hostname, int port = 30004);

  virtual ~RTDEReceiveInterface();

  /**
    * @returns Time elapsed since the controller was started [s]
    */
  double getTimestamp();

  /**
    * @returns Target joint positions
    */
  std::vector<double> getTargetQ();

  /**
    * @returns Target joint velocities
    */
  std::vector<double> getTargetQd();

  /**
    * @returns Target joint accelerations
    */
  std::vector<double> getTargetQdd();

  /**
    * @returns Target joint currents
    */
  std::vector<double> getTargetCurrent();

  /**
    * @returns Target joint moments (torques)
    */
  std::vector<double> getTargetMoment();

  /**
    * @returns Actual joint positions
    */
  std::vector<double> getActualQ();

  std::vector<double> getActualQd();

  std::vector<double> getActualCurrent();

  std::vector<double> getJointControlOutput();

  std::vector<double> getActualTCPPose();

  std::vector<double> getActualTCPSpeed();

  std::vector<double> getActualTCPForce();

  std::vector<double> getTargetTCPPose();

  std::vector<double> getTargetTCPSpeed();

  uint64_t getActualDigitalInputBits();

  std::vector<double> getJointTemperatures();

  double getActualExecutionTime();

  int32_t getRobotMode();

  std::vector<int32_t> getJointMode();

  int32_t getSafetyMode();

  std::vector<double> getActualToolAccelerometer();

  double getSpeedScaling();

  double getTargetSpeedFraction();

  double getActualMomentum();

  double getActualMainVoltage();

  double getActualRobotVoltage();

  double getActualRobotCurrent();

  std::vector<double> getActualJointVoltage();

  uint64_t getActualDigitalOutputBits();

  uint32_t getRuntimeState();

  void receiveCallback();

 private:
  std::vector<std::string> variables_;
  std::string hostname_;
  int port_;
  std::shared_ptr<RTDE> rtde_;
  std::shared_ptr<RobotState> robot_state_;
};

#endif  // RTDE_RECEIVE_INTERFACE_H
