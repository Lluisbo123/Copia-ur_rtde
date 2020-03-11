#pragma once
#ifndef RTDE_RECEIVE_INTERFACE_H
#define RTDE_RECEIVE_INTERFACE_H

#include <ur_rtde/rtde_export.h>
#include <memory>
#include <atomic>
#include <string>
#include <chrono>
#include <vector>

#define MAJOR_VERSION 0
#define CB3_MAJOR_VERSION 3

// forward declarations
namespace boost { class thread; }
namespace ur_rtde { class DashboardClient; }
namespace ur_rtde { class RobotState; }
namespace ur_rtde { class RTDE; }

namespace ur_rtde
{
class RTDEReceiveInterface
{
 public:
  RTDE_EXPORT explicit RTDEReceiveInterface(std::string hostname, std::vector<std::string> variables = {},
                                            int port = 30004);

  RTDE_EXPORT virtual ~RTDEReceiveInterface();

  /**
    * @returns Can be used to reconnect to the robot after a lost connection.
    */
  RTDE_EXPORT bool reconnect();

  /**
    * @returns Connection status for RTDE, useful for checking for lost connection.
    */
  RTDE_EXPORT bool isConnected();

  /**
    * @returns Time elapsed since the controller was started [s]
    */
  RTDE_EXPORT double getTimestamp();

  /**
    * @returns Target joint positions
    */
  RTDE_EXPORT std::vector<double> getTargetQ();

  /**
    * @returns Target joint velocities
    */
  RTDE_EXPORT std::vector<double> getTargetQd();

  /**
    * @returns Target joint accelerations
    */
  RTDE_EXPORT std::vector<double> getTargetQdd();

  /**
    * @returns Target joint currents
    */
  RTDE_EXPORT std::vector<double> getTargetCurrent();

  /**
    * @returns Target joint moments (torques)
    */
  RTDE_EXPORT std::vector<double> getTargetMoment();

  /**
    * @returns Actual joint positions
    */
  RTDE_EXPORT std::vector<double> getActualQ();

  /**
    * @returns Actual joint velocities
    */
  RTDE_EXPORT std::vector<double> getActualQd();

  /**
    * @returns Actual joint currents
    */
  RTDE_EXPORT std::vector<double> getActualCurrent();

  /**
    * @returns Joint control currents
    */
  RTDE_EXPORT std::vector<double> getJointControlOutput();

  /**
    * @returns Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector
    * representation of the tool orientation
    */
  RTDE_EXPORT std::vector<double> getActualTCPPose();

  /**
    * @returns Actual speed of the tool given in Cartesian coordinates
    */
  RTDE_EXPORT std::vector<double> getActualTCPSpeed();

  /**
    * @returns Generalized forces in the TCP
    */
  RTDE_EXPORT std::vector<double> getActualTCPForce();

  /**
    * @returns Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector
    * representation of the tool orientation
    */
  RTDE_EXPORT std::vector<double> getTargetTCPPose();

  /**
    * @returns Target speed of the tool given in Cartesian coordinates
    */
  RTDE_EXPORT std::vector<double> getTargetTCPSpeed();

  /**
    * @returns Current state of the digital inputs. 0-7: Standard, 8-15: Configurable, 16-17: Tool
    */
  RTDE_EXPORT uint64_t getActualDigitalInputBits();

  /**
    * @returns Temperature of each joint in degrees Celsius
    */
  RTDE_EXPORT std::vector<double> getJointTemperatures();

  /**
    * @returns Controller real-time thread execution time
    */
  RTDE_EXPORT double getActualExecutionTime();

  /**
    * @returns Robot mode
    * -1 = ROBOT_MODE_NO_CONTROLLER
    * 0 = ROBOT_MODE_DISCONNECTED
    * 1 = ROBOT_MODE_CONFIRM_SAFETY
    * 2	= ROBOT_MODE_BOOTING
    * 3 = ROBOT_MODE_POWER_OFF
    * 4 = ROBOT_MODE_POWER_ON
    * 5	= ROBOT_MODE_IDLE
    * 6	= ROBOT_MODE_BACKDRIVE
    * 7	= ROBOT_MODE_RUNNING
    * 8	= ROBOT_MODE_UPDATING_FIRMWARE
    */
  RTDE_EXPORT int32_t getRobotMode();

  /**
    * @returns Robot status
    * Bits 0-3: Is power on | Is program running | Is teach button pressed | Is power button pressed
    */
  RTDE_EXPORT uint32_t getRobotStatus();

  /**
    * @returns Joint control modes
    */
  RTDE_EXPORT std::vector<int32_t> getJointMode();

  /**
    * @returns Safety mode
    */
  RTDE_EXPORT int32_t getSafetyMode();

  /**
    * @returns Tool x, y and z accelerometer values
    */
  RTDE_EXPORT std::vector<double> getActualToolAccelerometer();

  /**
    * @returns Speed scaling of the trajectory limiter
    */
  RTDE_EXPORT double getSpeedScaling();

  /**
    * @returns Target speed fraction
    */
  RTDE_EXPORT double getTargetSpeedFraction();

  /**
    * @returns Norm of Cartesian linear momentum
    */
  RTDE_EXPORT double getActualMomentum();

  /**
    * @returns Safety Control Board: Main voltage
    */
  RTDE_EXPORT double getActualMainVoltage();

  /**
    * @returns Safety Control Board: Robot voltage (48V)
    */
  RTDE_EXPORT double getActualRobotVoltage();

  /**
    * @returns Safety Control Board: Robot current
    */
  RTDE_EXPORT double getActualRobotCurrent();

  /**
    * @returns Actual joint voltages
    */
  RTDE_EXPORT std::vector<double> getActualJointVoltage();

  /**
    * @returns Current state of the digital outputs. 0-7: Standard, 8-15: Configurable, 16-17: Tool
    */
  RTDE_EXPORT uint64_t getActualDigitalOutputBits();

  /**
    * @returns Program state
    */
  RTDE_EXPORT uint32_t getRuntimeState();

  /**
    * @returns Standard analog input 0 [A or V]
    */
  RTDE_EXPORT double getStandardAnalogInput0();

  /**
    * @returns Standard analog input 1 [A or V]
    */
  RTDE_EXPORT double getStandardAnalogInput1();

  /**
    * @returns Standard analog output 0 [A or V]
    */
  RTDE_EXPORT double getStandardAnalogOutput0();

  /**
    * @returns Standard analog output 1 [A or V]
    */
  RTDE_EXPORT double getStandardAnalogOutput1();

  RTDE_EXPORT void receiveCallback();

 private:
  bool setupRecipes(const double &frequency);

 private:
  std::vector<std::string> variables_;
  std::string hostname_;
  int port_;
  std::shared_ptr<RTDE> rtde_;
  std::atomic<bool> stop_thread{false};
  std::shared_ptr<boost::thread> th_;
  std::shared_ptr<RobotState> robot_state_;
};

}  // namespace ur_rtde

#endif  // RTDE_RECEIVE_INTERFACE_H
