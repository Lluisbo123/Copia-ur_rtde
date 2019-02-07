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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <boost/thread.hpp>

#define MAJOR_VERSION 0
#define CB3_MAJOR_VERSION 3

class RTDEReceiveInterface
{
 public:

  explicit RTDEReceiveInterface(std::string hostname, std::vector<std::string> variables = {}, int port = 30004);

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
  std::atomic<bool> stop_thread {false};
  std::shared_ptr<boost::thread> th_;
  std::shared_ptr<RobotState> robot_state_;
};

namespace py = pybind11;


PYBIND11_MODULE(rtde_receive, m)
{
  m.doc() = "RTDE Receive Interface";
  py::class_<RTDEReceiveInterface>(m, "RTDEReceiveInterface")
      .def(py::init<std::string>())
      .def("getTimestamp", &RTDEReceiveInterface::getTimestamp)
      .def("getTargetQ", &RTDEReceiveInterface::getTargetQ)
      .def("getTargetQd", &RTDEReceiveInterface::getTargetQd)
      .def("getTargetQdd", &RTDEReceiveInterface::getTargetQdd)
      .def("getTargetCurrent", &RTDEReceiveInterface::getTargetCurrent)
      .def("getTargetMoment", &RTDEReceiveInterface::getTargetMoment)
      .def("getActualQ", &RTDEReceiveInterface::getActualQ)
      .def("getActualQd", &RTDEReceiveInterface::getActualQd)
      .def("getActualCurrent", &RTDEReceiveInterface::getActualCurrent)
      .def("getJointControlOutput", &RTDEReceiveInterface::getJointControlOutput)
      .def("getActualTCPPose", &RTDEReceiveInterface::getActualTCPPose)
      .def("getActualTCPSpeed", &RTDEReceiveInterface::getActualTCPSpeed)
      .def("getActualTCPForce", &RTDEReceiveInterface::getActualTCPForce)
      .def("getTargetTCPPose", &RTDEReceiveInterface::getTargetTCPPose)
      .def("getTargetTCPSpeed", &RTDEReceiveInterface::getTargetTCPSpeed)
      .def("getActualDigitalInputBits", &RTDEReceiveInterface::getActualDigitalInputBits)
      .def("getJointTemperatures", &RTDEReceiveInterface::getJointTemperatures)
      .def("getActualExecutionTime", &RTDEReceiveInterface::getActualExecutionTime)
      .def("getRobotModes", &RTDEReceiveInterface::getRobotMode)
      .def("getJointMode", &RTDEReceiveInterface::getJointMode)
      .def("getSafetyMode", &RTDEReceiveInterface::getSafetyMode)
      .def("getActualToolAccelerometer", &RTDEReceiveInterface::getActualToolAccelerometer)
      .def("getSpeedScaling", &RTDEReceiveInterface::getSpeedScaling)
      .def("getTargetSpeedFraction", &RTDEReceiveInterface::getTargetSpeedFraction)
      .def("getActualMomentum", &RTDEReceiveInterface::getActualMomentum)
      .def("getActualMainVoltage", &RTDEReceiveInterface::getActualMainVoltage)
      .def("getActualRobotVoltage", &RTDEReceiveInterface::getActualRobotVoltage)
      .def("getActualRobotCurrent", &RTDEReceiveInterface::getActualRobotCurrent)
      .def("getActualJointVoltage", &RTDEReceiveInterface::getActualJointVoltage)
      .def("getActualDigitalOutputBits", &RTDEReceiveInterface::getActualDigitalOutputBits)
      .def("getRuntimeState", &RTDEReceiveInterface::getRuntimeState)
      .def("__repr__", [](const RTDEReceiveInterface& a)
      {
        return "<rtde_py.RTDEReceiveInterface>";
      });
}

#endif  // RTDE_RECEIVE_INTERFACE_H
