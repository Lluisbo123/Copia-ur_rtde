#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

namespace py = pybind11;
using namespace ur_rtde;

namespace rtde_control
{
PYBIND11_MODULE(rtde_control, m)
{
  m.doc() = "RTDE Control Interface";
  py::class_<RTDEControlInterface>(m, "RTDEControlInterface")
      .def(py::init<std::string>())
      .def("reconnect", &RTDEControlInterface::reconnect,
           "Can be used to reconnect to the robot after a lost connection.", py::call_guard<py::gil_scoped_release>())
      .def("isConnected", &RTDEControlInterface::isConnected, py::call_guard<py::gil_scoped_release>())
      .def("sendCustomScriptFunction", &RTDEControlInterface::sendCustomScriptFunction,
           py::call_guard<py::gil_scoped_release>())
      .def("sendCustomScriptFile", &RTDEControlInterface::sendCustomScriptFile,
           py::call_guard<py::gil_scoped_release>())
      .def("stopRobot", &RTDEControlInterface::stopRobot, py::call_guard<py::gil_scoped_release>())
      .def("moveJ",
           (bool (RTDEControlInterface::*)(const std::vector<std::vector<double>> &path)) & RTDEControlInterface::moveJ,
           "moveJ with path", py::call_guard<py::gil_scoped_release>())
      .def("moveJ", (bool (RTDEControlInterface::*)(const std::vector<double> &q, double speed, double acceleration)) &
                        RTDEControlInterface::moveJ,
           "moveJ without path", py::arg("q"), py::arg("speed") = 1.05, py::arg("acceleration") = 1.4,
           py::call_guard<py::gil_scoped_release>())
      .def("moveJ_IK", &RTDEControlInterface::moveJ_IK, py::arg("pose"), py::arg("speed") = 1.05,
           py::arg("acceleration") = 1.4, py::call_guard<py::gil_scoped_release>())
      .def("moveL",
           (bool (RTDEControlInterface::*)(const std::vector<std::vector<double>> &path)) & RTDEControlInterface::moveL,
           "moveL with path", py::call_guard<py::gil_scoped_release>())
      .def("moveL",
           (bool (RTDEControlInterface::*)(const std::vector<double> &pose, double speed, double acceleration)) &
               RTDEControlInterface::moveL,
           "moveL without path", py::arg("pose"), py::arg("speed") = 0.25, py::arg("acceleration") = 1.2,
           py::call_guard<py::gil_scoped_release>())
      .def("moveL_FK", &RTDEControlInterface::moveL_FK, py::arg("q"), py::arg("speed") = 0.25,
           py::arg("acceleration") = 1.2, py::call_guard<py::gil_scoped_release>())
      .def("moveC", &RTDEControlInterface::moveC, py::call_guard<py::gil_scoped_release>())
      .def("speedJ", &RTDEControlInterface::speedJ, py::arg("qd"), py::arg("acceleration") = 0.5, py::arg("time") = 0.0,
           py::call_guard<py::gil_scoped_release>())
      .def("speedL", &RTDEControlInterface::speedL, py::arg("xd"), py::arg("acceleration") = 0.25,
           py::arg("time") = 0.0, py::call_guard<py::gil_scoped_release>())
      .def("speedStop", &RTDEControlInterface::speedStop, py::call_guard<py::gil_scoped_release>())
      .def("servoJ", &RTDEControlInterface::servoJ, py::call_guard<py::gil_scoped_release>())
      .def("servoL", &RTDEControlInterface::servoL, py::call_guard<py::gil_scoped_release>())
      .def("servoC", &RTDEControlInterface::servoC, py::arg("pose"), py::arg("speed") = 0.25,
           py::arg("acceleration") = 1.2, py::arg("blend") = 0.0, py::call_guard<py::gil_scoped_release>())
      .def("servoStop", &RTDEControlInterface::servoStop, py::call_guard<py::gil_scoped_release>())
      .def("forceModeStart", &RTDEControlInterface::forceModeStart, py::call_guard<py::gil_scoped_release>())
      .def("forceModeStop", &RTDEControlInterface::forceModeStop, py::call_guard<py::gil_scoped_release>())
      .def("forceModeSetDamping", &RTDEControlInterface::forceModeSetDamping, py::call_guard<py::gil_scoped_release>())
      .def("toolContact", &RTDEControlInterface::toolContact, py::call_guard<py::gil_scoped_release>())
      .def("getTargetWaypoint", &RTDEControlInterface::getTargetWaypoint, py::call_guard<py::gil_scoped_release>())
      .def("getActualJointPositionsHistory", &RTDEControlInterface::getActualJointPositionsHistory,
           py::call_guard<py::gil_scoped_release>())
      .def("getStepTime", &RTDEControlInterface::getStepTime, py::call_guard<py::gil_scoped_release>())
      .def("teachMode", &RTDEControlInterface::teachMode, py::call_guard<py::gil_scoped_release>())
      .def("endTeachMode", &RTDEControlInterface::endTeachMode, py::call_guard<py::gil_scoped_release>())
      .def("forceModeSetGainScaling", &RTDEControlInterface::forceModeSetGainScaling,
           py::call_guard<py::gil_scoped_release>())
      .def("zeroFtSensor", &RTDEControlInterface::zeroFtSensor, py::call_guard<py::gil_scoped_release>())
      .def("setPayload", &RTDEControlInterface::setPayload, py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const RTDEControlInterface &a)
           {
        return "<rtde_control.RTDEControlInterface>";
      });
}
};

namespace rtde_receive
{
PYBIND11_MODULE(rtde_receive, m)
{
  m.doc() = "RTDE Receive Interface";
  py::class_<RTDEReceiveInterface>(m, "RTDEReceiveInterface")
      .def(py::init<std::string>())
      .def("reconnect", &RTDEReceiveInterface::reconnect, py::call_guard<py::gil_scoped_release>())
      .def("isConnected", &RTDEReceiveInterface::isConnected, py::call_guard<py::gil_scoped_release>())
      .def("getTimestamp", &RTDEReceiveInterface::getTimestamp, py::call_guard<py::gil_scoped_release>())
      .def("getTargetQ", &RTDEReceiveInterface::getTargetQ, py::call_guard<py::gil_scoped_release>())
      .def("getTargetQd", &RTDEReceiveInterface::getTargetQd, py::call_guard<py::gil_scoped_release>())
      .def("getTargetQdd", &RTDEReceiveInterface::getTargetQdd, py::call_guard<py::gil_scoped_release>())
      .def("getTargetCurrent", &RTDEReceiveInterface::getTargetCurrent, py::call_guard<py::gil_scoped_release>())
      .def("getTargetMoment", &RTDEReceiveInterface::getTargetMoment, py::call_guard<py::gil_scoped_release>())
      .def("getActualQ", &RTDEReceiveInterface::getActualQ, py::call_guard<py::gil_scoped_release>())
      .def("getActualQd", &RTDEReceiveInterface::getActualQd, py::call_guard<py::gil_scoped_release>())
      .def("getActualCurrent", &RTDEReceiveInterface::getActualCurrent, py::call_guard<py::gil_scoped_release>())
      .def("getJointControlOutput", &RTDEReceiveInterface::getJointControlOutput,
           py::call_guard<py::gil_scoped_release>())
      .def("getActualTCPPose", &RTDEReceiveInterface::getActualTCPPose, py::call_guard<py::gil_scoped_release>())
      .def("getActualTCPSpeed", &RTDEReceiveInterface::getActualTCPSpeed, py::call_guard<py::gil_scoped_release>())
      .def("getActualTCPForce", &RTDEReceiveInterface::getActualTCPForce, py::call_guard<py::gil_scoped_release>())
      .def("getTargetTCPPose", &RTDEReceiveInterface::getTargetTCPPose, py::call_guard<py::gil_scoped_release>())
      .def("getTargetTCPSpeed", &RTDEReceiveInterface::getTargetTCPSpeed, py::call_guard<py::gil_scoped_release>())
      .def("getActualDigitalInputBits", &RTDEReceiveInterface::getActualDigitalInputBits,
           py::call_guard<py::gil_scoped_release>())
      .def("getJointTemperatures", &RTDEReceiveInterface::getJointTemperatures,
           py::call_guard<py::gil_scoped_release>())
      .def("getActualExecutionTime", &RTDEReceiveInterface::getActualExecutionTime,
           py::call_guard<py::gil_scoped_release>())
      .def("getRobotModes", &RTDEReceiveInterface::getRobotMode, py::call_guard<py::gil_scoped_release>())
      .def("getJointMode", &RTDEReceiveInterface::getJointMode, py::call_guard<py::gil_scoped_release>())
      .def("getSafetyMode", &RTDEReceiveInterface::getSafetyMode, py::call_guard<py::gil_scoped_release>())
      .def("getActualToolAccelerometer", &RTDEReceiveInterface::getActualToolAccelerometer,
           py::call_guard<py::gil_scoped_release>())
      .def("getSpeedScaling", &RTDEReceiveInterface::getSpeedScaling, py::call_guard<py::gil_scoped_release>())
      .def("getTargetSpeedFraction", &RTDEReceiveInterface::getTargetSpeedFraction,
           py::call_guard<py::gil_scoped_release>())
      .def("getActualMomentum", &RTDEReceiveInterface::getActualMomentum, py::call_guard<py::gil_scoped_release>())
      .def("getActualMainVoltage", &RTDEReceiveInterface::getActualMainVoltage,
           py::call_guard<py::gil_scoped_release>())
      .def("getActualRobotVoltage", &RTDEReceiveInterface::getActualRobotVoltage,
           py::call_guard<py::gil_scoped_release>())
      .def("getActualRobotCurrent", &RTDEReceiveInterface::getActualRobotCurrent,
           py::call_guard<py::gil_scoped_release>())
      .def("getActualJointVoltage", &RTDEReceiveInterface::getActualJointVoltage,
           py::call_guard<py::gil_scoped_release>())
      .def("getActualDigitalOutputBits", &RTDEReceiveInterface::getActualDigitalOutputBits,
           py::call_guard<py::gil_scoped_release>())
      .def("getRuntimeState", &RTDEReceiveInterface::getRuntimeState, py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogInput0", &RTDEReceiveInterface::getStandardAnalogInput0,
           py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogInput1", &RTDEReceiveInterface::getStandardAnalogInput1,
           py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogOutput0", &RTDEReceiveInterface::getStandardAnalogOutput0,
           py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogOutput1", &RTDEReceiveInterface::getStandardAnalogOutput1,
           py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const RTDEReceiveInterface &a)
           {
        return "<rtde_receive.RTDEReceiveInterface>";
      });
}
};

namespace rtde_io
{
PYBIND11_MODULE(rtde_io, m)
{
  m.doc() = "RTDE IO Interface";
  py::class_<RTDEIOInterface>(m, "RTDEIOInterface")
      .def(py::init<std::string>())
      .def("reconnect", &RTDEIOInterface::reconnect)
      .def("setStandardDigitalOut", &RTDEIOInterface::setStandardDigitalOut, py::call_guard<py::gil_scoped_release>())
      .def("setToolDigitalOut", &RTDEIOInterface::setToolDigitalOut, py::call_guard<py::gil_scoped_release>())
      .def("setSpeedSlider", &RTDEIOInterface::setSpeedSlider, py::call_guard<py::gil_scoped_release>())
      .def("setAnalogOutputVoltage", &RTDEIOInterface::setAnalogOutputVoltage, py::call_guard<py::gil_scoped_release>())
      .def("setAnalogOutputCurrent", &RTDEIOInterface::setAnalogOutputCurrent, py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const RTDEIOInterface &a)
           {
        return "<rtde_io.RTDEIOInterface>";
      });
}
};
