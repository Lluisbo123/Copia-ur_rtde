#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_control_interface_doc.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_io_interface_doc.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_receive_interface_doc.h>
#include <ur_rtde/script_client.h>

namespace py = pybind11;
using namespace ur_rtde;

namespace rtde_control
{
PYBIND11_MODULE(rtde_control, m)
{
  m.doc() = "RTDE Control Interface";
  py::class_<RTDEControlInterface>(m, "RTDEControlInterface")
      .def(py::init<std::string>())
      .def("reconnect", &RTDEControlInterface::reconnect, DOC(ur_rtde, RTDEControlInterface, reconnect),
           py::call_guard<py::gil_scoped_release>())
      .def("isConnected", &RTDEControlInterface::isConnected, DOC(ur_rtde, RTDEControlInterface, isConnected),
           py::call_guard<py::gil_scoped_release>())
      .def("sendCustomScriptFunction", &RTDEControlInterface::sendCustomScriptFunction,
           DOC(ur_rtde, RTDEControlInterface, sendCustomScriptFunction), py::call_guard<py::gil_scoped_release>())
      .def("sendCustomScriptFile", &RTDEControlInterface::sendCustomScriptFile,
           DOC(ur_rtde, RTDEControlInterface, sendCustomScriptFile), py::call_guard<py::gil_scoped_release>())
      .def("stopRobot", &RTDEControlInterface::stopRobot, DOC(ur_rtde, RTDEControlInterface, stopRobot),
           py::call_guard<py::gil_scoped_release>())
      .def("moveJ",
           (bool (RTDEControlInterface::*)(const std::vector<std::vector<double>> &path)) & RTDEControlInterface::moveJ,
           DOC(ur_rtde, RTDEControlInterface, moveJ_2), py::call_guard<py::gil_scoped_release>())
      .def("moveJ",
           (bool (RTDEControlInterface::*)(const std::vector<double> &q, double speed, double acceleration)) &
               RTDEControlInterface::moveJ,
           DOC(ur_rtde, RTDEControlInterface, moveJ), py::arg("q"), py::arg("speed") = 1.05,
           py::arg("acceleration") = 1.4, py::call_guard<py::gil_scoped_release>())
      .def("moveJ_IK", &RTDEControlInterface::moveJ_IK, DOC(ur_rtde, RTDEControlInterface, moveJ_IK), py::arg("pose"),
           py::arg("speed") = 1.05, py::arg("acceleration") = 1.4, py::call_guard<py::gil_scoped_release>())
      .def("moveL",
           (bool (RTDEControlInterface::*)(const std::vector<std::vector<double>> &path)) & RTDEControlInterface::moveL,
           DOC(ur_rtde, RTDEControlInterface, moveL_2), py::call_guard<py::gil_scoped_release>())
      .def("moveL",
           (bool (RTDEControlInterface::*)(const std::vector<double> &pose, double speed, double acceleration)) &
               RTDEControlInterface::moveL,
           DOC(ur_rtde, RTDEControlInterface, moveL), py::arg("pose"), py::arg("speed") = 0.25,
           py::arg("acceleration") = 1.2, py::call_guard<py::gil_scoped_release>())
      .def("moveL_FK", &RTDEControlInterface::moveL_FK, DOC(ur_rtde, RTDEControlInterface, moveL_FK), py::arg("q"),
           py::arg("speed") = 0.25, py::arg("acceleration") = 1.2, py::call_guard<py::gil_scoped_release>())
      .def("moveC", &RTDEControlInterface::moveC, DOC(ur_rtde, RTDEControlInterface, moveC),
           py::call_guard<py::gil_scoped_release>())
      .def("speedJ", &RTDEControlInterface::speedJ, DOC(ur_rtde, RTDEControlInterface, speedJ), py::arg("qd"),
           py::arg("acceleration") = 0.5, py::arg("time") = 0.0, py::call_guard<py::gil_scoped_release>())
      .def("speedL", &RTDEControlInterface::speedL, DOC(ur_rtde, RTDEControlInterface, speedL), py::arg("xd"),
           py::arg("acceleration") = 0.25, py::arg("time") = 0.0, py::call_guard<py::gil_scoped_release>())
      .def("speedStop", &RTDEControlInterface::speedStop, DOC(ur_rtde, RTDEControlInterface, speedStop),
           py::call_guard<py::gil_scoped_release>())
      .def("servoJ", &RTDEControlInterface::servoJ, DOC(ur_rtde, RTDEControlInterface, servoJ),
           py::call_guard<py::gil_scoped_release>())
      .def("servoL", &RTDEControlInterface::servoL, DOC(ur_rtde, RTDEControlInterface, servoL),
           py::call_guard<py::gil_scoped_release>())
      .def("servoC", &RTDEControlInterface::servoC, DOC(ur_rtde, RTDEControlInterface, servoC), py::arg("pose"),
           py::arg("speed") = 0.25, py::arg("acceleration") = 1.2, py::arg("blend") = 0.0,
           py::call_guard<py::gil_scoped_release>())
      .def("servoStop", &RTDEControlInterface::servoStop, DOC(ur_rtde, RTDEControlInterface, servoStop),
           py::call_guard<py::gil_scoped_release>())
      .def("forceModeStart", &RTDEControlInterface::forceModeStart, DOC(ur_rtde, RTDEControlInterface, forceModeStart),
           py::call_guard<py::gil_scoped_release>())
      .def("forceModeStop", &RTDEControlInterface::forceModeStop, DOC(ur_rtde, RTDEControlInterface, forceModeStop),
           py::call_guard<py::gil_scoped_release>())
      .def("forceModeSetDamping", &RTDEControlInterface::forceModeSetDamping,
           DOC(ur_rtde, RTDEControlInterface, forceModeSetDamping), py::call_guard<py::gil_scoped_release>())
      .def("toolContact", &RTDEControlInterface::toolContact, DOC(ur_rtde, RTDEControlInterface, toolContact),
           py::call_guard<py::gil_scoped_release>())
      .def("getTargetWaypoint", &RTDEControlInterface::getTargetWaypoint,
           DOC(ur_rtde, RTDEControlInterface, getTargetWaypoint), py::call_guard<py::gil_scoped_release>())
      .def("getActualJointPositionsHistory", &RTDEControlInterface::getActualJointPositionsHistory,
           DOC(ur_rtde, RTDEControlInterface, getActualJointPositionsHistory), py::call_guard<py::gil_scoped_release>())
      .def("getStepTime", &RTDEControlInterface::getStepTime, DOC(ur_rtde, RTDEControlInterface, getStepTime),
           py::call_guard<py::gil_scoped_release>())
      .def("teachMode", &RTDEControlInterface::teachMode, DOC(ur_rtde, RTDEControlInterface, teachMode),
           py::call_guard<py::gil_scoped_release>())
      .def("endTeachMode", &RTDEControlInterface::endTeachMode, DOC(ur_rtde, RTDEControlInterface, endTeachMode),
           py::call_guard<py::gil_scoped_release>())
      .def("forceModeSetGainScaling", &RTDEControlInterface::forceModeSetGainScaling,
           DOC(ur_rtde, RTDEControlInterface, forceModeSetGainScaling), py::call_guard<py::gil_scoped_release>())
      .def("zeroFtSensor", &RTDEControlInterface::zeroFtSensor, DOC(ur_rtde, RTDEControlInterface, zeroFtSensor),
           py::call_guard<py::gil_scoped_release>())
      .def("setPayload", &RTDEControlInterface::setPayload, DOC(ur_rtde, RTDEControlInterface, setPayload),
           py::call_guard<py::gil_scoped_release>())
      .def("setTcp", &RTDEControlInterface::setTcp, DOC(ur_rtde, RTDEControlInterface, setTcp),
           py::call_guard<py::gil_scoped_release>())
      .def("getInverseKinematics", &RTDEControlInterface::getInverseKinematics,
           DOC(ur_rtde, RTDEControlInterface, getInverseKinematics), py::arg("x"), py::arg("qnear"),
           py::arg("max_position_error") = 1e-10, py::arg("max_orientation_error") = 1e-10,
           py::call_guard<py::gil_scoped_release>())
      .def("triggerProtectiveStop", &RTDEControlInterface::triggerProtectiveStop,
           DOC(ur_rtde, RTDEControlInterface, triggerProtectiveStop), py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const RTDEControlInterface &a) { return "<rtde_control.RTDEControlInterface>"; });
}
};  // namespace rtde_control

namespace rtde_receive
{
PYBIND11_MODULE(rtde_receive, m)
{
  m.doc() = "RTDE Receive Interface";
  py::class_<RTDEReceiveInterface>(m, "RTDEReceiveInterface")
      .def(py::init<std::string>())
      .def("reconnect", &RTDEReceiveInterface::reconnect, DOC(ur_rtde, RTDEReceiveInterface, reconnect), py::call_guard<py::gil_scoped_release>())
      .def("isConnected", &RTDEReceiveInterface::isConnected, DOC(ur_rtde, RTDEReceiveInterface, isConnected), py::call_guard<py::gil_scoped_release>())
      .def("getTimestamp", &RTDEReceiveInterface::getTimestamp, DOC(ur_rtde, RTDEReceiveInterface, getTimestamp), py::call_guard<py::gil_scoped_release>())
      .def("getTargetQ", &RTDEReceiveInterface::getTargetQ, DOC(ur_rtde, RTDEReceiveInterface, getTargetQ), py::call_guard<py::gil_scoped_release>())
      .def("getTargetQd", &RTDEReceiveInterface::getTargetQd, DOC(ur_rtde, RTDEReceiveInterface, getTargetQd), py::call_guard<py::gil_scoped_release>())
      .def("getTargetQdd", &RTDEReceiveInterface::getTargetQdd, DOC(ur_rtde, RTDEReceiveInterface, getTargetQdd), py::call_guard<py::gil_scoped_release>())
      .def("getTargetCurrent", &RTDEReceiveInterface::getTargetCurrent, DOC(ur_rtde, RTDEReceiveInterface, getTargetCurrent), py::call_guard<py::gil_scoped_release>())
      .def("getTargetMoment", &RTDEReceiveInterface::getTargetMoment, DOC(ur_rtde, RTDEReceiveInterface, getTargetMoment), py::call_guard<py::gil_scoped_release>())
      .def("getActualQ", &RTDEReceiveInterface::getActualQ, DOC(ur_rtde, RTDEReceiveInterface, getActualQ), py::call_guard<py::gil_scoped_release>())
      .def("getActualQd", &RTDEReceiveInterface::getActualQd, DOC(ur_rtde, RTDEReceiveInterface, getActualQd), py::call_guard<py::gil_scoped_release>())
      .def("getActualCurrent", &RTDEReceiveInterface::getActualCurrent, DOC(ur_rtde, RTDEReceiveInterface, getActualCurrent), py::call_guard<py::gil_scoped_release>())
      .def("getJointControlOutput", &RTDEReceiveInterface::getJointControlOutput, DOC(ur_rtde, RTDEReceiveInterface, getJointControlOutput),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualTCPPose", &RTDEReceiveInterface::getActualTCPPose, DOC(ur_rtde, RTDEReceiveInterface, getActualTCPPose), py::call_guard<py::gil_scoped_release>())
      .def("getActualTCPSpeed", &RTDEReceiveInterface::getActualTCPSpeed, DOC(ur_rtde, RTDEReceiveInterface, getActualTCPSpeed), py::call_guard<py::gil_scoped_release>())
      .def("getActualTCPForce", &RTDEReceiveInterface::getActualTCPForce, DOC(ur_rtde, RTDEReceiveInterface, getActualTCPForce), py::call_guard<py::gil_scoped_release>())
      .def("getTargetTCPPose", &RTDEReceiveInterface::getTargetTCPPose, DOC(ur_rtde, RTDEReceiveInterface, getTargetTCPPose), py::call_guard<py::gil_scoped_release>())
      .def("getTargetTCPSpeed", &RTDEReceiveInterface::getTargetTCPSpeed, DOC(ur_rtde, RTDEReceiveInterface, getTargetTCPSpeed), py::call_guard<py::gil_scoped_release>())
      .def("getActualDigitalInputBits", &RTDEReceiveInterface::getActualDigitalInputBits, DOC(ur_rtde, RTDEReceiveInterface, getActualDigitalInputBits),
           py::call_guard<py::gil_scoped_release>())
      .def("getJointTemperatures", &RTDEReceiveInterface::getJointTemperatures, DOC(ur_rtde, RTDEReceiveInterface, getJointTemperatures),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualExecutionTime", &RTDEReceiveInterface::getActualExecutionTime, DOC(ur_rtde, RTDEReceiveInterface, getActualExecutionTime),
           py::call_guard<py::gil_scoped_release>())
      .def("getRobotMode", &RTDEReceiveInterface::getRobotMode, DOC(ur_rtde, RTDEReceiveInterface, getRobotMode), py::call_guard<py::gil_scoped_release>())
      .def("getJointMode", &RTDEReceiveInterface::getJointMode, DOC(ur_rtde, RTDEReceiveInterface, getJointMode), py::call_guard<py::gil_scoped_release>())
      .def("getSafetyMode", &RTDEReceiveInterface::getSafetyMode, DOC(ur_rtde, RTDEReceiveInterface, getSafetyMode), py::call_guard<py::gil_scoped_release>())
      .def("getActualToolAccelerometer", &RTDEReceiveInterface::getActualToolAccelerometer, DOC(ur_rtde, RTDEReceiveInterface, getActualToolAccelerometer),
           py::call_guard<py::gil_scoped_release>())
      .def("getSpeedScaling", &RTDEReceiveInterface::getSpeedScaling, DOC(ur_rtde, RTDEReceiveInterface, getSpeedScaling), py::call_guard<py::gil_scoped_release>())
      .def("getTargetSpeedFraction", &RTDEReceiveInterface::getTargetSpeedFraction, DOC(ur_rtde, RTDEReceiveInterface, getTargetSpeedFraction),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualMomentum", &RTDEReceiveInterface::getActualMomentum, DOC(ur_rtde, RTDEReceiveInterface, getActualMomentum), py::call_guard<py::gil_scoped_release>())
      .def("getActualMainVoltage", &RTDEReceiveInterface::getActualMainVoltage, DOC(ur_rtde, RTDEReceiveInterface, getActualMainVoltage),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualRobotVoltage", &RTDEReceiveInterface::getActualRobotVoltage, DOC(ur_rtde, RTDEReceiveInterface, getActualRobotVoltage),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualRobotCurrent", &RTDEReceiveInterface::getActualRobotCurrent, DOC(ur_rtde, RTDEReceiveInterface, getActualRobotCurrent),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualJointVoltage", &RTDEReceiveInterface::getActualJointVoltage, DOC(ur_rtde, RTDEReceiveInterface, getActualJointVoltage),
           py::call_guard<py::gil_scoped_release>())
      .def("getActualDigitalOutputBits", &RTDEReceiveInterface::getActualDigitalOutputBits, DOC(ur_rtde, RTDEReceiveInterface, getActualDigitalOutputBits),
           py::call_guard<py::gil_scoped_release>())
      .def("getRuntimeState", &RTDEReceiveInterface::getRuntimeState, DOC(ur_rtde, RTDEReceiveInterface, getRuntimeState), py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogInput0", &RTDEReceiveInterface::getStandardAnalogInput0, DOC(ur_rtde, RTDEReceiveInterface, getStandardAnalogInput0),
           py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogInput1", &RTDEReceiveInterface::getStandardAnalogInput1, DOC(ur_rtde, RTDEReceiveInterface, getStandardAnalogInput1),
           py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogOutput0", &RTDEReceiveInterface::getStandardAnalogOutput0, DOC(ur_rtde, RTDEReceiveInterface, getStandardAnalogOutput0),
           py::call_guard<py::gil_scoped_release>())
      .def("getStandardAnalogOutput1", &RTDEReceiveInterface::getStandardAnalogOutput1, DOC(ur_rtde, RTDEReceiveInterface, getStandardAnalogOutput1),
           py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const RTDEReceiveInterface &a) { return "<rtde_receive.RTDEReceiveInterface>"; });
}
};  // namespace rtde_receive

namespace rtde_io
{
PYBIND11_MODULE(rtde_io, m)
{
  m.doc() = "RTDE IO Interface";
  py::class_<RTDEIOInterface>(m, "RTDEIOInterface")
      .def(py::init<std::string>())
      .def("reconnect", &RTDEIOInterface::reconnect)
      .def("setStandardDigitalOut", &RTDEIOInterface::setStandardDigitalOut, DOC(ur_rtde, RTDEIOInterface, setStandardDigitalOut), py::call_guard<py::gil_scoped_release>())
      .def("setToolDigitalOut", &RTDEIOInterface::setToolDigitalOut, DOC(ur_rtde, RTDEIOInterface, setToolDigitalOut), py::call_guard<py::gil_scoped_release>())
      .def("setSpeedSlider", &RTDEIOInterface::setSpeedSlider, DOC(ur_rtde, RTDEIOInterface, setSpeedSlider), py::call_guard<py::gil_scoped_release>())
      .def("setAnalogOutputVoltage", &RTDEIOInterface::setAnalogOutputVoltage, DOC(ur_rtde, RTDEIOInterface, setAnalogOutputVoltage), py::call_guard<py::gil_scoped_release>())
      .def("setAnalogOutputCurrent", &RTDEIOInterface::setAnalogOutputCurrent, DOC(ur_rtde, RTDEIOInterface, setAnalogOutputCurrent), py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const RTDEIOInterface &a) { return "<rtde_io.RTDEIOInterface>"; });
}
};  // namespace rtde_io

namespace script_client
{
PYBIND11_MODULE(script_client, m)
{
  m.doc() = "Script Client";
  py::class_<ScriptClient>(m, "ScriptClient")
      .def(py::init<std::string, uint32_t, uint32_t>())
      .def("connect", &ScriptClient::connect, py::call_guard<py::gil_scoped_release>())
      .def("isConnected", &ScriptClient::isConnected, py::call_guard<py::gil_scoped_release>())
      .def("disconnect", &ScriptClient::disconnect, py::call_guard<py::gil_scoped_release>())
      .def("sendScript", (bool (ScriptClient::*)()) & ScriptClient::sendScript,
           py::call_guard<py::gil_scoped_release>())
      .def("sendScript", (bool (ScriptClient::*)(const std::string &file_name)) & ScriptClient::sendScript,
           py::call_guard<py::gil_scoped_release>())
      .def("sendScriptCommand", &ScriptClient::sendScriptCommand, py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const ScriptClient &a) { return "<script_client.ScriptClient>"; });
}
};  // namespace script_client

namespace dashboard_client
{
PYBIND11_MODULE(dashboard_client, m)
{
  m.doc() = "Dashboard Client";
  py::class_<DashboardClient>(m, "DashboardClient")
      .def(py::init<std::string>())
      .def("connect", &DashboardClient::connect, py::call_guard<py::gil_scoped_release>())
      .def("isConnected", &DashboardClient::isConnected, py::call_guard<py::gil_scoped_release>())
      .def("disconnect", &DashboardClient::disconnect, py::call_guard<py::gil_scoped_release>())
      .def("send", &DashboardClient::send, py::call_guard<py::gil_scoped_release>())
      .def("receive", &DashboardClient::receive, py::call_guard<py::gil_scoped_release>())
      .def("loadURP", &DashboardClient::loadURP, py::call_guard<py::gil_scoped_release>())
      .def("play", &DashboardClient::play, py::call_guard<py::gil_scoped_release>())
      .def("stop", &DashboardClient::stop, py::call_guard<py::gil_scoped_release>())
      .def("pause", &DashboardClient::pause, py::call_guard<py::gil_scoped_release>())
      .def("quit", &DashboardClient::quit, py::call_guard<py::gil_scoped_release>())
      .def("shutdown", &DashboardClient::shutdown, py::call_guard<py::gil_scoped_release>())
      .def("running", &DashboardClient::running, py::call_guard<py::gil_scoped_release>())
      .def("popup", &DashboardClient::popup, py::call_guard<py::gil_scoped_release>())
      .def("closePopup", &DashboardClient::closePopup, py::call_guard<py::gil_scoped_release>())
      .def("closeSafetyPopup", &DashboardClient::closeSafetyPopup, py::call_guard<py::gil_scoped_release>())
      .def("powerOn", &DashboardClient::powerOn, py::call_guard<py::gil_scoped_release>())
      .def("powerOff", &DashboardClient::powerOff, py::call_guard<py::gil_scoped_release>())
      .def("brakeRelease", &DashboardClient::brakeRelease, py::call_guard<py::gil_scoped_release>())
      .def("unlockProtectiveStop", &DashboardClient::unlockProtectiveStop, py::call_guard<py::gil_scoped_release>())
      .def("restartSafety", &DashboardClient::restartSafety, py::call_guard<py::gil_scoped_release>())
      .def("polyscopeVersion", &DashboardClient::polyscopeVersion, py::call_guard<py::gil_scoped_release>())
      .def("programState", &DashboardClient::programState, py::call_guard<py::gil_scoped_release>())
      .def("robotmode", &DashboardClient::robotmode, py::call_guard<py::gil_scoped_release>())
      .def("getLoadedProgram", &DashboardClient::getLoadedProgram, py::call_guard<py::gil_scoped_release>())
      .def("safetymode", &DashboardClient::safetymode, py::call_guard<py::gil_scoped_release>())
      .def("safetystatus", &DashboardClient::safetystatus, py::call_guard<py::gil_scoped_release>())
      .def("addToLog", &DashboardClient::addToLog, py::call_guard<py::gil_scoped_release>())
      .def("isProgramSaved", &DashboardClient::isProgramSaved, py::call_guard<py::gil_scoped_release>())
      .def("setUserRole", &DashboardClient::setUserRole, py::call_guard<py::gil_scoped_release>())
      .def("__repr__", [](const DashboardClient &a) { return "<dashboard_client.DashboardClient>"; });
}
};  // namespace dashboard_client
