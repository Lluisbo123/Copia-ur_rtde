#include <rtde_receive_interface.h>
#include <boost/thread.hpp>
#include <iostream>

RTDEReceiveInterface::RTDEReceiveInterface(std::vector<std::string> variables, std::string hostname, int port)
    : variables_(std::move(variables)), hostname_(std::move(hostname)), port_(port)
{
  rtde_ = std::make_shared<RTDE>(hostname_);
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  auto controller_version = rtde_->getControllerVersion();
  uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

  double frequency = 125;
  // If e-Series Robot set frequency to 500Hz
  if (major_version > CB3_MAJOR_VERSION)
    frequency = 500;

  // Setup output
  rtde_->sendOutputSetup(variables_, frequency);

  // Start RTDE data synchronization
  rtde_->sendStart();

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>();

  // Start executing receiveCallback
  boost::thread th(boost::bind(&RTDEReceiveInterface::receiveCallback, this));

  // Wait until the first robot state has been received
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

RTDEReceiveInterface::~RTDEReceiveInterface()
{
  if (rtde_ != nullptr)
  {
    if (rtde_->isConnected())
      rtde_->disconnect();
  }
}

void RTDEReceiveInterface::receiveCallback()
{
  while (true)
  {
    // Receive and update the robot state
    rtde_->receiveData(robot_state_);
  }
}

double RTDEReceiveInterface::getTimestamp()
{
  return robot_state_->getTimestamp();
}

std::vector<double> RTDEReceiveInterface::getTargetQ()
{
  return robot_state_->getTarget_q();
}

std::vector<double> RTDEReceiveInterface::getTargetQd()
{
  return robot_state_->getTarget_qd();
}

std::vector<double> RTDEReceiveInterface::getTargetQdd()
{
  return robot_state_->getTarget_qdd();
}

std::vector<double> RTDEReceiveInterface::getTargetCurrent()
{
  return robot_state_->getTarget_current();
}

std::vector<double> RTDEReceiveInterface::getTargetMoment()
{
  return robot_state_->getTarget_moment();
}

std::vector<double> RTDEReceiveInterface::getActualQ()
{
  return robot_state_->getActual_q();
}

std::vector<double> RTDEReceiveInterface::getActualQd()
{
  return robot_state_->getActual_qd();
}

std::vector<double> RTDEReceiveInterface::getActualCurrent()
{
  return robot_state_->getActual_current();
}

std::vector<double> RTDEReceiveInterface::getJointControlOutput()
{
  return robot_state_->getJoint_control_output();
}

std::vector<double> RTDEReceiveInterface::getActualTCPPose()
{
  return robot_state_->getActual_TCP_pose();
}

std::vector<double> RTDEReceiveInterface::getActualTCPSpeed()
{
  return robot_state_->getActual_TCP_speed();
}

std::vector<double> RTDEReceiveInterface::getActualTCPForce()
{
  return robot_state_->getActual_TCP_force();
}

std::vector<double> RTDEReceiveInterface::getTargetTCPPose()
{
  return robot_state_->getTarget_TCP_pose();
}

std::vector<double> RTDEReceiveInterface::getTargetTCPSpeed()
{
  return robot_state_->getTarget_TCP_speed();
}

uint64_t RTDEReceiveInterface::getActualDigitalInputBits()
{
  return robot_state_->getActual_digital_input_bits();
}

std::vector<double> RTDEReceiveInterface::getJointTemperatures()
{
  return robot_state_->getJoint_temperatures();
}

double RTDEReceiveInterface::getActualExecutionTime()
{
  return robot_state_->getActual_execution_time();
}

int32_t RTDEReceiveInterface::getRobotMode()
{
  return robot_state_->getRobot_mode();
}

std::vector<int32_t> RTDEReceiveInterface::getJointMode()
{
  return robot_state_->getJoint_mode();
}

int32_t RTDEReceiveInterface::getSafetyMode()
{
  return robot_state_->getSafety_mode();
}

std::vector<double> RTDEReceiveInterface::getActualToolAccelerometer()
{
  return robot_state_->getActual_tool_accelerometer();
}

double RTDEReceiveInterface::getSpeedScaling()
{
  return robot_state_->getSpeed_scaling();
}

double RTDEReceiveInterface::getTargetSpeedFraction()
{
  return robot_state_->getTarget_speed_fraction();
}

double RTDEReceiveInterface::getActualMomentum()
{
  return robot_state_->getActual_momentum();
}

double RTDEReceiveInterface::getActualMainVoltage()
{
  return robot_state_->getActual_main_voltage();
}

double RTDEReceiveInterface::getActualRobotVoltage()
{
  return robot_state_->getActual_robot_voltage();
}

double RTDEReceiveInterface::getActualRobotCurrent()
{
  return robot_state_->getActual_robot_current();
}

std::vector<double> RTDEReceiveInterface::getActualJointVoltage()
{
  return robot_state_->getActual_joint_voltage();
}

uint64_t RTDEReceiveInterface::getActualDigitalOutputBits()
{
  return robot_state_->getActual_digital_output_bits();
}

uint32_t RTDEReceiveInterface::getRuntimeState()
{
  return robot_state_->getRuntime_state();
}
