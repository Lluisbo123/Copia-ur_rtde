#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/robot_state.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <bitset>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <iostream>
#include <thread>

namespace ur_rtde
{
RTDEReceiveInterface::RTDEReceiveInterface(std::string hostname, std::vector<std::string> variables, bool verbose,
                                           bool use_upper_range_registers)
    : variables_(std::move(variables)),
      hostname_(std::move(hostname)),
      verbose_(verbose),
      use_upper_range_registers_(use_upper_range_registers)
{
  port_ = 30004;
  rtde_ = std::make_shared<RTDE>(hostname_, port_, verbose_);
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  auto controller_version = rtde_->getControllerVersion();
  uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

  frequency_ = 125;
  // If e-Series Robot set frequency to 500Hz
  if (major_version > CB3_MAJOR_VERSION)
    frequency_ = 500;

  // Set delta time to be used by receiveCallback
  delta_time_ = 1 / frequency_;

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>();

  // Map the output registers to functions
  initOutputRegFuncMap();

  // Init pausing state
  pausing_state_ = PausingState::RUNNING;
  pausing_ramp_up_increment_ = 0.01;

  if (use_upper_range_registers_)
    register_offset_ = 24;
  else
    register_offset_ = 0;

  // Setup recipes
  setupRecipes(frequency_);

  // Start RTDE data synchronization
  rtde_->sendStart();

  // Start executing receiveCallback
  th_ = std::make_shared<boost::thread>(boost::bind(&RTDEReceiveInterface::receiveCallback, this));

  // Wait until the first robot state has been received
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

RTDEReceiveInterface::~RTDEReceiveInterface()
{
  disconnect();
}

void RTDEReceiveInterface::disconnect()
{
  // Stop the receive callback function
  stop_thread = true;
  th_->interrupt();
  th_->join();

  if (rtde_ != nullptr)
  {
    if (rtde_->isConnected())
      rtde_->disconnect();
  }

  // Wait until everything has disconnected
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

bool RTDEReceiveInterface::setupRecipes(const double& frequency)
{
  if (variables_.empty())
  {
    // Assume all variables
    variables_ = {"timestamp",
                  "target_q",
                  "target_qd",
                  "target_qdd",
                  "target_current",
                  "target_moment",
                  "actual_q",
                  "actual_qd",
                  "actual_current",
                  "joint_control_output",
                  "actual_TCP_pose",
                  "actual_TCP_speed",
                  "actual_TCP_force",
                  "target_TCP_pose",
                  "target_TCP_speed",
                  "actual_digital_input_bits",
                  "joint_temperatures",
                  "actual_execution_time",
                  "robot_mode",
                  "joint_mode",
                  "safety_mode",
                  "actual_tool_accelerometer",
                  "speed_scaling",
                  "target_speed_fraction",
                  "actual_momentum",
                  "actual_main_voltage",
                  "actual_robot_voltage",
                  "actual_robot_current",
                  "actual_joint_voltage",
                  "actual_digital_output_bits",
                  "runtime_state",
                  "standard_analog_input0",
                  "standard_analog_input1",
                  "standard_analog_output0",
                  "standard_analog_output1",
                  "robot_status_bits",
                  "safety_status_bits",
                  outIntReg(2),
                  outIntReg(12),
                  outIntReg(13),
                  outIntReg(14),
                  outIntReg(15),
                  outIntReg(16),
                  outIntReg(17),
                  outIntReg(18),
                  outIntReg(19),
                  outDoubleReg(12),
                  outDoubleReg(13),
                  outDoubleReg(14),
                  outDoubleReg(15),
                  outDoubleReg(16),
                  outDoubleReg(17),
                  outDoubleReg(18),
                  outDoubleReg(19)};
  }

  // Setup output
  rtde_->sendOutputSetup(variables_, frequency);
  return true;
}

void RTDEReceiveInterface::initOutputRegFuncMap()
{
  output_reg_func_map_["getOutput_int_register_0"] = std::bind(&RobotState::getOutput_int_register_0, robot_state_);
  output_reg_func_map_["getOutput_int_register_1"] = std::bind(&RobotState::getOutput_int_register_1, robot_state_);
  output_reg_func_map_["getOutput_int_register_2"] = std::bind(&RobotState::getOutput_int_register_2, robot_state_);
  output_reg_func_map_["getOutput_int_register_3"] = std::bind(&RobotState::getOutput_int_register_3, robot_state_);
  output_reg_func_map_["getOutput_int_register_4"] = std::bind(&RobotState::getOutput_int_register_4, robot_state_);
  output_reg_func_map_["getOutput_int_register_5"] = std::bind(&RobotState::getOutput_int_register_5, robot_state_);
  output_reg_func_map_["getOutput_int_register_6"] = std::bind(&RobotState::getOutput_int_register_6, robot_state_);
  output_reg_func_map_["getOutput_int_register_7"] = std::bind(&RobotState::getOutput_int_register_7, robot_state_);
  output_reg_func_map_["getOutput_int_register_8"] = std::bind(&RobotState::getOutput_int_register_8, robot_state_);
  output_reg_func_map_["getOutput_int_register_9"] = std::bind(&RobotState::getOutput_int_register_9, robot_state_);
  output_reg_func_map_["getOutput_int_register_10"] = std::bind(&RobotState::getOutput_int_register_10, robot_state_);
  output_reg_func_map_["getOutput_int_register_11"] = std::bind(&RobotState::getOutput_int_register_11, robot_state_);
  output_reg_func_map_["getOutput_int_register_12"] = std::bind(&RobotState::getOutput_int_register_12, robot_state_);
  output_reg_func_map_["getOutput_int_register_13"] = std::bind(&RobotState::getOutput_int_register_13, robot_state_);
  output_reg_func_map_["getOutput_int_register_14"] = std::bind(&RobotState::getOutput_int_register_14, robot_state_);
  output_reg_func_map_["getOutput_int_register_15"] = std::bind(&RobotState::getOutput_int_register_15, robot_state_);
  output_reg_func_map_["getOutput_int_register_16"] = std::bind(&RobotState::getOutput_int_register_16, robot_state_);
  output_reg_func_map_["getOutput_int_register_17"] = std::bind(&RobotState::getOutput_int_register_17, robot_state_);
  output_reg_func_map_["getOutput_int_register_18"] = std::bind(&RobotState::getOutput_int_register_18, robot_state_);
  output_reg_func_map_["getOutput_int_register_19"] = std::bind(&RobotState::getOutput_int_register_19, robot_state_);
  output_reg_func_map_["getOutput_int_register_20"] = std::bind(&RobotState::getOutput_int_register_20, robot_state_);
  output_reg_func_map_["getOutput_int_register_21"] = std::bind(&RobotState::getOutput_int_register_21, robot_state_);
  output_reg_func_map_["getOutput_int_register_22"] = std::bind(&RobotState::getOutput_int_register_22, robot_state_);
  output_reg_func_map_["getOutput_int_register_23"] = std::bind(&RobotState::getOutput_int_register_23, robot_state_);
  output_reg_func_map_["getOutput_int_register_24"] = std::bind(&RobotState::getOutput_int_register_24, robot_state_);
  output_reg_func_map_["getOutput_int_register_25"] = std::bind(&RobotState::getOutput_int_register_25, robot_state_);
  output_reg_func_map_["getOutput_int_register_26"] = std::bind(&RobotState::getOutput_int_register_26, robot_state_);
  output_reg_func_map_["getOutput_int_register_27"] = std::bind(&RobotState::getOutput_int_register_27, robot_state_);
  output_reg_func_map_["getOutput_int_register_28"] = std::bind(&RobotState::getOutput_int_register_28, robot_state_);
  output_reg_func_map_["getOutput_int_register_29"] = std::bind(&RobotState::getOutput_int_register_29, robot_state_);
  output_reg_func_map_["getOutput_int_register_30"] = std::bind(&RobotState::getOutput_int_register_30, robot_state_);
  output_reg_func_map_["getOutput_int_register_31"] = std::bind(&RobotState::getOutput_int_register_31, robot_state_);
  output_reg_func_map_["getOutput_int_register_32"] = std::bind(&RobotState::getOutput_int_register_32, robot_state_);
  output_reg_func_map_["getOutput_int_register_33"] = std::bind(&RobotState::getOutput_int_register_33, robot_state_);
  output_reg_func_map_["getOutput_int_register_34"] = std::bind(&RobotState::getOutput_int_register_34, robot_state_);
  output_reg_func_map_["getOutput_int_register_35"] = std::bind(&RobotState::getOutput_int_register_35, robot_state_);
  output_reg_func_map_["getOutput_int_register_36"] = std::bind(&RobotState::getOutput_int_register_36, robot_state_);
  output_reg_func_map_["getOutput_int_register_37"] = std::bind(&RobotState::getOutput_int_register_37, robot_state_);
  output_reg_func_map_["getOutput_int_register_38"] = std::bind(&RobotState::getOutput_int_register_38, robot_state_);
  output_reg_func_map_["getOutput_int_register_39"] = std::bind(&RobotState::getOutput_int_register_39, robot_state_);
  output_reg_func_map_["getOutput_int_register_40"] = std::bind(&RobotState::getOutput_int_register_40, robot_state_);
  output_reg_func_map_["getOutput_int_register_41"] = std::bind(&RobotState::getOutput_int_register_41, robot_state_);
  output_reg_func_map_["getOutput_int_register_42"] = std::bind(&RobotState::getOutput_int_register_42, robot_state_);
  output_reg_func_map_["getOutput_int_register_43"] = std::bind(&RobotState::getOutput_int_register_43, robot_state_);
  output_reg_func_map_["getOutput_int_register_44"] = std::bind(&RobotState::getOutput_int_register_44, robot_state_);
  output_reg_func_map_["getOutput_int_register_45"] = std::bind(&RobotState::getOutput_int_register_45, robot_state_);
  output_reg_func_map_["getOutput_int_register_46"] = std::bind(&RobotState::getOutput_int_register_46, robot_state_);
  output_reg_func_map_["getOutput_int_register_47"] = std::bind(&RobotState::getOutput_int_register_47, robot_state_);

  output_reg_func_map_["getOutput_double_register_0"] =
      std::bind(&RobotState::getOutput_double_register_0, robot_state_);
  output_reg_func_map_["getOutput_double_register_1"] =
      std::bind(&RobotState::getOutput_double_register_1, robot_state_);
  output_reg_func_map_["getOutput_double_register_2"] =
      std::bind(&RobotState::getOutput_double_register_2, robot_state_);
  output_reg_func_map_["getOutput_double_register_3"] =
      std::bind(&RobotState::getOutput_double_register_3, robot_state_);
  output_reg_func_map_["getOutput_double_register_4"] =
      std::bind(&RobotState::getOutput_double_register_4, robot_state_);
  output_reg_func_map_["getOutput_double_register_5"] =
      std::bind(&RobotState::getOutput_double_register_5, robot_state_);
  output_reg_func_map_["getOutput_double_register_6"] =
      std::bind(&RobotState::getOutput_double_register_6, robot_state_);
  output_reg_func_map_["getOutput_double_register_7"] =
      std::bind(&RobotState::getOutput_double_register_7, robot_state_);
  output_reg_func_map_["getOutput_double_register_8"] =
      std::bind(&RobotState::getOutput_double_register_8, robot_state_);
  output_reg_func_map_["getOutput_double_register_9"] =
      std::bind(&RobotState::getOutput_double_register_9, robot_state_);
  output_reg_func_map_["getOutput_double_register_10"] =
      std::bind(&RobotState::getOutput_double_register_10, robot_state_);
  output_reg_func_map_["getOutput_double_register_11"] =
      std::bind(&RobotState::getOutput_double_register_11, robot_state_);
  output_reg_func_map_["getOutput_double_register_12"] =
      std::bind(&RobotState::getOutput_double_register_12, robot_state_);
  output_reg_func_map_["getOutput_double_register_13"] =
      std::bind(&RobotState::getOutput_double_register_13, robot_state_);
  output_reg_func_map_["getOutput_double_register_14"] =
      std::bind(&RobotState::getOutput_double_register_14, robot_state_);
  output_reg_func_map_["getOutput_double_register_15"] =
      std::bind(&RobotState::getOutput_double_register_15, robot_state_);
  output_reg_func_map_["getOutput_double_register_16"] =
      std::bind(&RobotState::getOutput_double_register_16, robot_state_);
  output_reg_func_map_["getOutput_double_register_17"] =
      std::bind(&RobotState::getOutput_double_register_17, robot_state_);
  output_reg_func_map_["getOutput_double_register_18"] =
      std::bind(&RobotState::getOutput_double_register_18, robot_state_);
  output_reg_func_map_["getOutput_double_register_19"] =
      std::bind(&RobotState::getOutput_double_register_19, robot_state_);
  output_reg_func_map_["getOutput_double_register_20"] =
      std::bind(&RobotState::getOutput_double_register_20, robot_state_);
  output_reg_func_map_["getOutput_double_register_21"] =
      std::bind(&RobotState::getOutput_double_register_21, robot_state_);
  output_reg_func_map_["getOutput_double_register_22"] =
      std::bind(&RobotState::getOutput_double_register_22, robot_state_);
  output_reg_func_map_["getOutput_double_register_23"] =
      std::bind(&RobotState::getOutput_double_register_23, robot_state_);
  output_reg_func_map_["getOutput_double_register_24"] =
      std::bind(&RobotState::getOutput_double_register_24, robot_state_);
  output_reg_func_map_["getOutput_double_register_25"] =
      std::bind(&RobotState::getOutput_double_register_25, robot_state_);
  output_reg_func_map_["getOutput_double_register_26"] =
      std::bind(&RobotState::getOutput_double_register_26, robot_state_);
  output_reg_func_map_["getOutput_double_register_27"] =
      std::bind(&RobotState::getOutput_double_register_27, robot_state_);
  output_reg_func_map_["getOutput_double_register_28"] =
      std::bind(&RobotState::getOutput_double_register_28, robot_state_);
  output_reg_func_map_["getOutput_double_register_29"] =
      std::bind(&RobotState::getOutput_double_register_29, robot_state_);
  output_reg_func_map_["getOutput_double_register_30"] =
      std::bind(&RobotState::getOutput_double_register_30, robot_state_);
  output_reg_func_map_["getOutput_double_register_31"] =
      std::bind(&RobotState::getOutput_double_register_31, robot_state_);
  output_reg_func_map_["getOutput_double_register_32"] =
      std::bind(&RobotState::getOutput_double_register_32, robot_state_);
  output_reg_func_map_["getOutput_double_register_33"] =
      std::bind(&RobotState::getOutput_double_register_33, robot_state_);
  output_reg_func_map_["getOutput_double_register_34"] =
      std::bind(&RobotState::getOutput_double_register_34, robot_state_);
  output_reg_func_map_["getOutput_double_register_35"] =
      std::bind(&RobotState::getOutput_double_register_35, robot_state_);
  output_reg_func_map_["getOutput_double_register_36"] =
      std::bind(&RobotState::getOutput_double_register_36, robot_state_);
  output_reg_func_map_["getOutput_double_register_37"] =
      std::bind(&RobotState::getOutput_double_register_37, robot_state_);
  output_reg_func_map_["getOutput_double_register_38"] =
      std::bind(&RobotState::getOutput_double_register_38, robot_state_);
  output_reg_func_map_["getOutput_double_register_39"] =
      std::bind(&RobotState::getOutput_double_register_39, robot_state_);
  output_reg_func_map_["getOutput_double_register_40"] =
      std::bind(&RobotState::getOutput_double_register_40, robot_state_);
  output_reg_func_map_["getOutput_double_register_41"] =
      std::bind(&RobotState::getOutput_double_register_41, robot_state_);
  output_reg_func_map_["getOutput_double_register_42"] =
      std::bind(&RobotState::getOutput_double_register_42, robot_state_);
  output_reg_func_map_["getOutput_double_register_43"] =
      std::bind(&RobotState::getOutput_double_register_43, robot_state_);
  output_reg_func_map_["getOutput_double_register_44"] =
      std::bind(&RobotState::getOutput_double_register_44, robot_state_);
  output_reg_func_map_["getOutput_double_register_45"] =
      std::bind(&RobotState::getOutput_double_register_45, robot_state_);
  output_reg_func_map_["getOutput_double_register_46"] =
      std::bind(&RobotState::getOutput_double_register_46, robot_state_);
  output_reg_func_map_["getOutput_double_register_47"] =
      std::bind(&RobotState::getOutput_double_register_47, robot_state_);
}

void RTDEReceiveInterface::receiveCallback()
{
  while (!stop_thread)
  {
    // Receive and update the robot state
    try
    {
      rtde_->receiveData(robot_state_);
      // temporary hack to fix synchronization problems on windows.
#ifndef _WIN32
      std::this_thread::sleep_for(std::chrono::microseconds(100));
#endif
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      if (rtde_->isConnected())
        rtde_->disconnect();
      stop_thread = true;
    }
  }
}

bool RTDEReceiveInterface::reconnect()
{
  if (rtde_ != nullptr)
  {
    rtde_->connect();
    rtde_->negotiateProtocolVersion();
    auto controller_version = rtde_->getControllerVersion();
    uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

    frequency_ = 125;
    // If e-Series Robot set frequency to 500Hz
    if (major_version > CB3_MAJOR_VERSION)
      frequency_ = 500;

    // Set delta time to be used by receiveCallback
    delta_time_ = 1 / frequency_;

    // Init Robot state
    robot_state_ = std::make_shared<RobotState>();

    // Map the output registers to functions
    initOutputRegFuncMap();

    // Setup recipes
    setupRecipes(frequency_);

    // Start RTDE data synchronization
    rtde_->sendStart();

    stop_thread = false;

    // Start executing receiveCallback
    th_ = std::make_shared<boost::thread>(boost::bind(&RTDEReceiveInterface::receiveCallback, this));

    // Wait until the first robot state has been received
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return RTDEReceiveInterface::isConnected();
}

bool RTDEReceiveInterface::isConnected()
{
  return rtde_->isConnected();
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

uint32_t RTDEReceiveInterface::getRobotStatus()
{
  return robot_state_->getRobot_status();
}

std::vector<int32_t> RTDEReceiveInterface::getJointMode()
{
  return robot_state_->getJoint_mode();
}

int32_t RTDEReceiveInterface::getSafetyMode()
{
  return robot_state_->getSafety_mode();
}

uint32_t RTDEReceiveInterface::getSafetyStatusBits()
{
  return robot_state_->getSafety_status_bits();
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

bool RTDEReceiveInterface::getDigitalOutState(std::uint8_t output_id)
{
  uint64_t output_bits = robot_state_->getActual_digital_output_bits();
  std::bitset<std::numeric_limits<uint64_t>::digits> output_bitset(output_bits);
  return output_bitset.test(output_id);
}

uint32_t RTDEReceiveInterface::getRuntimeState()
{
  return robot_state_->getRuntime_state();
}

double RTDEReceiveInterface::getStandardAnalogInput0()
{
  return robot_state_->getStandard_analog_input_0();
}

double RTDEReceiveInterface::getStandardAnalogInput1()
{
  return robot_state_->getStandard_analog_input_1();
}

double RTDEReceiveInterface::getStandardAnalogOutput0()
{
  return robot_state_->getStandard_analog_output_0();
}

double RTDEReceiveInterface::getStandardAnalogOutput1()
{
  return robot_state_->getStandard_analog_output_1();
}

bool RTDEReceiveInterface::isProtectiveStopped()
{
  if (robot_state_ != nullptr)
  {
    std::bitset<32> safety_status_bits(robot_state_->getSafety_status_bits());
    return safety_status_bits.test(SafetyStatus::IS_PROTECTIVE_STOPPED);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTDEReceiveInterface::isEmergencyStopped()
{
  if (robot_state_ != nullptr)
  {
    std::bitset<32> safety_status_bits(robot_state_->getSafety_status_bits());
    return safety_status_bits.test(SafetyStatus::IS_EMERGENCY_STOPPED);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

int RTDEReceiveInterface::getOutputIntRegister(int output_id)
{
  if (use_upper_range_registers_)
  {
    if (!isWithinBounds(output_id, 42, 46))
    {
      throw std::range_error(
          "The supported range of getOutputIntRegister() is [42-46], when using upper range, you specified: " +
          std::to_string(output_id));
    }
  }
  else
  {
    if (!isWithinBounds(output_id, 18, 22))
    {
      throw std::range_error(
          "The supported range of getOutputDoubleRegister() is [18-22], when using lower range you specified: " +
          std::to_string(output_id));
    }
  }

  return getOutputIntReg(output_id);
}

double RTDEReceiveInterface::getOutputDoubleRegister(int output_id)
{
  if (use_upper_range_registers_)
  {
    if (!isWithinBounds(output_id, 36, 43))
    {
      throw std::range_error(
          "The supported range of getOutputIntRegister() is [36-43], when using upper range, you specified: " +
          std::to_string(output_id));
    }
  }
  else
  {
    if (!isWithinBounds(output_id, 12, 19))
    {
      throw std::range_error(
          "The supported range of getOutputDoubleRegister() is [12-19], when using lower range you specified: " +
          std::to_string(output_id));
    }
  }

  return getOutputDoubleReg(output_id);
}

int RTDEReceiveInterface::getAsyncOperationProgress()
{
  return getOutputIntReg(2+register_offset_);
}

double RTDEReceiveInterface::getSpeedScalingCombined()
{
  int runtime_state = getRuntimeState();

  if (runtime_state == RuntimeState::PAUSED)
  {
    pausing_state_ = PausingState::PAUSED;
  }
  else if (runtime_state == RuntimeState::PLAYING && pausing_state_ == PausingState::PAUSED)
  {
    speed_scaling_combined_ = 0.0;
    pausing_state_ = PausingState::RAMPUP;
  }

  if (pausing_state_ == PausingState::RAMPUP)
  {
    double speed_scaling_ramp = speed_scaling_combined_ + pausing_ramp_up_increment_;
    speed_scaling_combined_ = std::min(speed_scaling_ramp, getSpeedScaling() * getTargetSpeedFraction());

    if (speed_scaling_ramp > getSpeedScaling() * getTargetSpeedFraction())
    {
      pausing_state_ = PausingState::RUNNING;
    }
  }
  else if (runtime_state == RuntimeState::RESUMING)
  {
    speed_scaling_combined_ = 0.0;
  }
  else
  {
    speed_scaling_combined_ = getSpeedScaling() * getTargetSpeedFraction();
  }

  return speed_scaling_combined_;
}

}  // namespace ur_rtde
