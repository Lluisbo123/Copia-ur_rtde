#include <ur_rtde/robot_state.h>

namespace ur_rtde
{

RobotState::RobotState()
{
}

RobotState::~RobotState()
{
}

bool RobotState::lockUpdateStateMutex()
{
  update_state_mutex_.lock();
  return true;
}

bool RobotState::unlockUpdateStateMutex()
{
  update_state_mutex_.unlock();
  return true;
}

double RobotState::getTimestamp()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return timestamp_;
}
void RobotState::setTimestamp(double timestamp)
{
  RobotState::timestamp_ = timestamp;
}
const std::vector<double> RobotState::getTarget_q()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return target_q_;
}
void RobotState::setTarget_q(const std::vector<double> &target_q)
{
  RobotState::target_q_ = target_q;
}
const std::vector<double> RobotState::getTarget_qd()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return target_qd_;
}
void RobotState::setTarget_qd(const std::vector<double> &target_qd)
{
  RobotState::target_qd_ = target_qd;
}
const std::vector<double> RobotState::getTarget_qdd()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return target_qdd_;
}
void RobotState::setTarget_qdd(const std::vector<double> &target_qdd)
{
  RobotState::target_qdd_ = target_qdd;
}
const std::vector<double> RobotState::getTarget_current()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return target_current_;
}
void RobotState::setTarget_current(const std::vector<double> &target_current)
{
  RobotState::target_current_ = target_current;
}
const std::vector<double> RobotState::getTarget_moment()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return target_moment_;
}
void RobotState::setTarget_moment(const std::vector<double> &target_moment)
{
  RobotState::target_moment_ = target_moment;
}
const std::vector<double> RobotState::getActual_q()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_q_;
}
void RobotState::setActual_q(const std::vector<double> &actual_q)
{
  RobotState::actual_q_ = actual_q;
}
const std::vector<double> RobotState::getActual_qd()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_qd_;
}
void RobotState::setActual_qd(const std::vector<double> &actual_qd)
{
  RobotState::actual_qd_ = actual_qd;
}
const std::vector<double> RobotState::getActual_current()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_current_;
}
void RobotState::setActual_current(const std::vector<double> &actual_current)
{
  RobotState::actual_current_ = actual_current;
}
const std::vector<double> RobotState::getJoint_control_output()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return joint_control_output_;
}
void RobotState::setJoint_control_output(const std::vector<double> &joint_control_output)
{
  RobotState::joint_control_output_ = joint_control_output;
}
const std::vector<double> RobotState::getActual_TCP_pose()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_TCP_pose_;
}
void RobotState::setActual_TCP_pose(const std::vector<double> &actual_TCP_pose)
{
  RobotState::actual_TCP_pose_ = actual_TCP_pose;
}
const std::vector<double> RobotState::getActual_TCP_speed()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_TCP_speed_;
}
void RobotState::setActual_TCP_speed(const std::vector<double> &actual_TCP_speed)
{
  RobotState::actual_TCP_speed_ = actual_TCP_speed;
}
const std::vector<double> RobotState::getActual_TCP_force()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_TCP_force_;
}
void RobotState::setActual_TCP_force(const std::vector<double> &actual_TCP_force)
{
  RobotState::actual_TCP_force_ = actual_TCP_force;
}
const std::vector<double> RobotState::getTarget_TCP_pose()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return target_TCP_pose_;
}
void RobotState::setTarget_TCP_pose(const std::vector<double> &target_TCP_pose)
{
  RobotState::target_TCP_pose_ = target_TCP_pose;
}
const std::vector<double> RobotState::getTarget_TCP_speed()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return target_TCP_speed_;
}
void RobotState::setTarget_TCP_speed(const std::vector<double> &target_TCP_speed)
{
  RobotState::target_TCP_speed_ = target_TCP_speed;
}
uint64_t RobotState::getActual_digital_input_bits()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_digital_input_bits_;
}
void RobotState::setActual_digital_input_bits(uint64_t actual_digital_input_bits)
{
  RobotState::actual_digital_input_bits_ = actual_digital_input_bits;
}
const std::vector<double> RobotState::getJoint_temperatures()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return joint_temperatures_;
}
void RobotState::setJoint_temperatures(const std::vector<double> &joint_temperatures)
{
  RobotState::joint_temperatures_ = joint_temperatures;
}
double RobotState::getActual_execution_time()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_execution_time_;
}
void RobotState::setActual_execution_time(double actual_execution_time)
{
  RobotState::actual_execution_time_ = actual_execution_time;
}
int32_t RobotState::getRobot_mode()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return robot_mode_;
}
void RobotState::setRobot_mode(int32_t robot_mode)
{
  RobotState::robot_mode_ = robot_mode;
}
uint32_t RobotState::getRobot_status()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return robot_status_;
}
void RobotState::setRobot_status(uint32_t robot_status)
{
  RobotState::robot_status_ = robot_status;
}
uint32_t RobotState::getSafety_status_bits()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return safety_status_bits_;
}
void RobotState::setSafety_status_bits(uint32_t safety_status_bits)
{
  RobotState::safety_status_bits_ = safety_status_bits;
}
const std::vector<int32_t> RobotState::getJoint_mode()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return joint_mode_;
}
void RobotState::setJoint_mode(const std::vector<int32_t> &joint_mode)
{
  RobotState::joint_mode_ = joint_mode;
}
int32_t RobotState::getSafety_mode()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return safety_mode_;
}
void RobotState::setSafety_mode(int32_t safety_mode)
{
  RobotState::safety_mode_ = safety_mode;
}
const std::vector<double> RobotState::getActual_tool_accelerometer()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_tool_accelerometer_;
}
void RobotState::setActual_tool_accelerometer(const std::vector<double> &actual_tool_accelerometer)
{
  RobotState::actual_tool_accelerometer_ = actual_tool_accelerometer;
}
double RobotState::getSpeed_scaling()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return speed_scaling_;
}
void RobotState::setSpeed_scaling(double speed_scaling)
{
  RobotState::speed_scaling_ = speed_scaling;
}
double RobotState::getTarget_speed_fraction()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return target_speed_fraction_;
}
void RobotState::setTarget_speed_fraction(double target_speed_fraction)
{
  RobotState::target_speed_fraction_ = target_speed_fraction;
}
double RobotState::getActual_momentum()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_momentum_;
}
void RobotState::setActual_momentum(double actual_momentum)
{
  RobotState::actual_momentum_ = actual_momentum;
}
double RobotState::getActual_main_voltage()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_main_voltage_;
}
void RobotState::setActual_main_voltage(double actual_main_voltage)
{
  RobotState::actual_main_voltage_ = actual_main_voltage;
}
double RobotState::getActual_robot_voltage()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_robot_voltage_;
}
void RobotState::setActual_robot_voltage(double actual_robot_voltage)
{
  RobotState::actual_robot_voltage_ = actual_robot_voltage;
}
double RobotState::getActual_robot_current()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_robot_current_;
}
void RobotState::setActual_robot_current(double actual_robot_current)
{
  RobotState::actual_robot_current_ = actual_robot_current;
}
const std::vector<double> RobotState::getActual_joint_voltage()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_joint_voltage_;
}
void RobotState::setActual_joint_voltage(const std::vector<double> &actual_joint_voltage)
{
  RobotState::actual_joint_voltage_ = actual_joint_voltage;
}
uint64_t RobotState::getActual_digital_output_bits()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return actual_digital_output_bits_;
}
void RobotState::setActual_digital_output_bits(uint64_t actual_digital_output_bits)
{
  RobotState::actual_digital_output_bits_ = actual_digital_output_bits;
}
uint32_t RobotState::getRuntime_state()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return runtime_state_;
}
double RobotState::getStandard_analog_input_0()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return standard_analog_input_0_;
}
void RobotState::setStandard_analog_input_0(double standard_analog_input_0)
{
  RobotState::standard_analog_input_0_ = standard_analog_input_0;
}
double RobotState::getStandard_analog_input_1()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return standard_analog_input_1_;
}
void RobotState::setStandard_analog_input_1(double standard_analog_input_1)
{
  RobotState::standard_analog_input_1_ = standard_analog_input_1;
}
double RobotState::getStandard_analog_output_0()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return standard_analog_output_0_;
}
void RobotState::setStandard_analog_output_0(double standard_analog_output_0)
{
  RobotState::standard_analog_output_0_ = standard_analog_output_0;
}
double RobotState::getStandard_analog_output_1()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return standard_analog_output_1_;
}
void RobotState::setStandard_analog_output_1(double standard_analog_output_1)
{
  RobotState::standard_analog_output_1_ = standard_analog_output_1;
}
void RobotState::setRuntime_state(uint32_t runtime_state)
{
  RobotState::runtime_state_ = runtime_state;
}
uint32_t RobotState::getOutput_bit_registers0_to_31()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_bit_registers0_to_31_;
}
void RobotState::setOutput_bit_registers0_to_31(uint32_t output_bit_registers0_to_31)
{
  RobotState::output_bit_registers0_to_31_ = output_bit_registers0_to_31;
}
uint32_t RobotState::getOutput_bit_registers32_to_63()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_bit_registers32_to_63_;
}
void RobotState::setOutput_bit_registers32_to_63(uint32_t output_bit_registers32_to_63)
{
  RobotState::output_bit_registers32_to_63_ = output_bit_registers32_to_63;
}
int32_t RobotState::getOutput_int_register_0()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_0_;
}
void RobotState::setOutput_int_register_0(int32_t output_int_register_0)
{
  RobotState::output_int_register_0_ = output_int_register_0;
}
int32_t RobotState::getOutput_int_register_1()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_1_;
}
void RobotState::setOutput_int_register_1(int32_t output_int_register_1)
{
  RobotState::output_int_register_1_ = output_int_register_1;
}
int32_t RobotState::getOutput_int_register_2()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_2_;
}
void RobotState::setOutput_int_register_2(int32_t output_int_register_2)
{
  RobotState::output_int_register_2_ = output_int_register_2;
}
int32_t RobotState::getOutput_int_register_3()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_3_;
}
void RobotState::setOutput_int_register_3(int32_t output_int_register_3)
{
  RobotState::output_int_register_3_ = output_int_register_3;
}
int32_t RobotState::getOutput_int_register_4()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_4_;
}
void RobotState::setOutput_int_register_4(int32_t output_int_register_4)
{
  RobotState::output_int_register_4_ = output_int_register_4;
}
int32_t RobotState::getOutput_int_register_5()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_5_;
}
void RobotState::setOutput_int_register_5(int32_t output_int_register_5)
{
  RobotState::output_int_register_5_ = output_int_register_5;
}
int32_t RobotState::getOutput_int_register_6()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_6_;
}
void RobotState::setOutput_int_register_6(int32_t output_int_register_6)
{
  RobotState::output_int_register_6_ = output_int_register_6;
}
int32_t RobotState::getOutput_int_register_7()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_7_;
}
void RobotState::setOutput_int_register_7(int32_t output_int_register_7)
{
  RobotState::output_int_register_7_ = output_int_register_7;
}
int32_t RobotState::getOutput_int_register_8()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_8_;
}
void RobotState::setOutput_int_register_8(int32_t output_int_register_8)
{
  RobotState::output_int_register_8_ = output_int_register_8;
}
int32_t RobotState::getOutput_int_register_9()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_9_;
}
void RobotState::setOutput_int_register_9(int32_t output_int_register_9)
{
  RobotState::output_int_register_9_ = output_int_register_9;
}
int32_t RobotState::getOutput_int_register_10()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_10_;
}
void RobotState::setOutput_int_register_10(int32_t output_int_register_10)
{
  RobotState::output_int_register_10_ = output_int_register_10;
}
int32_t RobotState::getOutput_int_register_11()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_11_;
}
void RobotState::setOutput_int_register_11(int32_t output_int_register_11)
{
  RobotState::output_int_register_11_ = output_int_register_11;
}
int32_t RobotState::getOutput_int_register_12()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_12_;
}
void RobotState::setOutput_int_register_12(int32_t output_int_register_12)
{
  RobotState::output_int_register_12_ = output_int_register_12;
}
int32_t RobotState::getOutput_int_register_13()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_13_;
}
void RobotState::setOutput_int_register_13(int32_t output_int_register_13)
{
  RobotState::output_int_register_13_ = output_int_register_13;
}
int32_t RobotState::getOutput_int_register_14()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_14_;
}
void RobotState::setOutput_int_register_14(int32_t output_int_register_14)
{
  RobotState::output_int_register_14_ = output_int_register_14;
}
int32_t RobotState::getOutput_int_register_15()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_15_;
}
void RobotState::setOutput_int_register_15(int32_t output_int_register_15)
{
  RobotState::output_int_register_15_ = output_int_register_15;
}
int32_t RobotState::getOutput_int_register_16()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_16_;
}
void RobotState::setOutput_int_register_16(int32_t output_int_register_16)
{
  RobotState::output_int_register_16_ = output_int_register_16;
}
int32_t RobotState::getOutput_int_register_17()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_17_;
}
void RobotState::setOutput_int_register_17(int32_t output_int_register_17)
{
  RobotState::output_int_register_17_ = output_int_register_17;
}
int32_t RobotState::getOutput_int_register_18()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_18_;
}
void RobotState::setOutput_int_register_18(int32_t output_int_register_18)
{
  RobotState::output_int_register_18_ = output_int_register_18;
}
int32_t RobotState::getOutput_int_register_19()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_19_;
}
void RobotState::setOutput_int_register_19(int32_t output_int_register_19)
{
  RobotState::output_int_register_19_ = output_int_register_19;
}
int32_t RobotState::getOutput_int_register_20()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_20_;
}
void RobotState::setOutput_int_register_20(int32_t output_int_register_20)
{
  RobotState::output_int_register_20_ = output_int_register_20;
}
int32_t RobotState::getOutput_int_register_21()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_21_;
}
void RobotState::setOutput_int_register_21(int32_t output_int_register_21)
{
  RobotState::output_int_register_21_ = output_int_register_21;
}
int32_t RobotState::getOutput_int_register_22()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_22_;
}
void RobotState::setOutput_int_register_22(int32_t output_int_register_22)
{
  RobotState::output_int_register_22_ = output_int_register_22;
}
int32_t RobotState::getOutput_int_register_23()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_23_;
}
void RobotState::setOutput_int_register_23(int32_t output_int_register_23)
{
  RobotState::output_int_register_23_ = output_int_register_23;
}
int32_t RobotState::getOutput_int_register_24()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_24_;
}
void RobotState::setOutput_int_register_24(int32_t output_int_register_24)
{
  RobotState::output_int_register_24_ = output_int_register_24;
}
int32_t RobotState::getOutput_int_register_25()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_25_;
}
void RobotState::setOutput_int_register_25(int32_t output_int_register_25)
{
  RobotState::output_int_register_25_ = output_int_register_25;
}
int32_t RobotState::getOutput_int_register_26()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_26_;
}
void RobotState::setOutput_int_register_26(int32_t output_int_register_26)
{
  RobotState::output_int_register_26_ = output_int_register_26;
}
int32_t RobotState::getOutput_int_register_27()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_27_;
}
void RobotState::setOutput_int_register_27(int32_t output_int_register_27)
{
  RobotState::output_int_register_27_ = output_int_register_27;
}
int32_t RobotState::getOutput_int_register_28()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_28_;
}
void RobotState::setOutput_int_register_28(int32_t output_int_register_28)
{
  RobotState::output_int_register_28_ = output_int_register_28;
}
int32_t RobotState::getOutput_int_register_29()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_29_;
}
void RobotState::setOutput_int_register_29(int32_t output_int_register_29)
{
  RobotState::output_int_register_29_ = output_int_register_29;
}
int32_t RobotState::getOutput_int_register_30()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_30_;
}
void RobotState::setOutput_int_register_30(int32_t output_int_register_30)
{
  RobotState::output_int_register_30_ = output_int_register_30;
}
int32_t RobotState::getOutput_int_register_31()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_31_;
}
void RobotState::setOutput_int_register_31(int32_t output_int_register_31)
{
  RobotState::output_int_register_31_ = output_int_register_31;
}
int32_t RobotState::getOutput_int_register_32()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_32_;
}
void RobotState::setOutput_int_register_32(int32_t output_int_register_32)
{
  RobotState::output_int_register_32_ = output_int_register_32;
}
int32_t RobotState::getOutput_int_register_33()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_33_;
}
void RobotState::setOutput_int_register_33(int32_t output_int_register_33)
{
  RobotState::output_int_register_33_ = output_int_register_33;
}
int32_t RobotState::getOutput_int_register_34()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_34_;
}
void RobotState::setOutput_int_register_34(int32_t output_int_register_34)
{
  RobotState::output_int_register_34_ = output_int_register_34;
}
int32_t RobotState::getOutput_int_register_35()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_35_;
}
void RobotState::setOutput_int_register_35(int32_t output_int_register_35)
{
  RobotState::output_int_register_35_ = output_int_register_35;
}
int32_t RobotState::getOutput_int_register_36()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_36_;
}
void RobotState::setOutput_int_register_36(int32_t output_int_register_36)
{
  RobotState::output_int_register_36_ = output_int_register_36;
}
int32_t RobotState::getOutput_int_register_37()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_37_;
}
void RobotState::setOutput_int_register_37(int32_t output_int_register_37)
{
  RobotState::output_int_register_37_ = output_int_register_37;
}
int32_t RobotState::getOutput_int_register_38()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_38_;
}
void RobotState::setOutput_int_register_38(int32_t output_int_register_38)
{
  RobotState::output_int_register_38_ = output_int_register_38;
}
int32_t RobotState::getOutput_int_register_39()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_39_;
}
void RobotState::setOutput_int_register_39(int32_t output_int_register_39)
{
  RobotState::output_int_register_39_ = output_int_register_39;
}
int32_t RobotState::getOutput_int_register_40()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_40_;
}
void RobotState::setOutput_int_register_40(int32_t output_int_register_40)
{
  RobotState::output_int_register_40_ = output_int_register_40;
}
int32_t RobotState::getOutput_int_register_41()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_41_;
}
void RobotState::setOutput_int_register_41(int32_t output_int_register_41)
{
  RobotState::output_int_register_41_ = output_int_register_41;
}
int32_t RobotState::getOutput_int_register_42()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_42_;
}
void RobotState::setOutput_int_register_42(int32_t output_int_register_42)
{
  RobotState::output_int_register_42_ = output_int_register_42;
}
int32_t RobotState::getOutput_int_register_43()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_43_;
}
void RobotState::setOutput_int_register_43(int32_t output_int_register_43)
{
  RobotState::output_int_register_43_ = output_int_register_43;
}
int32_t RobotState::getOutput_int_register_44()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_44_;
}
void RobotState::setOutput_int_register_44(int32_t output_int_register_44)
{
  RobotState::output_int_register_44_ = output_int_register_44;
}
int32_t RobotState::getOutput_int_register_45()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_45_;
}
void RobotState::setOutput_int_register_45(int32_t output_int_register_45)
{
  RobotState::output_int_register_45_ = output_int_register_45;
}
int32_t RobotState::getOutput_int_register_46()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_46_;
}
void RobotState::setOutput_int_register_46(int32_t output_int_register_46)
{
  RobotState::output_int_register_46_ = output_int_register_46;
}
int32_t RobotState::getOutput_int_register_47()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_int_register_47_;
}
void RobotState::setOutput_int_register_47(int32_t output_int_register_47)
{
  RobotState::output_int_register_47_ = output_int_register_47;
}
double RobotState::getOutput_double_register_0()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_0_;
}
void RobotState::setOutput_double_register_0(double output_double_register_0)
{
  RobotState::output_double_register_0_ = output_double_register_0;
}
double RobotState::getOutput_double_register_1()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_1_;
}
void RobotState::setOutput_double_register_1(double output_double_register_1)
{
  RobotState::output_double_register_1_ = output_double_register_1;
}
double RobotState::getOutput_double_register_2()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_2_;
}
void RobotState::setOutput_double_register_2(double output_double_register_2)
{
  RobotState::output_double_register_2_ = output_double_register_2;
}
double RobotState::getOutput_double_register_3()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_3_;
}
void RobotState::setOutput_double_register_3(double output_double_register_3)
{
  RobotState::output_double_register_3_ = output_double_register_3;
}
double RobotState::getOutput_double_register_4()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_4_;
}
void RobotState::setOutput_double_register_4(double output_double_register_4)
{
  RobotState::output_double_register_4_ = output_double_register_4;
}
double RobotState::getOutput_double_register_5()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_5_;
}
void RobotState::setOutput_double_register_5(double output_double_register_5)
{
  RobotState::output_double_register_5_ = output_double_register_5;
}
double RobotState::getOutput_double_register_6()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_6_;
}
void RobotState::setOutput_double_register_6(double output_double_register_6)
{
  RobotState::output_double_register_6_ = output_double_register_6;
}
double RobotState::getOutput_double_register_7()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_7_;
}
void RobotState::setOutput_double_register_7(double output_double_register_7)
{
  RobotState::output_double_register_7_ = output_double_register_7;
}
double RobotState::getOutput_double_register_8()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_8_;
}
void RobotState::setOutput_double_register_8(double output_double_register_8)
{
  RobotState::output_double_register_8_ = output_double_register_8;
}
double RobotState::getOutput_double_register_9()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_9_;
}
void RobotState::setOutput_double_register_9(double output_double_register_9)
{
  RobotState::output_double_register_9_ = output_double_register_9;
}
double RobotState::getOutput_double_register_10()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_10_;
}
void RobotState::setOutput_double_register_10(double output_double_register_10)
{
  RobotState::output_double_register_10_ = output_double_register_10;
}
double RobotState::getOutput_double_register_11()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_11_;
}
void RobotState::setOutput_double_register_11(double output_double_register_11)
{
  RobotState::output_double_register_11_ = output_double_register_11;
}
double RobotState::getOutput_double_register_12()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_12_;
}
void RobotState::setOutput_double_register_12(double output_double_register_12)
{
  RobotState::output_double_register_12_ = output_double_register_12;
}
double RobotState::getOutput_double_register_13()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_13_;
}
void RobotState::setOutput_double_register_13(double output_double_register_13)
{
  RobotState::output_double_register_13_ = output_double_register_13;
}
double RobotState::getOutput_double_register_14()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_14_;
}
void RobotState::setOutput_double_register_14(double output_double_register_14)
{
  RobotState::output_double_register_14_ = output_double_register_14;
}
double RobotState::getOutput_double_register_15()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_15_;
}
void RobotState::setOutput_double_register_15(double output_double_register_15)
{
  RobotState::output_double_register_15_ = output_double_register_15;
}
double RobotState::getOutput_double_register_16()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_16_;
}
void RobotState::setOutput_double_register_16(double output_double_register_16)
{
  RobotState::output_double_register_16_ = output_double_register_16;
}
double RobotState::getOutput_double_register_17()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_17_;
}
void RobotState::setOutput_double_register_17(double output_double_register_17)
{
  RobotState::output_double_register_17_ = output_double_register_17;
}
double RobotState::getOutput_double_register_18()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_18_;
}
void RobotState::setOutput_double_register_18(double output_double_register_18)
{
  RobotState::output_double_register_18_ = output_double_register_18;
}
double RobotState::getOutput_double_register_19()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_19_;
}
void RobotState::setOutput_double_register_19(double output_double_register_19)
{
  RobotState::output_double_register_19_ = output_double_register_19;
}
double RobotState::getOutput_double_register_20()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_20_;
}
void RobotState::setOutput_double_register_20(double output_double_register_20)
{
  RobotState::output_double_register_20_ = output_double_register_20;
}
double RobotState::getOutput_double_register_21()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_21_;
}
void RobotState::setOutput_double_register_21(double output_double_register_21)
{
  RobotState::output_double_register_21_ = output_double_register_21;
}
double RobotState::getOutput_double_register_22()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_22_;
}
void RobotState::setOutput_double_register_22(double output_double_register_22)
{
  RobotState::output_double_register_22_ = output_double_register_22;
}
double RobotState::getOutput_double_register_23()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_23_;
}
void RobotState::setOutput_double_register_23(double output_double_register_23)
{
  RobotState::output_double_register_23_ = output_double_register_23;
}
double RobotState::getOutput_double_register_24()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_24_;
}
void RobotState::setOutput_double_register_24(double output_double_register_24)
{
  RobotState::output_double_register_24_ = output_double_register_24;
}
double RobotState::getOutput_double_register_25()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_25_;
}
void RobotState::setOutput_double_register_25(double output_double_register_25)
{
  RobotState::output_double_register_25_ = output_double_register_25;
}
double RobotState::getOutput_double_register_26()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_26_;
}
void RobotState::setOutput_double_register_26(double output_double_register_26)
{
  RobotState::output_double_register_26_ = output_double_register_26;
}
double RobotState::getOutput_double_register_27()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_27_;
}
void RobotState::setOutput_double_register_27(double output_double_register_27)
{
  RobotState::output_double_register_27_ = output_double_register_27;
}
double RobotState::getOutput_double_register_28()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_28_;
}
void RobotState::setOutput_double_register_28(double output_double_register_28)
{
  RobotState::output_double_register_28_ = output_double_register_28;
}
double RobotState::getOutput_double_register_29()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_29_;
}
void RobotState::setOutput_double_register_29(double output_double_register_29)
{
  RobotState::output_double_register_29_ = output_double_register_29;
}
double RobotState::getOutput_double_register_30()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_30_;
}
void RobotState::setOutput_double_register_30(double output_double_register_30)
{
  RobotState::output_double_register_30_ = output_double_register_30;
}
double RobotState::getOutput_double_register_31()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_31_;
}
void RobotState::setOutput_double_register_31(double output_double_register_31)
{
  RobotState::output_double_register_31_ = output_double_register_31;
}
double RobotState::getOutput_double_register_32()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_32_;
}
void RobotState::setOutput_double_register_32(double output_double_register_32)
{
  RobotState::output_double_register_32_ = output_double_register_32;
}
double RobotState::getOutput_double_register_33()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_33_;
}
void RobotState::setOutput_double_register_33(double output_double_register_33)
{
  RobotState::output_double_register_33_ = output_double_register_33;
}
double RobotState::getOutput_double_register_34()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_34_;
}
void RobotState::setOutput_double_register_34(double output_double_register_34)
{
  RobotState::output_double_register_34_ = output_double_register_34;
}
double RobotState::getOutput_double_register_35()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_35_;
}
void RobotState::setOutput_double_register_35(double output_double_register_35)
{
  RobotState::output_double_register_35_ = output_double_register_35;
}
double RobotState::getOutput_double_register_36()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_36_;
}
void RobotState::setOutput_double_register_36(double output_double_register_36)
{
  RobotState::output_double_register_36_ = output_double_register_36;
}
double RobotState::getOutput_double_register_37()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_37_;
}
void RobotState::setOutput_double_register_37(double output_double_register_37)
{
  RobotState::output_double_register_37_ = output_double_register_37;
}
double RobotState::getOutput_double_register_38()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_38_;
}
void RobotState::setOutput_double_register_38(double output_double_register_38)
{
  RobotState::output_double_register_38_ = output_double_register_38;
}
double RobotState::getOutput_double_register_39()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_39_;
}
void RobotState::setOutput_double_register_39(double output_double_register_39)
{
  RobotState::output_double_register_39_ = output_double_register_39;
}
double RobotState::getOutput_double_register_40()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_40_;
}
void RobotState::setOutput_double_register_40(double output_double_register_40)
{
  RobotState::output_double_register_40_ = output_double_register_40;
}
double RobotState::getOutput_double_register_41()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_41_;
}
void RobotState::setOutput_double_register_41(double output_double_register_41)
{
  RobotState::output_double_register_41_ = output_double_register_41;
}
double RobotState::getOutput_double_register_42()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_42_;
}
void RobotState::setOutput_double_register_42(double output_double_register_42)
{
  RobotState::output_double_register_42_ = output_double_register_42;
}
double RobotState::getOutput_double_register_43()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_43_;
}
void RobotState::setOutput_double_register_43(double output_double_register_43)
{
  RobotState::output_double_register_43_ = output_double_register_43;
}
double RobotState::getOutput_double_register_44()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_44_;
}
void RobotState::setOutput_double_register_44(double output_double_register_44)
{
  RobotState::output_double_register_44_ = output_double_register_44;
}
double RobotState::getOutput_double_register_45()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_45_;
}
void RobotState::setOutput_double_register_45(double output_double_register_45)
{
  RobotState::output_double_register_45_ = output_double_register_45;
}
double RobotState::getOutput_double_register_46()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_46_;
}
void RobotState::setOutput_double_register_46(double output_double_register_46)
{
  RobotState::output_double_register_46_ = output_double_register_46;
}
double RobotState::getOutput_double_register_47()
{
  std::lock_guard<std::mutex> lock(update_state_mutex_);
  return output_double_register_47_;
}
void RobotState::setOutput_double_register_47(double output_double_register_47)
{
  RobotState::output_double_register_47_ = output_double_register_47;
}

}  // namespace ur_rtde