#ifndef RTDE_ROBOT_STATE_H
#define RTDE_ROBOT_STATE_H

#include <rtde_export.h>
#include <vector>
#include <cstdint>
#include <mutex>

class RTDE_EXPORT RobotState
{
 private:
  double timestamp;
  std::vector<double> target_q;
  std::vector<double> target_qd;
  std::vector<double> target_qdd;
  std::vector<double> target_current;
  std::vector<double> target_moment;
  std::vector<double> actual_q;
  std::vector<double> actual_qd;
  std::vector<double> actual_current;
  std::vector<double> joint_control_output;
  std::vector<double> actual_TCP_pose;
  std::vector<double> actual_TCP_speed;
  std::vector<double> actual_TCP_force;
  std::vector<double> target_TCP_pose;
  std::vector<double> target_TCP_speed;
  uint64_t actual_digital_input_bits;
  std::vector<double> joint_temperatures;
  double actual_execution_time;
  int32_t robot_mode;
  std::vector<int32_t> joint_mode;
  int32_t safety_mode;
  std::vector<double> actual_tool_accelerometer;
  double speed_scaling;
  double target_speed_fraction;
  double actual_momentum;
  double actual_main_voltage;
  double actual_robot_voltage;
  double actual_robot_current;
  std::vector<double> actual_joint_voltage;
  uint64_t actual_digital_output_bits;
  uint32_t runtime_state;

  uint32_t output_bit_registers0_to_31;
  uint32_t output_bit_registers32_to_63;

  int32_t output_int_register_0;
  int32_t output_int_register_1;
  int32_t output_int_register_2;
  int32_t output_int_register_3;
  int32_t output_int_register_4;
  int32_t output_int_register_5;
  int32_t output_int_register_6;
  int32_t output_int_register_7;
  int32_t output_int_register_8;
  int32_t output_int_register_9;
  int32_t output_int_register_10;
  int32_t output_int_register_11;
  int32_t output_int_register_12;
  int32_t output_int_register_13;
  int32_t output_int_register_14;
  int32_t output_int_register_15;
  int32_t output_int_register_16;
  int32_t output_int_register_17;
  int32_t output_int_register_18;
  int32_t output_int_register_19;
  int32_t output_int_register_20;
  int32_t output_int_register_21;
  int32_t output_int_register_22;
  int32_t output_int_register_23;

  double output_double_register_0;
  double output_double_register_1;
  double output_double_register_2;
  double output_double_register_3;
  double output_double_register_4;
  double output_double_register_5;
  double output_double_register_6;
  double output_double_register_7;
  double output_double_register_8;
  double output_double_register_9;
  double output_double_register_10;
  double output_double_register_11;
  double output_double_register_12;
  double output_double_register_13;
  double output_double_register_14;
  double output_double_register_15;
  double output_double_register_16;
  double output_double_register_17;
  double output_double_register_18;
  double output_double_register_19;
  double output_double_register_20;
  double output_double_register_21;
  double output_double_register_22;
  double output_double_register_23;

 public:
  double getTimestamp()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return timestamp;
  }
  void setTimestamp(double timestamp)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::timestamp = timestamp;
  }
  const std::vector<double> &getTarget_q()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_q;
  }
  void setTarget_q(const std::vector<double> &target_q)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::target_q = target_q;
  }
  const std::vector<double> &getTarget_qd()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_qd;
  }
  void setTarget_qd(const std::vector<double> &target_qd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::target_qd = target_qd;
  }
  const std::vector<double> &getTarget_qdd()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_qdd;
  }
  void setTarget_qdd(const std::vector<double> &target_qdd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::target_qdd = target_qdd;
  }
  const std::vector<double> &getTarget_current()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_current;
  }
  void setTarget_current(const std::vector<double> &target_current)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::target_current = target_current;
  }
  const std::vector<double> &getTarget_moment()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_moment;
  }
  void setTarget_moment(const std::vector<double> &target_moment)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::target_moment = target_moment;
  }
  const std::vector<double> &getActual_q()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_q;
  }
  void setActual_q(const std::vector<double> &actual_q)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_q = actual_q;
  }
  const std::vector<double> &getActual_qd()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_qd;
  }
  void setActual_qd(const std::vector<double> &actual_qd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_qd = actual_qd;
  }
  const std::vector<double> &getActual_current()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_current;
  }
  void setActual_current(const std::vector<double> &actual_current)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_current = actual_current;
  }
  const std::vector<double> &getJoint_control_output()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return joint_control_output;
  }
  void setJoint_control_output(const std::vector<double> &joint_control_output)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::joint_control_output = joint_control_output;
  }
  const std::vector<double> &getActual_TCP_pose()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_TCP_pose;
  }
  void setActual_TCP_pose(const std::vector<double> &actual_TCP_pose)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_TCP_pose = actual_TCP_pose;
  }
  const std::vector<double> &getActual_TCP_speed()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_TCP_speed;
  }
  void setActual_TCP_speed(const std::vector<double> &actual_TCP_speed)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_TCP_speed = actual_TCP_speed;
  }
  const std::vector<double> &getActual_TCP_force()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_TCP_force;
  }
  void setActual_TCP_force(const std::vector<double> &actual_TCP_force)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_TCP_force = actual_TCP_force;
  }
  const std::vector<double> &getTarget_TCP_pose()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_TCP_pose;
  }
  void setTarget_TCP_pose(const std::vector<double> &target_TCP_pose)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::target_TCP_pose = target_TCP_pose;
  }
  const std::vector<double> &getTarget_TCP_speed()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_TCP_speed;
  }
  void setTarget_TCP_speed(const std::vector<double> &target_TCP_speed)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::target_TCP_speed = target_TCP_speed;
  }
  uint64_t getActual_digital_input_bits()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_digital_input_bits;
  }
  void setActual_digital_input_bits(uint64_t actual_digital_input_bits)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_digital_input_bits = actual_digital_input_bits;
  }
  const std::vector<double> &getJoint_temperatures()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return joint_temperatures;
  }
  void setJoint_temperatures(const std::vector<double> &joint_temperatures)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::joint_temperatures = joint_temperatures;
  }
  double getActual_execution_time()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_execution_time;
  }
  void setActual_execution_time(double actual_execution_time)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_execution_time = actual_execution_time;
  }
  int32_t getRobot_mode()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return robot_mode;
  }
  void setRobot_mode(int32_t robot_mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::robot_mode = robot_mode;
  }
  const std::vector<int32_t> &getJoint_mode()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return joint_mode;
  }
  void setJoint_mode(const std::vector<int32_t> &joint_mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::joint_mode = joint_mode;
  }
  int32_t getSafety_mode()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return safety_mode;
  }
  void setSafety_mode(int32_t safety_mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::safety_mode = safety_mode;
  }
  const std::vector<double> &getActual_tool_accelerometer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_tool_accelerometer;
  }
  void setActual_tool_accelerometer(const std::vector<double> &actual_tool_accelerometer)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_tool_accelerometer = actual_tool_accelerometer;
  }
  double getSpeed_scaling()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return speed_scaling;
  }
  void setSpeed_scaling(double speed_scaling)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::speed_scaling = speed_scaling;
  }
  double getTarget_speed_fraction()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_speed_fraction;
  }
  void setTarget_speed_fraction(double target_speed_fraction)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::target_speed_fraction = target_speed_fraction;
  }
  double getActual_momentum()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_momentum;
  }
  void setActual_momentum(double actual_momentum)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_momentum = actual_momentum;
  }
  double getActual_main_voltage()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_main_voltage;
  }
  void setActual_main_voltage(double actual_main_voltage)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_main_voltage = actual_main_voltage;
  }
  double getActual_robot_voltage()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_robot_voltage;
  }
  void setActual_robot_voltage(double actual_robot_voltage)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_robot_voltage = actual_robot_voltage;
  }
  double getActual_robot_current()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_robot_current;
  }
  void setActual_robot_current(double actual_robot_current)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_robot_current = actual_robot_current;
  }
  const std::vector<double> &getActual_joint_voltage()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_joint_voltage;
  }
  void setActual_joint_voltage(const std::vector<double> &actual_joint_voltage)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_joint_voltage = actual_joint_voltage;
  }
  uint64_t getActual_digital_output_bits()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return actual_digital_output_bits;
  }
  void setActual_digital_output_bits(uint64_t actual_digital_output_bits)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::actual_digital_output_bits = actual_digital_output_bits;
  }
  uint32_t getRuntime_state()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return runtime_state;
  }
  void setRuntime_state(uint32_t runtime_state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::runtime_state = runtime_state;
  }
  uint32_t getOutput_bit_registers0_to_31()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_bit_registers0_to_31;
  }
  void setOutput_bit_registers0_to_31(uint32_t output_bit_registers0_to_31)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_bit_registers0_to_31 = output_bit_registers0_to_31;
  }
  uint32_t getOutput_bit_registers32_to_63()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_bit_registers32_to_63;
  }
  void setOutput_bit_registers32_to_63(uint32_t output_bit_registers32_to_63)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_bit_registers32_to_63 = output_bit_registers32_to_63;
  }
  int32_t getOutput_int_register_0()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_0;
  }
  void setOutput_int_register_0(int32_t output_int_register_0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_0 = output_int_register_0;
  }
  int32_t getOutput_int_register_1()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_1;
  }
  void setOutput_int_register_1(int32_t output_int_register_1)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_1 = output_int_register_1;
  }
  int32_t getOutput_int_register_2()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_2;
  }
  void setOutput_int_register_2(int32_t output_int_register_2)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_2 = output_int_register_2;
  }
  int32_t getOutput_int_register_3()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_3;
  }
  void setOutput_int_register_3(int32_t output_int_register_3)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_3 = output_int_register_3;
  }
  int32_t getOutput_int_register_4()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_4;
  }
  void setOutput_int_register_4(int32_t output_int_register_4)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_4 = output_int_register_4;
  }
  int32_t getOutput_int_register_5()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_5;
  }
  void setOutput_int_register_5(int32_t output_int_register_5)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_5 = output_int_register_5;
  }
  int32_t getOutput_int_register_6()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_6;
  }
  void setOutput_int_register_6(int32_t output_int_register_6)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_6 = output_int_register_6;
  }
  int32_t getOutput_int_register_7()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_7;
  }
  void setOutput_int_register_7(int32_t output_int_register_7)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_7 = output_int_register_7;
  }
  int32_t getOutput_int_register_8()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_8;
  }
  void setOutput_int_register_8(int32_t output_int_register_8)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_8 = output_int_register_8;
  }
  int32_t getOutput_int_register_9()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_9;
  }
  void setOutput_int_register_9(int32_t output_int_register_9)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_9 = output_int_register_9;
  }
  int32_t getOutput_int_register_10()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_10;
  }
  void setOutput_int_register_10(int32_t output_int_register_10)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_10 = output_int_register_10;
  }
  int32_t getOutput_int_register_11()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_11;
  }
  void setOutput_int_register_11(int32_t output_int_register_11)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_11 = output_int_register_11;
  }
  int32_t getOutput_int_register_12()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_12;
  }
  void setOutput_int_register_12(int32_t output_int_register_12)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_12 = output_int_register_12;
  }
  int32_t getOutput_int_register_13()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_13;
  }
  void setOutput_int_register_13(int32_t output_int_register_13)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_13 = output_int_register_13;
  }
  int32_t getOutput_int_register_14()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_14;
  }
  void setOutput_int_register_14(int32_t output_int_register_14)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_14 = output_int_register_14;
  }
  int32_t getOutput_int_register_15()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_15;
  }
  void setOutput_int_register_15(int32_t output_int_register_15)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_15 = output_int_register_15;
  }
  int32_t getOutput_int_register_16()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_16;
  }
  void setOutput_int_register_16(int32_t output_int_register_16)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_16 = output_int_register_16;
  }
  int32_t getOutput_int_register_17()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_17;
  }
  void setOutput_int_register_17(int32_t output_int_register_17)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_17 = output_int_register_17;
  }
  int32_t getOutput_int_register_18()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_18;
  }
  void setOutput_int_register_18(int32_t output_int_register_18)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_18 = output_int_register_18;
  }
  int32_t getOutput_int_register_19()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_19;
  }
  void setOutput_int_register_19(int32_t output_int_register_19)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_19 = output_int_register_19;
  }
  int32_t getOutput_int_register_20()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_20;
  }
  void setOutput_int_register_20(int32_t output_int_register_20)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_20 = output_int_register_20;
  }
  int32_t getOutput_int_register_21()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_21;
  }
  void setOutput_int_register_21(int32_t output_int_register_21)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_21 = output_int_register_21;
  }
  int32_t getOutput_int_register_22()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_22;
  }
  void setOutput_int_register_22(int32_t output_int_register_22)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_22 = output_int_register_22;
  }
  int32_t getOutput_int_register_23()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_int_register_23;
  }
  void setOutput_int_register_23(int32_t output_int_register_23)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_int_register_23 = output_int_register_23;
  }
  double getOutput_double_register_0()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_0;
  }
  void setOutput_double_register_0(double output_double_register_0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_0 = output_double_register_0;
  }
  double getOutput_double_register_1()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_1;
  }
  void setOutput_double_register_1(double output_double_register_1)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_1 = output_double_register_1;
  }
  double getOutput_double_register_2()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_2;
  }
  void setOutput_double_register_2(double output_double_register_2)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_2 = output_double_register_2;
  }
  double getOutput_double_register_3()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_3;
  }
  void setOutput_double_register_3(double output_double_register_3)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_3 = output_double_register_3;
  }
  double getOutput_double_register_4()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_4;
  }
  void setOutput_double_register_4(double output_double_register_4)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_4 = output_double_register_4;
  }
  double getOutput_double_register_5()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_5;
  }
  void setOutput_double_register_5(double output_double_register_5)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_5 = output_double_register_5;
  }
  double getOutput_double_register_6()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_6;
  }
  void setOutput_double_register_6(double output_double_register_6)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_6 = output_double_register_6;
  }
  double getOutput_double_register_7()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_7;
  }
  void setOutput_double_register_7(double output_double_register_7)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_7 = output_double_register_7;
  }
  double getOutput_double_register_8()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_8;
  }
  void setOutput_double_register_8(double output_double_register_8)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_8 = output_double_register_8;
  }
  double getOutput_double_register_9()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_9;
  }
  void setOutput_double_register_9(double output_double_register_9)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_9 = output_double_register_9;
  }
  double getOutput_double_register_10()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_10;
  }
  void setOutput_double_register_10(double output_double_register_10)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_10 = output_double_register_10;
  }
  double getOutput_double_register_11()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_11;
  }
  void setOutput_double_register_11(double output_double_register_11)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_11 = output_double_register_11;
  }
  double getOutput_double_register_12()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_12;
  }
  void setOutput_double_register_12(double output_double_register_12)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_12 = output_double_register_12;
  }
  double getOutput_double_register_13()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_13;
  }
  void setOutput_double_register_13(double output_double_register_13)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_13 = output_double_register_13;
  }
  double getOutput_double_register_14()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_14;
  }
  void setOutput_double_register_14(double output_double_register_14)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_14 = output_double_register_14;
  }
  double getOutput_double_register_15()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_15;
  }
  void setOutput_double_register_15(double output_double_register_15)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_15 = output_double_register_15;
  }
  double getOutput_double_register_16()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_16;
  }
  void setOutput_double_register_16(double output_double_register_16)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_16 = output_double_register_16;
  }
  double getOutput_double_register_17()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_17;
  }
  void setOutput_double_register_17(double output_double_register_17)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_17 = output_double_register_17;
  }
  double getOutput_double_register_18()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_18;
  }
  void setOutput_double_register_18(double output_double_register_18)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_18 = output_double_register_18;
  }
  double getOutput_double_register_19()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_19;
  }
  void setOutput_double_register_19(double output_double_register_19)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_19 = output_double_register_19;
  }
  double getOutput_double_register_20()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_20;
  }
  void setOutput_double_register_20(double output_double_register_20)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_20 = output_double_register_20;
  }
  double getOutput_double_register_21()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_21;
  }
  void setOutput_double_register_21(double output_double_register_21)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_21 = output_double_register_21;
  }
  double getOutput_double_register_22()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_22;
  }
  void setOutput_double_register_22(double output_double_register_22)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_22 = output_double_register_22;
  }
  double getOutput_double_register_23()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_double_register_23;
  }
  void setOutput_double_register_23(double output_double_register_23)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState::output_double_register_23 = output_double_register_23;
  }

 private:
  std::mutex mutex_;
};

#endif  // RTDE_ROBOT_STATE_H
