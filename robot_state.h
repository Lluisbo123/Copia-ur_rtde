#ifndef RTDE_ROBOT_STATE_H
#define RTDE_ROBOT_STATE_H

#include <vector>
#include <cstdint>

class RobotState
{
 public:
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
};

#endif //RTDE_ROBOT_STATE_H
