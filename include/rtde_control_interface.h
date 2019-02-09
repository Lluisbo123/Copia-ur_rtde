#ifndef RTDE_RTDE_CONTROL_INTERFACE_H
#define RTDE_RTDE_CONTROL_INTERFACE_H

#include <rtde_export.h>
#include <rtde.h>
#include <dashboard_client.h>
#include <script_client.h>
#include <thread>
#include <sstream>

#define MAJOR_VERSION 0
#define CB3_MAJOR_VERSION 3
#define UR_CONTROLLER_READY 2
#define UR_CONTROLLER_EXECUTING 0

#define UR_VELOCITY_MAX 1.0
#define UR_VELOCITY_ABSOLUTE_MAX 3.14
#define UR_VELOCITY_MIN 0
#define UR_ACCELERATION_MAX 2.0
#define UR_ACCELERATION_ABSOLUTE_MAX 5.0
#define UR_ACCELERATION_MIN 0
#define UR_SERVO_LOOKAHEAD_TIME_MAX 0.2
#define UR_SERVO_LOOKAHEAD_TIME_MIN 0.03
#define UR_SERVO_GAIN_MAX 2000
#define UR_SERVO_GAIN_MIN 100
#define UR_BLEND_MAX 2.0
#define UR_BLEND_MIN 0.0

class RTDE_EXPORT RTDEControlInterface
{
 public:
  explicit RTDEControlInterface(std::string hostname, int port = 30004);

  virtual ~RTDEControlInterface();

  /**
    * @brief This function will stop whatever the robot is doing, and terminate script on controller
    */
  void stopRobot();

  /**
    * @brief Move to joint position (linear in joint-space)
    * @param q joint positions
    * @param speed joint speed of leading axis [rad/s]
    * @param acceleration joint acceleration of leading axis [rad/s^2]
    */
  void moveJ(const std::vector<double>& q, double speed, double acceleration);

  /**
    * @brief Move to each joint position specified in a path
    * @param path with joint positions that includes acceleration, speed and blend for each position
    */
  void moveJ(const std::vector<std::vector<double>>& path);

  /**
    * @brief Move to pose (linear in joint-space)
    * @param pose target pose
    * @param speed joint speed of leading axis [rad/s]
    * @param acceleration joint acceleration of leading axis [rad/s^2]
    */
  void moveJ_IK(const std::vector<double>& pose, double speed, double acceleration);

  /**
    * @brief Move to position (linear in tool-space)
    * @param pose target pose
    * @param speed tool speed [m/s]
    * @param acceleration tool acceleration [m/s^2]
    */
  void moveL(const std::vector<double>& pose, double speed, double acceleration);

  /**
    * @brief Move to each pose specified in a path
    * @param path with tool poses that includes acceleration, speed and blend for each position
    */
  void moveL(const std::vector<std::vector<double>>& path);

  /**
    * @brief Move to position (linear in tool-space)
    * @param q joint positions
    * @param speed tool speed [m/s]
    * @param acceleration tool acceleration [m/s^2]
  */
  void moveL_FK(const std::vector<double>& q, double speed, double acceleration);

  void moveC(const std::vector<double>& pose_via, const std::vector<double>& pose_to, double speed, double acceleration,
             int mode);

  void speedJ(const std::vector<double>& qd, double acceleration, double time = 0.0);

  void speedL(const std::vector<double>& xd, double acceleration, double time = 0.0);

  void servoJ(const std::vector<double>& q, double speed, double acceleration, double time, double lookahead_time,
              double gain);

  void servoC(const std::vector<double>& pose, double speed, double acceleration, double blend);

  void forceModeStart(const std::vector<double>& task_frame, const std::vector<int>& selection_vector,
                      const std::vector<double>& wrench, int type, const std::vector<double>& limits);

  void forceModeUpdate(const std::vector<double>& wrench);

  void forceModeStop();

  void zeroFtSensor();

  void setStandardDigitalOut(std::uint8_t output_id, bool signal_level);

  void setToolDigitalOut(std::uint8_t output_id, bool signal_level);

 private:
  void sendCommand(const RTDE::RobotCommand& cmd);

  void sendClearCommand();

  int getControlScriptState();

  void verifyValueIsWithin(const double& value, const double& min, const double& max);

  std::string prepareCmdScript(const std::vector<std::vector<double>>& path, const std::string& cmd);

 private:
  std::string hostname_;
  int port_;
  std::shared_ptr<RTDE> rtde_;
  std::shared_ptr<DashboardClient> db_client_;
  std::shared_ptr<ScriptClient> script_client_;
  std::shared_ptr<RobotState> robot_state_;
};

#endif  // RTDE_RTDE_CONTROL_INTERFACE_H
