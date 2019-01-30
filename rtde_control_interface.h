#ifndef RTDE_RTDE_CONTROL_INTERFACE_H
#define RTDE_RTDE_CONTROL_INTERFACE_H

#include "rtde.h"
#include "dashboard_client.h"
#include "script_client.h"
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <sstream>
#include <boost/thread.hpp>

#define MAJOR_VERSION 0
#define CB3_MAJOR_VERSION 3
#define UR_VELOCITY_MAX 1.0
#define UR_VELOCITY_ABSOLUTE_MAX 3.14
#define UR_VELOCITY_MIN 0
#define UR_ACCELERATION_MAX 2.0
#define UR_ACCELERATION_ABSOLUTE_MAX 5.0
#define UR_ACCELERATION_MIN 0
#define UR_BLEND_MAX 2.0
#define UR_BLEND_MIN 0.0

class RTDEControlInterface
{
 public:

  explicit RTDEControlInterface(std::string hostname, int port = 30004);

  virtual ~RTDEControlInterface();

  void stopRobot();

  void moveJ(const std::vector<double>& q, double speed, double acceleration);

  void moveJ(const std::vector<std::vector<double>>& path);

  void moveJ_IK(const std::vector<double>& pose, double speed, double acceleration);

  void moveL(const std::vector<double>& pose, double speed, double acceleration);

  void moveL(const std::vector<std::vector<double>>& path);

  void moveL_FK(const std::vector<double>& q, double speed, double acceleration);

  void moveC(const std::vector<double>& pose_via, const std::vector<double>& pose_to, double speed, double acceleration,
             int mode);

  void speedJ(const std::vector<double>& qd, double acceleration, double time);

  void speedL(const std::vector<double>& xd, double acceleration, double time);

  void servoJ(const std::vector<double>& q, double speed, double acceleration, double time, double lookahead_time,
              double gain);

  void servoC(const std::vector<double>& pose, double speed, double acceleration, double blend);

  void forceModeStart(const std::vector<double>& task_frame, const std::vector<int>& selection_vector,
                      const std::vector<double>& wrench, int type, const std::vector<double>& limits);

  void forceModeUpdate(const std::vector<double>& wrench);

  void forceModeStop();

  void zeroFtSensor();

 private:
  void sendCommand();

  void verifyValueIsWithin(const double& value, const double& min, const double& max);

  std::string prepareCmdScript(const std::vector<std::vector<double>>& path, const std::string& cmd);

  RTDE::RobotCommand popCommand()
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);
    while (command_queue_.empty())
    {
      cond_.wait(mlock);
    }
    auto item = command_queue_.front();
    command_queue_.pop();
    return item;
  }

  void popCommand(RTDE::RobotCommand& item)
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);
    while (command_queue_.empty())
    {
      cond_.wait(mlock);
    }
    item = command_queue_.front();
    command_queue_.pop();
  }

  void pushCommand(const RTDE::RobotCommand& item)
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);
    command_queue_.push(item);
    mlock.unlock();
    cond_.notify_one();
  }

  void pushCommand(RTDE::RobotCommand&& item)
  {
    std::unique_lock<std::mutex> mlock(queue_mutex_);
    command_queue_.push(std::move(item));
    mlock.unlock();
    cond_.notify_one();
  }

 private:
  std::string hostname_;
  int port_;
  std::shared_ptr<RTDE> rtde_;
  std::shared_ptr<DashboardClient> db_client_;
  std::shared_ptr<ScriptClient> script_client_;
  std::shared_ptr<RobotState> robot_state_;
  std::queue<RTDE::RobotCommand> command_queue_;
  std::mutex queue_mutex_;
  std::condition_variable cond_;
};

#endif  // RTDE_RTDE_CONTROL_INTERFACE_H
