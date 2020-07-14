#include <ur_rtde/rtde_control_interface.h>
#include <thread>
#include <chrono>
#include <iostream>

using namespace ur_rtde;
using namespace std::chrono; 

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("127.0.0.1");

  std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
  std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
  std::vector<double> wrench_down = {0, 0, -10, 0, 0, 0};
  std::vector<double> wrench_up = {0, 0, 10, 0, 0, 0};
  int force_type = 2;
  std::vector<double> limits = {2, 2, 1.5, 1, 1, 1};

  rtde_control.forceModeStart(task_frame, selection_vector, wrench_down, force_type, limits);
  std::cout << std::endl << "Going Down!" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << std::endl << "Going Up!" << std::endl << std::endl;
  rtde_control.forceModeUpdate(wrench_up);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  rtde_control.forceModeStop();
  rtde_control.stopScript();

  return 0;
}
