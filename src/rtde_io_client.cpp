#include <ur_rtde/rtde_io_interface.h>
#include <thread>
#include <chrono>
#include <iostream>

using namespace ur_rtde;

int main(int argc, char* argv[])
{
  RTDEIOInterface rtde_io("127.0.0.1");

  rtde_io.setStandardDigitalOut(7, true);
  rtde_io.setAnalogOutputCurrent(1, 0.25);

  return 0;
}