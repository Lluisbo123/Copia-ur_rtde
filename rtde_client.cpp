#include "rtde.h"
#include <iostream>

int main (int argc, char *argv[])
{
  RTDE rtde("127.0.0.1");
  rtde.connect();
  rtde.getControllerVersion();
  std::vector<std::string> test_vector;
  rtde.sendOutputSetup(test_vector, test_vector, 125);
  rtde.disconnect();
  return 0;
}