#include "rtde.h"
#include <iostream>
#include <chrono>
#include <numeric>

using namespace std::chrono;

int main (int argc, char *argv[])
{
  int frequency = 1000;
  int samples = 10;
  RTDE rtde("127.0.0.1");
  rtde.connect();
  rtde.negotiateProtocolVersion();
  rtde.getControllerVersion();
  std::vector<std::string> test_vector;
  rtde.sendOutputSetup(test_vector, test_vector, frequency);
  rtde.sendStart();

  std::vector<int> durations;

  int i=1;
  bool keep_running = true;
  while (keep_running)
  {
    /*if (i % frequency)
    {
      // Write to CSV
    }*/

    if (samples > 0 and i >= samples)
    {
      keep_running = false;
    }

    // Receive data
    auto start = high_resolution_clock::now();
    rtde.receive();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    //std::cout << "Getting a sample took: " << duration.count() << "us" << std::endl;
    durations.push_back(duration.count());
    i += 1;
  }

  double sum = std::accumulate(durations.begin(), durations.end(), 0);
  double average = sum / durations.size();
  std::cout << "Average sample acquisition rate: " << average << "us" << std::endl;


  rtde.disconnect();
  return 0;
}