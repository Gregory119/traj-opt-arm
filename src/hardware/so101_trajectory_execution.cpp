#include "so101_bus.hpp"

#include <deque>
#include <iostream>
#include <string>
#include <vector>


int main(int argc, char** argv) {
  SO101Bus::Config cfg;
  if (argc >= 2) cfg.device = argv[1]; // /dev/ttyACM0
  cfg.record_timing_stats = true;

  SO101Bus bus(cfg);
  if (!bus.connect()) {
    std::cerr << "failed to connect to " << cfg.device << "\n";
    return 1;
  }

  std::deque<TrajElement> traj;
  
  const double dt = 0.020; //step in s
  const double T  = 5.0; //total time
  const int steps = static_cast<int>(T / dt); 
  for (int k = 0; k <= steps; ++k) {
    traj.push_back(TrajElement{.time= k * dt, .val={90, 45, 110, 90, 90, 0}});
  }

  if (!bus.execute_traj_full(traj, PosUnit::DEGREE)) {
    std::cerr << "trajectory execution failed\n";
    return 2;
  }

  std::cout << "done\n";
  return 0;
}













