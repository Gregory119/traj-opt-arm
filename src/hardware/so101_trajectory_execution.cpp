#include "so101_bus.hpp"

#include <deque>
#include <iostream>
#include <string>
#include <vector>

static TrajElement wp(double t,
                      double s1, double s2, double s3,
                      double s4, double s5, double s6) {
  TrajElement e;
  e.time = t;  // seconds
  e.val  = {s1, s2, s3, s4, s5, s6};// degrees converted by traj_value_to_ticks()
  return e;
}

int main(int argc, char** argv) {
  SO101Bus::Config cfg;
  if (argc >= 2) cfg.device = argv[1]; // /dev/ttyACM0
  cfg.final_settle_ms = 1200;
  cfg.record_timing_stats = true;

  SO101Bus bus(cfg);
  if (!bus.connect()) {
    std::cerr << "failed to connect to " << cfg.device << "\n";
    return 1;
  }

  std::deque<TrajElement> traj;
  /*
  traj.push_back(wp(0.0,  90,  45,  110,  90,  90,  0));
  traj.push_back(wp(1.0, 100,  90,  110,  90,  45,  0));
  traj.push_back(wp(2.0, 110, 45,  110,  90,  135,  0));
  traj.push_back(wp(3.0,  90, 90,  110, 100,  90,  0));
  traj.push_back(wp(4.0,  60,  45,  110, 110,  45, 0));
  traj.push_back(wp(5.0,  70,  90,  110,  90,  135,  0));
  */
  
  const double dt = 0.020; //step in s
  const double T  = 5.0; //total time
  const int steps = static_cast<int>(T / dt); 
  for (int k = 0; k <= steps; ++k) {
    traj.push_back(TrajElement{.time= k * dt, .val={90, 45, 110, 90, 90, 0}});
  }


  if (!bus.execute_traj_full(traj)) {
    std::cerr << "trajectory execution failed\n";
    return 2;
  }

  std::cout << "done\n";
  return 0;
}













