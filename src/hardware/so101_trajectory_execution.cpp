#include "so101_bus.hpp"

#include <iostream>


int main(int argc, char** argv) {
  SO101Bus::Config cfg;
  if (argc >= 2) cfg.device = argv[1]; // /dev/ttyACM0
  cfg.record_timing_stats = true;

  cfg.sid_to_pos_tic_range = {{
      {1, ServoPosRange{751, 3470}},
      {2, ServoPosRange{920, 3281}},
      {3, ServoPosRange{933, 3137}},
      {4, ServoPosRange{875, 3215}},
      {5, ServoPosRange{221, 4022}},
      {6, ServoPosRange{2037, 3499}},
  }};

  SO101Bus bus(cfg);
  if (!bus.connect()) {
    std::cerr << "failed to connect to " << cfg.device << "\n";
    return 1;
  }

  DiscreteJointStateTraj traj;
  
  const double dt = 0.020; //step in s
  const double T  = 5.0; //total time
  const int steps = static_cast<int>(T / dt); 
  for (int k = 0; k <= steps; ++k) {
    traj.push_back(
        JointState{.time = k * dt,
                   .q = (Eigen::VectorXd(6) << 0.0 * std::numbers::pi / 180.0,
                         0.0,
                         0.0,
                         0.0,
                         0.0,
                         0.0 * std::numbers::pi / 180.0)
                            .finished()});
  }

  if (!bus.execute_traj_full(traj, PosUnit::RADIAN)) {
    std::cerr << "trajectory execution failed\n";
    return 2;
  }

  std::cout << "done\n";
  return 0;
}













