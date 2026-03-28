#include <iostream>

#include "so101_bus.hpp"

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cout << "Missing calibration file path." << std::endl;
        return 0;
    }
    Calibration calibration(std::string{argv[1]});
    SO101Bus::Config cfg(calibration);
    cfg.record_timing_stats = true;

    SO101Bus bus(cfg);
    if (!bus.connect()) {
        std::cerr << "failed to connect to " << cfg.device << "\n";
        return 1;
    }

    DiscreteJointStateTraj traj;

    const double dt = 0.020;  // step in s
    const double T = 5.0;     // total time
    const int steps = static_cast<int>(T / dt);
    for (int k = 0; k <= steps; ++k) {
        traj.push_back(JointState{
            .time = k * dt,
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
