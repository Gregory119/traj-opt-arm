#include "IpIpoptApplication.hpp"
#include "simulator.hpp"

// main function
int main(int argc, const char **argv)
{
    std::deque<JointState> traj{
        {.time = 2.0,
         .q = (Eigen::VectorXd(6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished()},
        {.time = 3.0,
         .q = (Eigen::VectorXd(6) << 1.0, -1.0, 0.0, 0.0, 0.0, 0.0).finished()},
    };
    Simulator::getInstance()->setTrajectory(std::move(traj));
    Simulator::getInstance()->run();

    return EXIT_SUCCESS;
}
