#include "simulator.hpp"

// main function
int main(int argc, const char **argv)
{
    std::deque<JointState> traj{
        {.time = 1.0,
         .q = (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished()},
        {.time = 1.5,
         .q = (Eigen::VectorXd(6) << 15.0 * std::numbers::pi / 180.0,
               0.0,
               0.0,
               0.0,
               0.0,
               0.0)
                  .finished()},
    };
    Simulator::getInstance()->setTrajectory(std::move(traj));
    Simulator::getInstance()->run();

    return EXIT_SUCCESS;
}
