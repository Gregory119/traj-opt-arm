#include "simulator.hpp"

// main function
int main(int argc, const char **argv)
{
    std::deque<TrajElement> traj{
        {.time = 3.0, .val = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
        {.time = 4.0, .val = {1.0, -1.0, 0.0, 0.0, 0.0, 0.0}},
    };
    Simulator::getInstance()->setTrajectory(std::move(traj));
    Simulator::getInstance()->run();

    return EXIT_SUCCESS;
}
