#include "simulator.hpp"

// main function
int main(int argc, const char** argv) {
  // check command-line arguments
  // if (argc!=2) {
  //   std::printf(" USAGE:  basic modelfile\n");
  //   return EXIT_FAILURE;
  // }

    Simulator::getInstance()->setTrajectory({{.time=3.0, .val={1.0, 0.0, 0.0, 0.0, 0.0, 0.0}}});
    Simulator::getInstance()->run();
    
  return EXIT_SUCCESS;
}
