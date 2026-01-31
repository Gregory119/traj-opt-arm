#include "simulator.hpp"

// main function
int main(int argc, const char** argv) {
  // check command-line arguments
  // if (argc!=2) {
  //   std::printf(" USAGE:  basic modelfile\n");
  //   return EXIT_FAILURE;
  // }

    Simulator::getInstance()->run();
    
  return EXIT_SUCCESS;
}
