#include <mujoco/mujoco.h>
#include <iostream>
#include <string>

// MuJoCo data structures
mjModel* m = NULL;      // MuJoCo model
mjData* d = NULL;       // MuJoCo data

int main(int argc, const char** argv) {
    std::cout << "MuJoCo version: " << mj_versionString() << std::endl;

    // Load model from XML file
    // Note: You need a model file, e.g., "hello.xml" in your working directory.
    char error[1000] = "Could not load XML model";
    m = mj_loadXML("hello.xml", NULL, error, 1000);
    if (!m) {
        std::cout << error << std::endl;
        return 1;
    }

    // Make data
    d = mj_makeData(m);

    // Run simulation for 1000 steps
    for (int i = 0; i < 1000; i++) {
        mj_step(m, d);
    }

    std::cout << "Simulation finished." << std::endl;

    // Free model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}
