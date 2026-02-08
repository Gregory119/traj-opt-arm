#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <csignal>
#include <unistd.h>

#include "so101_bus.hpp"

static volatile sig_atomic_t g_stop = 0; //flag set for safe read/write

static void on_sigint(int) { g_stop = 1; } // set function for the flag

static void restore_terminal() { //function for rewriting terminal output
    std::printf("\033[?25h"); //show cursor
    std::fflush(stdout); //clear output buffer
}

static double now_sec_monotonic() { //timing function
    timespec ts{}; //timespec object
    clock_gettime(CLOCK_MONOTONIC, &ts); // get current timestamp and store in the timespec object
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec; // return seconds from CLOCK_MONOTONIC
}

int main(int argc, char** argv) {
    const char* device = (argc >= 2) ? argv[1] : "/dev/ttyACM0"; // path to device
    double rate_hz = (argc >= 3) ? std::atof(argv[2]) : 10.0; // set the rewrite rate
    if (rate_hz <= 0.1) rate_hz = 0.1;
    if (rate_hz > 100.0) rate_hz = 100.0;

    int fd = so101::open_port_1Mbps(device); // open port to read data
    if (fd < 0) {
        std::fprintf(stderr, "cant open %s: %s\n", device, std::strerror(errno)); // return error code and close port if configuration fails
        return 1;
    }

    std::signal(SIGINT, on_sigint); // set the sig handler 
    std::atexit(restore_terminal); // register the function to call at program termination

    // clear screen
    std::printf("\033[2J\033[H\033[?25l");
    std::fflush(stdout);

    const double t0 = now_sec_monotonic(); // set the start time
    const int period_us = (int)(1e6 / rate_hz); // compute the terminal display period in microseconds

    while (!g_stop) {
        std::printf("\033[H"); // home

        const double t = now_sec_monotonic() - t0; // measure time since the start of the program
        // print column descriptions
        std::printf("SO-101 Servo State (IDs 1..6)   t=%.2f s   rate=%.1f Hz   device=%s\n", t, rate_hz, device);
        std::printf("--------------------------------------------------------------------------------\n");
        std::printf(" ID |  Pos   |  Speed  |  Load   |  Volt  | Temp | Err | Raw (8 bytes @0x38)\n");
        std::printf("--------------------------------------------------------------------------------\n");

        for (int id = 1; id <= 6; ++id) { //iterate through all 6 servo IDs(generalize for any ID set later)
            so101::ServoStateBasic st{}; // initialize ServoStateBasic object. Data written to here
            bool ok = so101::feetech_read_state_basic(fd, (uint8_t)id, &st, 40); //read the state for the current servo ID, returns true if valid packet is received

            if (!ok) { // execute on failed read
                // no reply then show ping
                bool alive = so101::feetech_ping(fd, (uint8_t)id, 10);
                std::printf(" %2d |  ----  |  ----   |  ----   |  ----  | ---- | %s |\n",
                            id, alive ? "PING" : "----"); //replace data display for the current ID with this
                continue; // skip current iteration
            }

            // 0.1V units for voltage
            double volts = 0.1 * (double)st.present_voltage_raw;

            std::printf(" %2d | %5u | %7d | %7d | %5.1f | %4u | 0x%02X |", //print the received data located in the ServoStateBasic object 
                        id,
                        (unsigned)st.present_position,
                        (int)st.present_speed,
                        (int)st.present_load,
                        volts,
                        (unsigned)st.present_temp_c,
                        (unsigned)st.error);

            for (int k = 0; k < 8; ++k) { // print the raw 8 bytes (hex form) in last column
                std::printf(" %02X", (unsigned)st.raw[k]);
            }
            std::printf("\n"); //start new row
        }

        std::printf("\033[J"); // clear outpt after printing all servo data
        std::fflush(stdout); // clear the output writer buffer

        usleep(period_us); // wait a pre determined amount of microseconds before proceeding
    }

    ::close(fd); // close the file descriptor after program termination

    std::printf("\033[H\033[JStopped.\n");
    return 0;
}
