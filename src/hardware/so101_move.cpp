#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include "so101_bus.hpp"

static inline uint8_t lo(uint16_t v) { return static_cast<uint8_t>(v & 0xFF); } //lower 8 bits
static inline uint8_t hi(uint16_t v) { return static_cast<uint8_t>((v >> 8) & 0xFF); } //highest 8 bits

int main(int argc, char** argv) {

    const char* device = (argc >= 4) ? argv[3] : "/dev/ttyACM0"; //path to the device
    int exit_after_s = (argc >= 5) ? std::atoi(argv[4]) : 3; //convert string into int
    if (exit_after_s < 0) exit_after_s = 0; 

    long id_l  = std::strtol(argv[1], nullptr, 10); // string to long
    long pos_l = std::strtol(argv[2], nullptr, 10); //string to long

    int fd = so101::open_port_1Mbps(device); //open the port and use device path, return file descriptor
    if (fd < 0) { //if error code returned
        std::fprintf(stderr, "cant open %s: %s\n", device, std::strerror(errno));
        return 1;
    }

    const uint8_t id = static_cast<uint8_t>(id_l); //convert the long into an 8 bit int
    const uint16_t pos = static_cast<uint16_t>(pos_l); // convert the long int o an 16 bit unsigned int

    // check response
    if (!so101::feetech_ping(fd, id, 15)) { //returns true if valid response is received, stop on ping failure
        std::fprintf(stderr, "servo ID %d did not reply to ping\n", (int)id); //print the error message if no response received
    }

    // 6 byte instruction block at address 0x2A
    // [pos_L,pos_H, time_L,time_H, speed_L,speed_H]
    const uint8_t GOAL_BLOCK_ADDR = 0x2A;
    const uint16_t time_ms = 0; //ignore wait time
    const uint16_t speed   = 200; //default speed

    uint8_t payload[6] = { lo(pos), hi(pos), lo(time_ms), hi(time_ms), lo(speed), hi(speed) }; // 6 byte parameter payload to be sent

    uint8_t err = 0xFF; // initialize error byte
    bool ok = so101::feetech_write_bytes(fd, id, GOAL_BLOCK_ADDR, payload, sizeof(payload), 80, &err); //send packet data, returns true if write succeeded amd servo replies, and error is 0
    if (!ok) {
        std::fprintf(stderr, "move command not performed err=0x%02X\n", err); //print error message and error code if write is not performed
        ::close(fd); // close port
        return 1; // failed
    }

    std::printf("move command performed \n waiting %d s \n", exit_after_s); //print success message if write is performed
    ::sleep((unsigned)exit_after_s); // wait to close program after packet is sent

    ::close(fd); // close port
    return 0; // success
}
