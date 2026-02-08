#pragma once

#include <cstddef>
#include <cstdint>

namespace so101 {

// open ort
int open_port_1Mbps(const char* path);

// send ping
// true if valid
bool feetech_ping(int fd, uint8_t id, int timeout_ms = 10);

// write a one byte to register address
// wait for status reply
bool feetech_write_byte(int fd, uint8_t id, uint8_t address, uint8_t value, int timeout_ms = 50);

// write bytes starting at register address
// return true if success
bool feetech_write_bytes(int fd,
                         uint8_t id,
                         uint8_t start_address,
                         const uint8_t* data,
                         size_t data_len,
                         int timeout_ms = 50,
                         uint8_t* out_error = nullptr);

// read bytes starting at start_address
// returns true if a valid packet is received
bool feetech_read_bytes(int fd,
                        uint8_t id,
                        uint8_t start_address,
                        uint8_t* out,
                        size_t out_len,
                        int timeout_ms = 50,
                        uint8_t* out_error = nullptr);

// state
struct ServoStateBasic {
    uint8_t  error = 0xFF;

    // parsed fields
    uint16_t present_position = 0;
    int16_t  present_speed    = 0;
    int16_t  present_load     = 0;
    uint8_t  present_voltage_raw = 0;   // 0.1V scale
    uint8_t  present_temp_c      = 0;   // degC

    // block read from control table
    uint8_t raw[8] = {0};
};

// read 8 bytes starting at 0x38
bool feetech_read_state_basic(int fd, uint8_t id, ServoStateBasic* out, int timeout_ms = 80);

} // namespace so101
