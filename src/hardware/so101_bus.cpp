#include "so101_bus.hpp"

#include <cerrno> 
#include <cstdint>
#include <cstring>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>



namespace {

// return the 8 bit checksum
static uint8_t checksum_feetech(const uint8_t* body_no_header, size_t n) {
    
    uint32_t sum = 0; //initialize sum as 0
    for (size_t i = 0; i < n; ++i) sum += body_no_header[i]; // sum each byte 
    return static_cast<uint8_t>(~(sum & 0xFF)); //bitwise AND between the sum and 11111111(also truncate from 32 to 8 bit type)
                                                // then invert with a bitwise NOT
}


// this function writes raw bytes to fd(a file descriptor which serves as a destination for the serial port)
// buf is a pointer to the first byte to transmit, starting with 0xFF, 0xFF, ... 
// n is number of bytes to transmit
static bool write_all(int fd, const uint8_t* buf, size_t n) {

    while (n > 0) { //loop through all bytes
        ssize_t w = ::write(fd, buf, n); // sys call to write to /dev/ttyACM0 as if it were a file
                                         // returns number of bytes written
        if (w < 0) {
            if (errno == EINTR) continue; // error code: "function interrupted by a signal"
            return false; //this byte was not written
        }
        buf += static_cast<size_t>(w); //increment pointer by w bytes
        n   -= static_cast<size_t>(w); //decrement bytes remaining by w
    }
    return true; //all bytes successfully written
}


// this function waits within a timeframe of timeout_ms for the fd to become readable then performs one read of up to n bytes
static ssize_t read_with_timeout(int fd, uint8_t* buf, size_t n, int timeout_ms) {
    fd_set set; //declare set as type fd_set
    FD_ZERO(&set); // clears fd_set
    FD_SET(fd, &set); // adds fd to the set of descriptors

    timeval tv{}; //initialize timeval struct to handle time
    tv.tv_sec  = timeout_ms / 1000; //measure seconds
    tv.tv_usec = (timeout_ms % 1000) * 1000; // remaining time in microseconds with a modular operation and multiplying by 1000

    int rv = ::select(fd + 1, &set, nullptr, nullptr, &tv); //sys call for IO, sets up full set of descriptors
                                                            // fd+1 is highest number file descriptor for set
                                                            // args 3,4 null because they handle writing and exceptions
                                                            // tv tells select how ling to wait
    if (rv <= 0) return 0; // timeout or error, errno will display the error code
    return ::read(fd, buf, n);  // returns number of bytes read, 0 = closed, -1 = error
}

// read the packet incoming from descriptor fd at location expected_id
// wait timeout_ms t receive packet
// out_error is written to if packet is successfully parsed
// uint8_t out_params stores the parameter data
// expected_params len is the number of params in bytes
static bool read_status_packet_with_params(int fd,
                                           uint8_t expected_id,
                                           int timeout_ms,
                                           uint8_t* out_error,
                                           std::vector<uint8_t>* out_params,
                                           size_t expected_params_len) {
    // format: FF FF ID LEN ERR [params] CHK
    uint8_t r[256]; //initialize an array of 256 bytes
    int got = 0; // initialize received bytes at 0

    // read until timeout
    const int slice_ms = 5; // 5 ms per try
    int tries = (timeout_ms + slice_ms - 1) / slice_ms; // number of read attempts
    if (tries < 1) tries = 1; // start number of tries at 1 instead of 0

    for (int t = 0; t < tries && got < (int)sizeof(r); ++t) { // 
        ssize_t k = read_with_timeout(fd, r + got, sizeof(r) - got, slice_ms); //return number of bytes read
        if (k > 0) got += (int)k; // add bytes read for this try to the total number of bytes read

        // parse
        if (got < 6) continue; //skip this iteration if received bytes is less than 6

        for (int i = 0; i <= got - 6; ++i) { //iterate through the received bytes starting at the first
            if (r[i] != 0xFF || r[i+1] != 0xFF) continue; //skip this iteration if the first byte 
                                                          //is not 0xFF(per feetech documentation)

            uint8_t id  = r[i+2]; // id is third byte
            uint8_t len = r[i+3]; //length is the fourth byte
            if (id != expected_id) continue; //skip iteration if the id is not the id we are looking to read

            int frame_bytes = 4 + (int)len;  // FF FF ID LEN + (ERR CHK)
            if (i + frame_bytes > got) continue; //skip iteration if the iterator summed with the number of 
                                                //bytes is greater than the amount of read bytes

            const uint8_t err = r[i+4]; // error byte is fourth
            const uint8_t chk = r[i + frame_bytes - 1]; // checksum byte is the last byte

            const int params_len = (int)len - 2; // LEN counts ERR andCHK so params = LEN-2
            if (params_len < 0) continue; // if params length is invalid, skip this iteration

            //skip iteration if parameter lengths are invalid or expected vs actual dont match
            if (expected_params_len != (size_t)-1 && (size_t)params_len != expected_params_len) continue; 

            // checksum over ID, LEN, ERR, params
            std::vector<uint8_t> body; //initialize byte vector to store body data
            body.reserve((size_t)len + 2); // reserve len +2 bytes of memory
            body.push_back(id); // add id to the end of body
            body.push_back(len); // add len to the end of body
            for (int j = 0; j < (int)len - 1; ++j) { // ERR + params
                body.push_back(r[i + 4 + j]); //iteratre through read bytes and add it to the body
            }
            const uint8_t expect = checksum_feetech(body.data(), body.size()); // compute checksum
            if (expect != chk) continue; //skip iteration if checksum is not valid

            if (out_error) *out_error = err; //write error byte
            if (out_params) {
                out_params->assign(r + i + 5, r + i + 5 + params_len);
            }
            return true; //valid packet
        }
    }

    return false; // no valid packet found
}

} // namespace


namespace so101 {

//this function opens a serial port at a device path using system calls
int open_port_1Mbps(const char* path) { 
    int fd = ::open(path, O_RDWR | O_NOCTTY | O_SYNC); //open file descriptor using the device path 
    if (fd < 0) return -1; // -1 returned by open is an error

    termios tty{}; //struct termios variable
    if (tcgetattr(fd, &tty) != 0) { ::close(fd); return -1; } //if port attributes are not available, close it

    cfmakeraw(&tty); //configure port for raw IO

#ifdef B1000000 //visible to compiler
    cfsetispeed(&tty, B1000000); //setting speed of the port
    cfsetospeed(&tty, B1000000);
#else // visible to compiler
    ::close(fd); //close the port if the speed is not available
    errno = ENOTSUP; // error code meaning that the speed is not supported
    return -1;
#endif
    //set flags
    tty.c_cflag |= (CLOCAL | CREAD); // ignore control lines
    tty.c_cflag &= ~CSTOPB; //single stop bit
    tty.c_cflag &= ~PARENB; //no parity
    tty.c_cflag &= ~CRTSCTS; // no HW flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 data bits

    // return read() immediately
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    // 
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return -1; } // close the port if the attributes can not immediately be set
    tcflush(fd, TCIOFLUSH); // discard the data at device fd
    return fd; //return file descriptor after configuring the port
}



bool feetech_ping(int fd, uint8_t id, int timeout_ms) {
    // format FF FF ID 02 01 CHK
    uint8_t body[3] = {id, 0x02, 0x01}; // body is 3 bytes
    uint8_t chk     = checksum_feetech(body, sizeof(body)); //compute checksum
    uint8_t pkt[6]  = {0xFF, 0xFF, id, 0x02, 0x01, chk}; // full packet allocation

    tcflush(fd, TCIFLUSH); // clear old data
    if (!write_all(fd, pkt, sizeof(pkt))) return false; // return false if write fails

    uint8_t err = 0xFF; // initialize error byte
    return read_status_packet_with_params(fd, id, timeout_ms, &err,nullptr,(size_t)-1); //return true if valid packet is received, 
                                                                                        //false if not
}

bool feetech_write_byte(int fd, uint8_t id, uint8_t address, uint8_t value, int timeout_ms) {
    return feetech_write_bytes(fd, id, address, &value, 1, timeout_ms, nullptr); //useful for writing a single param byte(like position)
}

bool feetech_write_bytes(int fd,
                         uint8_t id,
                         uint8_t start_address,
                         const uint8_t* data,
                         size_t data_len,
                         int timeout_ms,
                         uint8_t* out_error) {
    // FF FF ID LENGTH INSTR(0x03) ADDR DATA... CHK
    // length = (params) + 2, params = 1(addr) + data_len
    if (!data && data_len != 0) { errno = EINVAL; return false; } //if no data but non zero length, set error code and return false

    const uint8_t length = static_cast<uint8_t>(data_len + 3); // compute length field: addr + data_len + 2
    std::vector<uint8_t> body; //initialize byte vector
    body.reserve(3 + 1 + data_len); //reserve this number of bytes worth of memory
    body.push_back(id); // add id byte to the end of the body
    body.push_back(length); // add length byte to the end of body
    body.push_back(0x03); // add this byte after length
    body.push_back(start_address); // add start address to the end of body
    for (size_t i = 0; i < data_len; ++i) body.push_back(data[i]); // iterate through data bytes in data and add it to body

    const uint8_t chk = checksum_feetech(body.data(), body.size()); // calculate checksum 

    std::vector<uint8_t> pkt; // packet byte vector
    pkt.reserve(2 + body.size() + 1); // reserve this amount of bytes, includes space for checksum 
    pkt.push_back(0xFF); // add packet start bytes
    pkt.push_back(0xFF);
    pkt.insert(pkt.end(), body.begin(), body.end()); 
    pkt.push_back(chk); // add the check sum

    tcflush(fd, TCIFLUSH); // clear reply bytes located at the file descriptor before writing
    if (!write_all(fd, pkt.data(), pkt.size())) return false; // write packet data into the file descriptor

    // if broadcast ID 0xFE does not reply
    if (id == 0xFE) return true;

    uint8_t err = 0xFF; //errpr byte
    if (!read_status_packet_with_params(fd, id, timeout_ms, &err,nullptr,(size_t)-1)) return false; // return false if no packet received before timeout
                                                                                                    // or invalid checksum, parse fail
    if (out_error) *out_error = err; // write write error byte
    return (err == 0x00); // returns false if there is an error
}

bool feetech_read_bytes(int fd,
                        uint8_t id,
                        uint8_t start_address,
                        uint8_t* out,
                        size_t out_len,
                        int timeout_ms,
                        uint8_t* out_error) {
    if (!out && out_len != 0) { errno = EINVAL; return false; } // if no data with nonzero data length, write error code and return false
    if (out_len > 250) { errno = EINVAL; return false; } // check max length

    // formar FF FF ID 04 02 <addr> <len> CHK
    uint8_t body[5] = { // set body bytes
        id,
        0x04,
        0x02,
        start_address,
        static_cast<uint8_t>(out_len)
    };
    uint8_t chk = checksum_feetech(body, sizeof(body)); // checksum byte
    uint8_t pkt[8] = {0xFF, 0xFF, body[0], body[1], body[2], body[3], body[4], chk}; // full packet memory allocation

    tcflush(fd, TCIFLUSH); // flush current response bytes before performing write
    if (!write_all(fd, pkt, sizeof(pkt))) return false; // write packet data and return false if write fail occurs

    std::vector<uint8_t> params; // initialize byte vector for parameters
    uint8_t err = 0xFF; // initialize error byte
    if (!read_status_packet_with_params(fd, id, timeout_ms, &err, &params, out_len)) return false; // read the status packet

    if (out_error) *out_error = err; // rewrite error byte
    if (err != 0x00) return false; // return false if there is an error
    if (params.size() != out_len) return false; // parameter bytes mismatch with expected length, return false

    if (out_len) std::memcpy(out, params.data(), out_len); // copy the parameter data to out address
    return true; // valid packet read
}

bool feetech_read_state_basic(int fd, uint8_t id, ServoStateBasic* out, int timeout_ms) {
    if (!out) { errno = EINVAL; return false; } //error code if out address is not valid

    // position documented at 0x38 2 bytes
    const uint8_t START = 0x38;
    const size_t  N     = 8;

    uint8_t err = 0xFF; // initialize error byte
    if (!feetech_read_bytes(fd, id, START, out->raw, N, timeout_ms, &err)) {
        out->error = err; // write error code to ServoStateBasic object
        return false; // return false when a read fail occurs
    }
    out->error = err; // copy error byte to ServoStateBasic object

    auto u16 = [&](int idx) -> uint16_t { // 16 bit unsigned int from combined bytes 
        return (uint16_t)out->raw[idx] | ((uint16_t)out->raw[idx + 1] << 8); // combine low and high bytes into a 16 bit unsinged int
    };
    auto s16 = [&](int idx) -> int16_t { 
        return (int16_t)u16(idx); //casts unsigned int into signed
    };

    // mapping
    out->present_position     = u16(0); // 0x38 to 0x39
    out->present_speed        = s16(2);  // 0x3A to 0x3B
    out->present_load         = s16(4); // 0x3C to 0x3D
    out->present_voltage_raw  = out->raw[6]; // 0x3E
    out->present_temp_c       = out->raw[7]; // 0x3F

    return true;
}

} // namespace so101
