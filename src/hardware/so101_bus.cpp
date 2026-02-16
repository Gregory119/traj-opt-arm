#include "so101_bus.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <thread>
#include <vector>
#include <utility>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

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

static inline uint8_t lo(uint16_t v) { return static_cast<uint8_t>(v & 0xFF); } // return lowest 8 bits of 16 bit int
static inline uint8_t hi(uint16_t v) { return static_cast<uint8_t>((v >> 8) & 0xFF); } //  return highest 8 bits of 16 bit int

} //end of namespace


// SO101Bus class handling lifecycle / connection

SO101Bus::SO101Bus() : SO101Bus(Config{}) {} //delegating default constructor for SO101_Bus and configuration classes
SO101Bus::SO101Bus(Config cfg) : cfg_(std::move(cfg)) {} //delegating parameterized constructor for SO101_Bus and 
                                                        // configuration classes

void SO101Bus::set_config(const Config& cfg) { //setter for the particular configuration
  cfg_ = cfg;
}

bool SO101Bus::connect() {
  // re open using cfg_.device
  port_.close();
  if (!port_.open(cfg_.device)) {
    std::fprintf(stderr, "cant open %s: %s\n", cfg_.device.c_str(), std::strerror(errno));
    return false;
  }

  //ping all servos using the file descriptor, set of servo ids, and allowed time before triggering timeout
  // return false if one doesn't reply
  if (!SO101Bus::ping_all(port_.fd(), cfg_.ids, cfg_.ping_timeout_ms)) {
    std::fprintf(stderr, "one or more servos did not reply to ping\n");
    port_.close();
    return false;
  }
  return true;
}

// overloaded function with device parameter that calls connect()
bool SO101Bus::connect(const std::string& device) {
  cfg_.device = device;
  return connect();
}

// close port without throwing an exception
void SO101Bus::disconnect() noexcept {
  port_.close();
}

//this function checks if the port is configured and open by calling is_open()
// If false is returned, call connect() to open and configure port
bool SO101Bus::ensure_connected_() {
  if (port_.is_open()) return true;
  return connect();
}

//overloadewd function that calls a parameterized function to ping all servos using file descriptor ,
// set of servo ids, and timeout associated with the port and configuration obejcts
bool SO101Bus::ping_all() {
  if (!ensure_connected_()) return false;
  return SO101Bus::ping_all(port_.fd(), cfg_.ids, cfg_.ping_timeout_ms);
}


// SO101Bus::Port, bind the lifetime of the FD to the lifetime of the port object

SO101Bus::Port::Port(const char* device) { (void)open(device); } // parameterized constructor for port object with c string input
SO101Bus::Port::Port(const std::string& device) { (void)open(device); } //overloaded parameterized constructor w string as input

SO101Bus::Port::~Port() { close(); } // Port object destructor

SO101Bus::Port::Port(Port&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; } // move constructor for transferring fd to other object

// move assignment 
SO101Bus::Port& SO101Bus::Port::operator=(Port&& other) noexcept { 
  if (this == &other) return *this;
  close();
  fd_ = other.fd_;
  other.fd_ = -1;
  return *this;
}

// reset any existing FD then open and configure a new port, return false if open or configure fails. C string input
bool SO101Bus::Port::open(const char* device) {
  close();
  int fd = SO101Bus::open_port_1Mbps(device);
  if (fd < 0) return false;
  fd_ = fd;
  return true;
}

//overloaded function for accepting string input
bool SO101Bus::Port::open(const std::string& device) {
  return open(device.c_str());
}

//close port if open and reset fd to -1 to represent closed state
void SO101Bus::Port::close() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}


int SO101Bus::open_port_1Mbps(const char* path) {
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

bool SO101Bus::feetech_ping(int fd, uint8_t id, int timeout_ms) {
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

bool SO101Bus::feetech_write_byte(int fd, uint8_t id, uint8_t address, uint8_t value, int timeout_ms) {
  return SO101Bus::feetech_write_bytes(fd, id, address, &value, 1, timeout_ms, 0x03, nullptr);  //useful for writing a single param byte(like position)
}

bool SO101Bus::feetech_write_bytes(int fd,
                                   uint8_t id,
                                   uint8_t start_address,
                                   const uint8_t* data,
                                   size_t data_len,
                                   int timeout_ms,
                                   uint8_t instruc_code,
                                   uint8_t* out_error) {
    // FF FF ID LENGTH INSTR(0x03) ADDR DATA... CHK
    // length = (params) + 2, params = 1(addr) + data_len
    if (!data && data_len != 0) { errno = EINVAL; return false; } //if no data but non zero length, set error code and return false

    const uint8_t length = static_cast<uint8_t>(data_len + 3); // compute length field: addr + data_len + 2
    std::vector<uint8_t> body; //initialize byte vector
    body.reserve(3 + 1 + data_len); //reserve this number of bytes worth of memory
    body.push_back(id); // add id byte to the end of the body
    body.push_back(length); // add length byte to the end of body
    body.push_back(instruc_code); // add this byte after length
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
  if (!read_status_packet_with_params(fd, id, timeout_ms, &err, nullptr, (size_t)-1)) {
    errno = ETIMEDOUT;
    return false; // return false if no packet received before timeout
                  // or invalid checksum, parse fail
  }
  if (out_error) *out_error = err; //write error byte
  return (err == 0x00); // returns false if there is an error
}

bool SO101Bus::feetech_read_bytes(int fd,
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
  if (!read_status_packet_with_params(fd, id, timeout_ms, &err, &params, out_len)) { // read the status packet
    errno = ETIMEDOUT;
    return false;
  }
  if (out_error) *out_error = err; // rewrite error byte
  if (err != 0x00) return false; // return false if there is an error
  if (params.size() != out_len) return false; // parameter bytes mismatch with expected length, return false

  if (out_len) std::memcpy(out, params.data(), out_len); // copy the parameter data to out address
  return true; // valid packet read
}

bool SO101Bus::feetech_read_state_basic(int fd, uint8_t id, ServoStateBasic* out, int timeout_ms) {
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

// map 
static std::vector<uint16_t> robot_mapping(int id, std::vector<uint16_t> v) {
  // servo position ranges for counter clockwise rotation
  // index 0 unused
  constexpr auto& kRanges = SO101Bus::kPosRangeById;

  // degree limits
  constexpr double kDegMin = 0.0;
  constexpr double kDegMax = 180.0;

  auto map_one = [&](int sid, uint16_t in) -> uint16_t {
    // already in servo units
    if (in > 18000u) return in;

    if (sid < 1 || sid > 6) return in;

    const double deg = std::clamp(in / 100.0, kDegMin, kDegMax);
    const double t = (deg - kDegMin) / (kDegMax - kDegMin);

    const auto& r = kRanges[sid];

    const double pos = static_cast<double>(r.pos_min) +
                       t * (static_cast<double>(r.pos_max - r.pos_min));
    const long p = std::lround(pos);
    const long lo = std::min<long>(r.pos_min, r.pos_max);
    const long hi = std::max<long>(r.pos_min, r.pos_max);
    return static_cast<uint16_t>(std::clamp<long>(p, std::max(0L, lo), std::min(65535L, hi)));
  };

  // map all 6 servos
  if (id == 0 && v.size() >= 6) {
    for (int sid = 1; sid <= 6; ++sid) {
      v[(size_t)(sid - 1)] = map_one(sid, v[(size_t)(sid - 1)]);
    }
    return v;
  }

  // map a single servo
  if (id >= 1 && id <= 6) {
    if (!v.empty()) v[0] = map_one(id, v[0]);
    return v;
  }

  // if 6 values , take as servos 1-6
  if (v.size() >= 6) {
    for (int sid = 1; sid <= 6; ++sid) {
      v[(size_t)(sid - 1)] = map_one(sid, v[(size_t)(sid - 1)]);
    }
  }

  return v;
}


// position only sync write

static bool feetech_sync_write_positions(int fd,
                                         const std::array<uint8_t, 6>& ids,
                                         const std::array<uint16_t, 6>& pos) {
  constexpr uint8_t kBroadcastId = 0xFE;
  constexpr uint8_t kInstruction = 0x83;  // sync write
  constexpr uint8_t kGoalPosAddr = 0x2A; // goal Position (2 bytes)
  constexpr uint8_t kMiniLen = 0x02; // pos(2)

  constexpr int kReadTimeoutMs   = 80; // for reading current pos
  constexpr int kSoftMarginUnits = 5;// stay away from hard endpoints
  constexpr uint16_t kEpsUnits   = 25; // midpoint tolerance
  constexpr int kSettleTimeoutMs = 600;// wait up to this long for midpoint
  constexpr int kPollPeriodMs    = 15;// polling interval during settle wait

  // send a position only SYNCWRITE
  auto send_sync = [&](const std::array<uint16_t, 6>& p) -> bool {
    uint8_t payload[1 + 6 * (1 + 2)]{};
    payload[0] = kMiniLen;

    for (int j = 0; j < 6; ++j) {
      const uint8_t id = ids[j];
      const uint16_t v = p[j];
      const int base = 1 + j * 3;
      payload[base + 0] = id;
      payload[base + 1] = lo(v);
      payload[base + 2] = hi(v);
    }

    uint8_t err = 0xFF;
    return SO101Bus::feetech_write_bytes(fd,
                                        kBroadcastId,
                                        kGoalPosAddr,
                                        payload,
                                        sizeof(payload),
                                        /*timeout_ms=*/80,
                                        kInstruction,
                                        &err);
  };

  // servo bounds
  // use robot_mapping(id,{0}) and robot_mapping(id,{18000}) if robot_mapping is implemented
  auto bounds_for_id = [&](int id) -> std::pair<uint16_t, uint16_t> {
    // fallback mapping
    if (id < 1 || id > 6) return {0, 0};

    const auto& r = SO101Bus::kPosRangeById[(size_t)id];
    uint16_t e0 = static_cast<uint16_t>(std::clamp<int>(r.pos_min, 0, 65535));
    uint16_t e1 = static_cast<uint16_t>(std::clamp<int>(r.pos_max, 0, 65535));

    const auto m0   = robot_mapping(id, std::vector<uint16_t>{0});
    const auto m180 = robot_mapping(id, std::vector<uint16_t>{18000});

    const bool mapping_looks_real =
        (m0.size() == 1 && m180.size() == 1 && !(m0[0] == 0 && m180[0] == 18000));

    if (mapping_looks_real) {
      e0 = m0[0];
      e1 = m180[0];
    }

    uint16_t loB = std::min(e0, e1);
    uint16_t hiB = std::max(e0, e1);

    // apply margin
    if (hiB > loB + 2 * kSoftMarginUnits) {
      loB = static_cast<uint16_t>(loB + kSoftMarginUnits);
      hiB = static_cast<uint16_t>(hiB - kSoftMarginUnits);
    }

    return {loB, hiB};
  };

  auto clamp_to_bounds = [&](int id, uint16_t p) -> uint16_t {
    const auto [loB, hiB] = bounds_for_id(id);
    return static_cast<uint16_t>(std::clamp<int>((int)p, (int)loB, (int)hiB));
  };

  auto midpoint_for_id = [&](int id) -> uint16_t {
    const auto [loB, hiB] = bounds_for_id(id);
    return static_cast<uint16_t>((uint32_t(loB) + uint32_t(hiB)) / 2u);
  };

  // print bounds
  static bool printed_bounds = false;
  if (!printed_bounds) {
    printed_bounds = true;
    for (int joint = 1; joint <= 6; ++joint) {
      const auto [loB, hiB] = bounds_for_id(joint);
      std::fprintf(stderr, "bounds joint %d: [%u, %u]\n", joint, loB, hiB);
    }
  }

  // clamp the commanded goal
  std::array<uint16_t, 6> goal = pos;
  for (int j = 0; j < 6; ++j) {
    const int joint = j + 1;
    goal[j] = clamp_to_bounds(joint, goal[j]);
  }


  // read current positions for midpoitn check
  std::array<uint16_t, 6> cur{};
  bool can_midpoint_guard = true;

  for (int j = 0; j < 6; ++j) {
    const uint8_t id = ids[j];
    SO101Bus::ServoStateBasic st{};

    if (!SO101Bus::feetech_read_state_basic(fd, id, &st, kReadTimeoutMs)) {
      std::fprintf(stderr,
                   "feetech_sync_write_positions: read_state_basic failed on id=%u (errno=%d: %s, err=0x%02X). "
                   "skipping midpoint guard and sending direct goal\n",
                   id, errno, std::strerror(errno), st.error);
      can_midpoint_guard = false;
      break;
    }

    cur[j] = st.present_position;
  }

  if (!can_midpoint_guard) {
    return send_sync(goal);
  }


  // decide which servos need the midpoint intermediate command
  std::array<uint16_t, 6> mid_cmd{};
  std::array<bool, 6>     needs_mid{};
  bool any_mid = false;

  for (int j = 0; j < 6; ++j) {
    const int joint = j + 1;

    const uint16_t mid = midpoint_for_id(joint);

    const uint16_t cur_c  = clamp_to_bounds(joint, cur[j]);
    const uint16_t goal_c = goal[j];

    const bool cur_half  = (cur_c  > mid);
    const bool goal_half = (goal_c > mid);

    const bool need = (cur_half != goal_half);
    needs_mid[j] = need;
    any_mid |= need;

    mid_cmd[j] = need ? mid : goal_c;
  }


  // if no servo crosses halves send the clamped goals
  if (!any_mid) {
    return send_sync(goal);
  }

  // send the intermediate midpoint command
  if (!send_sync(mid_cmd)) return false;

  //  wait for crossing servos
  const auto t0 = std::chrono::steady_clock::now();
  while (true) {
    bool all_settled = true;

    for (int j = 0; j < 6; ++j) {
      if (!needs_mid[j]) continue;

      const uint8_t id = ids[j];
      const int joint = j + 1;
      const uint16_t mid = midpoint_for_id(joint);

      SO101Bus::ServoStateBasic st{};
      if (!SO101Bus::feetech_read_state_basic(fd, id, &st, kReadTimeoutMs)) {
        std::fprintf(stderr,
                     "feetech_sync_write_positions: midpoint poll read failed on id=%u (errno=%d: %s, err=0x%02X). "
                     "Proceeding to final goal write.\n",
                     id, errno, std::strerror(errno), st.error);
        all_settled = true;   // stop waiting and issue final goals
        break;
      }

      const uint16_t now_p = clamp_to_bounds(joint, st.present_position);
      const uint16_t diff = (now_p > mid) ? (now_p - mid) : (mid - now_p);
      if (diff > kEpsUnits) all_settled = false;

    }

    if (all_settled) break;

    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
    if (elapsed_ms >= kSettleTimeoutMs) break;

    std::this_thread::sleep_for(std::chrono::milliseconds(kPollPeriodMs));
  }

  // send the final goal command
  return send_sync(goal);
}


static bool feetech_sync_write_positions(int fd, const std::array<uint16_t, 6>& pos) {
  constexpr std::array<uint8_t, 6> kDefaultIds{{1,2,3,4,5,6}};
  return feetech_sync_write_positions(fd, kDefaultIds, pos);
}

// reads the first uint16_t in each cell as position.
bool SO101Bus::feetech_sync_write(int fd, const std::vector<std::vector<std::uint16_t>>& line) {
  if (line.size() != 6) return false;

  std::array<uint16_t, 6> pos{};
  for (int j = 0; j < 6; ++j) {
    if (line[j].empty()) return false;
    pos[j] = line[j][0];
  }
  return feetech_sync_write_positions(fd, pos);
}


// converting TrajElement to servo position


static uint16_t traj_value_to_ticks(double v, int joint_index_1to6) {

  constexpr bool kInputIsRadians = false;
 
  // bad input
  if (!std::isfinite(v)) v = 90.0;

  if (std::abs(v) > 360.0) {
    const long p = std::lround(v);
    return static_cast<uint16_t>(std::clamp<long>(p, 0L, 65535L));
  }

  double deg = v;

  // ONLY ACCEPT 0 TO 180 DEGREE ANGLES
  if (deg < 0.0 || deg > 180.0) {
    std::fprintf(stderr,
                 "traj_value_to_ticks: joint %d input %.3f deg out of [0,180]; clamping\n",
                 joint_index_1to6, deg);
  }
  deg = std::clamp(deg, 0.0, 180.0);

  const uint16_t deg_centi =
      static_cast<uint16_t>(std::clamp<long>(std::lround(deg * 100.0), 0L, 18000L));

  const std::vector<uint16_t> mapped = robot_mapping(joint_index_1to6, std::vector<uint16_t>{deg_centi});
  return (!mapped.empty()) ? mapped[0] : deg_centi;
}


static bool goals_from_traj_element(const TrajElement& e, std::array<uint16_t, 6>* out_goals) {
  if (!out_goals) return false;
  if (e.val.size() < 6) return false;

  for (int j = 0; j < 6; ++j) {
    (*out_goals)[j] = traj_value_to_ticks(e.val[(size_t)j], /*joint_index_1to6=*/(j + 1));
  }
  return true;
}

// poll until either all servos are within tolerance or hit the deadline
// out_comms_dead is set true if no servos respond for several seconds
static bool wait_until_positions_reached_until(int fd,
                                               const std::array<uint8_t, 6>& ids,
                                               const std::array<uint16_t, 6>& goals,
                                               std::chrono::steady_clock::time_point deadline,
                                               bool* out_comms_dead) {
  if (out_comms_dead) *out_comms_dead = false;

  constexpr int kPosTolTicks = 100;
  constexpr int kPollPeriodMs = 15;
  constexpr int kReadTimeoutMs = 25;
  constexpr int kStablePollsReq = 2;

  std::array<bool, 6> reached{};
  reached.fill(false);

  int stable = 0;
  int consecutive_all_read_fail = 0;

  while (std::chrono::steady_clock::now() < deadline) {
    bool all_ok = true;
    bool any_read_ok = false;

    for (int j = 0; j < 6; ++j) {
      if (reached[j]) continue;

      const uint8_t id = ids[j];
      const uint16_t goal = goals[j];

      SO101Bus::ServoStateBasic st{};
      bool ok = false;
      for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
        if (SO101Bus::feetech_read_state_basic(fd, id, &st, kReadTimeoutMs)) ok = true;
        else ::usleep(2000);
      }

      if (!ok) {
        all_ok = false;
        continue;
      }
      any_read_ok = true;

      const int diff = std::abs((int)st.present_position - (int)goal);
      if (diff <= kPosTolTicks) reached[j] = true;
      else all_ok = false;
    }

    if (all_ok) {
      if (++stable >= kStablePollsReq) return true;
    } else {
      stable = 0;
    }

    if (!any_read_ok) {
      if (++consecutive_all_read_fail >= (3000 / kPollPeriodMs)) {
        if (out_comms_dead) *out_comms_dead = true;
        std::fprintf(stderr, "wait_until_positions_reached_until: no servos responding; aborting.\n");
        return false;
      }
    } else {
      consecutive_all_read_fail = 0;
    }

    ::usleep(kPollPeriodMs * 1000);
  }

  return false;
}


bool SO101Bus::ping_all(int fd, const std::array<uint8_t, 6>& ids, int timeout_ms) {
  for (uint8_t id : ids) {
    if (!SO101Bus::feetech_ping(fd, id, timeout_ms)) return false;
  }
  return true;
}

// trajectory execution

// overload for execution using a defined config
bool SO101Bus::execute_traj_full(const std::deque<TrajElement>& traj, const SO101Bus::Config& cfg) {
  SO101Bus tmp(cfg);
  return tmp.execute_traj_full(traj);
}

// execute a trajectory expressed as TrajElement waypoints
// send position only sync writes
// TrajElement.time field is used only for pacing and scheduling
// do not write any timeor speed registers
bool SO101Bus::execute_traj_full(const std::deque<TrajElement>& traj) {
  if (traj.empty()) return true;

  if (!ensure_connected_()) return false;

  const int fd = port_.fd();
  const auto& ids = cfg_.ids;

  // align t=0 with the first waypoint timestamp
  const double t0 = traj.front().time;
  const auto start = std::chrono::steady_clock::now();

  for (size_t i = 0; i < traj.size(); ++i) {
    const TrajElement& e = traj[i];

    // sleep until the scheduled send time
    double rel_s = e.time - t0;
    if (rel_s < 0.0) rel_s = 0.0;
    const auto send_tp = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>(rel_s));
    std::this_thread::sleep_until(send_tp);

    std::array<uint16_t, 6> goals{};
    if (!goals_from_traj_element(e, &goals)) {
      std::fprintf(stderr, "execute_traj_full(deque): waypoint %zu has val.size() < 6\n", i);
      return false;
    }

    if (!feetech_sync_write_positions(fd, goals)) {
      std::fprintf(stderr, "execute_traj_full(deque): waypoint %zu sync write failed\n", i);
      return false;
    }

    // poll during the time until the next waypoint
    // if goal missed by the next send time continue unless comms dead
    std::chrono::steady_clock::time_point deadline{};
    if (i + 1 < traj.size()) {
      double next_rel_s = traj[i + 1].time - t0;
      if (next_rel_s < rel_s) next_rel_s = rel_s;
      deadline = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(next_rel_s));
    } else {
      deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(cfg_.final_settle_ms);
    }

    if (deadline > std::chrono::steady_clock::now()) {
      bool comms_dead = false;
      const bool reached = wait_until_positions_reached_until(fd, ids, goals, deadline, &comms_dead);
      if (comms_dead) return false;
      if (!reached && (i + 1 < traj.size())) {
        std::fprintf(stderr,
                     "execute_traj_full(deque): waypoint %zu not reached before next waypoint; continuing \n",
                     i);
      }
    }
  }

  return true;
}
