#include "so101_bus.hpp"

#include <cassert>
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

using ReplyPacket = SO101Bus::ReplyPacket;

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
[[nodiscard]] static bool write_all(int fd, const uint8_t* buf, size_t n) {

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



static inline uint8_t lo(uint16_t v) { return static_cast<uint8_t>(v & 0xFF); } // return lowest 8 bits of 16 bit int
static inline uint8_t hi(uint16_t v) { return static_cast<uint8_t>((v >> 8) & 0xFF); } //  return highest 8 bits of 16 bit int

} //end of namespace


// SO101Bus class handling lifecycle / connection

SO101Bus::SO101Bus(Config cfg)
    : cfg_(std::move(cfg))
{
    assert(cfg_.calibration.size() == cfg_.ids.size());
}

bool SO101Bus::connect() {
  // re open using cfg_.device
  port_.close();
    if (!port_.open(cfg_.device)) {
    std::fprintf(stderr, "cant open %s: %s\n", cfg_.device.c_str(), std::strerror(errno));
    return false;
  }

  // ping all servos using the file descriptor, set of servo ids, and allowed time before triggering timeout
  // return false if one doesn't reply
  if (!SO101Bus::ping_all()) {
    std::fprintf(stderr, "one or more servos did not reply to ping\n");
    port_.close();
    return false;
  }

  if (!write_gains()) {
    return false;
  }

  return true;
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
  for (uint8_t id : cfg_.ids) {
    if (!feetech_ping(id, cfg_.rw_timeout_ms)) return false;
  }
  return true;
}


// read bytes from the port fd until the timeout expires then scan for a valid servo reply frame
// looks for feetech formatted packet skipping bytes until header is found
// accepts only frames if ID matches expected_id
// waits until full frame is available based on LEN
// given a valid frame, fills reply and returns true only if ERR == 0x00
// returns false on timeout or no valid frame
// also returns false on checksum,length, or parsing failure, mismatched ID, or if the servo reports a nonzero error status byte


bool SO101Bus::read_reply(
                uint8_t expected_id,
                int timeout_ms,
                ReplyPacket& reply){

    const int fd = port_.fd();
    // format: FF FF ID LEN ERR [params] CHK
    uint8_t r[256]; //initialize an array of 256 bytes
    int got = 0;
    const int header_size = 5;

    // sanity check
    if (timeout_ms < 1) {
    std::cout
        << "ERROR: SO101Bus::read_reply() - timeout too small. servo id = "
        << static_cast<int>(expected_id) << std::endl;
    return false;
    }

    // read the header
    while (got < header_size) {
        // return number of bytes read
        ssize_t k = read_with_timeout(fd, r + got, sizeof(r) - got, timeout_ms);
        if (k <= 0) {
        std::cout << "ERROR: SO101Bus::read_reply(). Failed to read "
                     "header. Either timeout, file descriptor closed, or "
                     "error. servo id = "
                  << static_cast<int>(expected_id) << std::endl;
        return false;
        }

        got += static_cast<int>(k);
    }
    // header is now parsed so fill it in the packet
    std::memcpy(&reply.initial, r, sizeof(reply.initial));
    reply.id = r[2];
    reply.data_length = r[3];
    reply.error_status = r[4];
    const int packet_size = 4+ reply.data_length; //total packet size will always be 4 greater than the LEN parameter

    // id check
    if(reply.id != expected_id){
        std::cout
            << "ERROR: SO101Bus::read_reply(). Replying servo ID does not "
               "match the requested ID or stream misalignment is causing"
               "the wrong byte to be read as ID. Exiting. servo id = "
            << static_cast<int>(expected_id) << std::endl;
        return false;
    }


    while (got < packet_size) {
        // return number of bytes read
        ssize_t k = read_with_timeout(fd, r + got, sizeof(r) - got, timeout_ms);
        if (k <= 0) {
        std::cout << "ERROR: SO101Bus::read_reply(). Failed to read "
                     "data and checksum. Either timeout, file descriptor "
                     "closed, or error. servo id = "
                  << static_cast<int>(expected_id) << std::endl;
        return false;
        }

        got += static_cast<int>(k);
    }

    // fill remaining data into reply packet
    // LEN includes ERR and CHK so we need to remove those bytes to get param length

    const int params_len = (int)reply.data_length - 2;
    reply.parameters.assign(r + header_size,
                            r + header_size + params_len);
    reply.check_sum = r[packet_size -1];



    // checksum check
    
    if (checksum_feetech(&r[2] , reply.data_length+1)
        != reply.check_sum) {
        std::cout << "ERROR: SO101Bus::read_reply(). Checksum match failure. "
                     "servo id = "
                  << static_cast<int>(expected_id) << std::endl;
        return false;
    }

    // check error
    if (reply.error_status != 0) {
        std::cout << "ERROR: SO101Bus::read_reply(). Servo in error status. "
                     "servo id = "
                  << static_cast<int>(expected_id) << std::endl;
        return false;
    }
    return true;
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

bool SO101Bus::feetech_ping(uint8_t id, int timeout_ms) {
    if (!ensure_connected_()) return false;
    const int fd = port_.fd();
    // format FF FF ID 02 01 CHK
    uint8_t body[3] = {id, 0x02, 0x01}; // body is 3 bytes
    uint8_t chk     = checksum_feetech(body, sizeof(body)); //compute checksum
    uint8_t pkt[6]  = {0xFF, 0xFF, id, 0x02, 0x01, chk}; // full packet allocation

    tcflush(fd, TCIFLUSH); // clear old data
    if (!write_all(fd, pkt, sizeof(pkt))) return false; // return false if write fails

    ReplyPacket reply;
    return read_reply(id, timeout_ms,reply); //return true if valid packet is received, 
                                                                                        //false if not
}

bool SO101Bus::feetech_write_byte(uint8_t id, uint8_t address, uint8_t value, int timeout_ms) {
    const int fd = port_.fd();
  return feetech_write_bytes(id, address, &value, 1, timeout_ms, 0x03, nullptr);  //useful for writing a single param byte(like position)
}

bool SO101Bus::feetech_write_bytes(uint8_t id,
                                   uint8_t start_address,
                                   const uint8_t* data,
                                   size_t data_len,
                                   int timeout_ms,
                                   uint8_t instruc_code,
                                   uint8_t* out_error) {
    if (!ensure_connected_()) return false;
    const int fd = port_.fd();
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

    ReplyPacket reply;
    const bool ok = read_reply(id, timeout_ms, reply);
    if (out_error) *out_error = reply.error_status;
    if (!ok) {
      if (reply.initial == 0) errno = ETIMEDOUT; // only if we never parsed a packet
      return false;
    }
    return true;
}

bool SO101Bus::feetech_read_bytes(uint8_t id, uint8_t start_address,
                                  std::vector<uint8_t>& out,
                                  int timeout_ms, uint8_t& out_error) {
    if (!ensure_connected_()) return false;
    const int fd = port_.fd();
    const size_t out_len = out.size();
    //if (out_len > 250) { errno = EINVAL; return false; } // check max length

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

    ReplyPacket reply;
    const bool ok = read_reply(id, timeout_ms, reply);
    out = reply.parameters;
    if (!ok) {
      if (reply.initial == 0) errno = ETIMEDOUT;
      return false;
    }
    if (reply.parameters.size() != out_len) return false;
    return true;
}

bool SO101Bus::feetech_read_state_basic(uint8_t id, ServoStateBasic& out, int timeout_ms) {
    const int fd = port_.fd();

    // position documented at 0x38 2 bytes
    const uint8_t START = 0x38;
    const size_t  N     = 8;

    std::vector<uint8_t> tmp(N);

    uint8_t err = 0xFF; // initialize error byte
    if (!feetech_read_bytes(id, START, tmp, timeout_ms, err)) {
        out.error = err; // write error code to ServoStateBasic object
        return false; // return false when a read fail occurs
    }
    out.error = err; // copy error byte to ServoStateBasic object
    if (N) std::memcpy(out.raw, tmp.data(), N);

    auto to_u16 = [&](int idx) -> uint16_t { // 16 bit unsigned int from combined bytes 
        return (uint16_t)out.raw[idx] | ((uint16_t)out.raw[idx + 1] << 8); // combine low and high bytes into a 16 bit unsinged int
    };
    auto to_s16 = [&](int idx) -> int16_t { 
        return (int16_t)to_u16(idx); //casts unsigned int into signed
    };

    // mapping
    out.present_position     = to_u16(0); // 0x38 to 0x39
    out.present_speed        = to_s16(2);  // 0x3A to 0x3B
    out.present_load         = to_s16(4); // 0x3C to 0x3D
    out.present_voltage_raw  = out.raw[6]; // 0x3E
    out.present_temp_c       = out.raw[7]; // 0x3F

    return true;
}

bool SO101Bus::read_all_states(const int timeout_ms,
                               const PosUnit unit,
                               Eigen::VectorXd &pos,
                               Eigen::VectorXd &vel)
{
    for (int j = 0; j < 6; ++j) {
        ServoStateBasic state;
        if (!feetech_read_state_basic(cfg_.ids[j], state, timeout_ms)) {
            return false;
        }
        pos(j) = cfg_.calibration.ticToPos(state.present_position,
                                           cfg_.ids[j],
                                           unit);
        vel(j)
            = cfg_.calibration.ticVelToVel(state.present_speed, cfg_.ids[j], unit);
    }
    return true;
}

bool SO101Bus::write_gains()
{
    for (int sid : cfg_.ids) {
        const bool ret
            = feetech_write_byte(sid, 0x15, cfg_.p, cfg_.rw_timeout_ms)
              && feetech_write_byte(sid, 0x17, cfg_.i, cfg_.rw_timeout_ms)
              && feetech_write_byte(sid, 0x16, cfg_.d, cfg_.rw_timeout_ms);
        if (!ret) {
            return ret;
        }
    }
    return true;
}

bool SO101Bus::write_all_positions(const std::array<uint16_t, 6>& pos, int timeout_ms) {
  if (!ensure_connected_()) return false;

  // check position is in range and return false if not
  for (int sid : cfg_.ids) {
    if (!cfg_.calibration.inRangeTic(pos[sid - 1], sid)) {
      std::cout
          << "write_all_positions() - target tick position out of range. pos="
          << static_cast<int>(pos[sid - 1]) << ", sid=" << sid << std::endl;
      return false;
    }
  }

  const int fd = port_.fd();
  const auto& ids = cfg_.ids;
  constexpr uint8_t kBroadcastId = 0xFE;
  constexpr uint8_t kInstruction = 0x83;  // sync write
  constexpr uint8_t kGoalPosAddr = 0x2A;  // goal Position (2 bytes)
  constexpr uint8_t kMiniLen     = 0x02; // pos(2)
  constexpr int kSoftMarginUnits = 5;

  if (timeout_ms < 0) {
    std::fprintf(stderr, "timeout_ms set to invalid value of: %d\n ",timeout_ms);
    return false;

  };

  auto send_sync = [&](const std::array<uint16_t, 6>& p) -> bool {
    uint8_t payload[1 + 6 * (1 + 2)]{};
    payload[0] = kMiniLen;

    for (int j = 0; j < 6; ++j) {
      const uint8_t  id = ids[j];
      const uint16_t v  = p[j];
      const int base = 1 + j * 3;
      payload[base + 0] = id;
      payload[base + 1] = lo(v);
      payload[base + 2] = hi(v);
    }

    uint8_t err = 0xFF;
    return this->feetech_write_bytes(kBroadcastId,
                                         kGoalPosAddr,
                                         payload,
                                         sizeof(payload),
                                         timeout_ms,
                                         kInstruction,
                                         &err);
  };

  // send the final goal command
  return send_sync(pos);
}

bool SO101Bus::execute_traj_full(const DiscreteJointStateTraj &traj,
                                 const PosUnit pos_unit,
                                 DiscreteJointStateTraj& meas_traj)
{
  if (traj.empty()) return true;

  if (!ensure_connected_()) return false;

  const int fd = port_.fd();
  const auto& ids = cfg_.ids;

  // align t=0 with the first waypoint timestamp
  const double t0 = traj.front().time;
  const auto start = std::chrono::steady_clock::now();
  double max_sample_time_err_ms = 0.0;

  std::array<uint16_t, 6> last_target_pos_tic{};

  for (size_t i = 0; i < traj.size(); ++i) {
    const JointState& e = traj[i];

    // sleep until the scheduled send time
    double rel_s = e.time - t0;
    if (rel_s < 0.0) rel_s = 0.0;
    const auto send_tp = start + std::chrono::duration<double>(rel_s);
    std::this_thread::sleep_until(send_tp);
    
    if (cfg_.record_timing_stats) {
      const auto now = std::chrono::steady_clock::now();
      const double err_ms =
          std::chrono::duration<double, std::milli>(now - send_tp).count();
      if (err_ms > max_sample_time_err_ms) max_sample_time_err_ms = err_ms;
    }

    // convert target position to tics
    std::array<uint16_t, 6> target_pos_tic{};
    std::cout << "sending target positions: [";
    for (int i{}; i < e.q.size(); ++i) {
      std::cout << e.q(i);
      if (!cfg_.calibration.inRangePos(e.q(i), cfg_.ids[i], pos_unit)) {
            return false;
      }
      switch (pos_unit) {
            case PosUnit::RADIAN:
                target_pos_tic[i] = cfg_.calibration.posToTic(e.q(i),
                                                              cfg_.ids[i],
                                                              PosUnit::RADIAN);
                break;

            case PosUnit::DEGREE:
                target_pos_tic[i] = cfg_.calibration.posToTic(e.q(i),
                                                              cfg_.ids[i],
                                                              PosUnit::DEGREE);
                break;
      }
      std::cout << "(" << target_pos_tic[i] << "), ";
    }
    std::cout << "]" << std::endl;

    if (!write_all_positions(target_pos_tic, cfg_.rw_timeout_ms)) {
      std::fprintf(stderr, "execute_traj_full(deque): waypoint %zu sync write failed\n", i);
      return false;
    }
    last_target_pos_tic = target_pos_tic;

    // use timestamp before all reads
    const std::chrono::duration<double> ts
        = std::chrono::steady_clock::now() - start;

    JointState js{.time = ts.count(),
                  .q = Eigen::VectorXd::Zero(cfg_.ids.size()),
                  .dq = Eigen::VectorXd::Zero(cfg_.ids.size()),
                  .ddq = Eigen::VectorXd::Zero(cfg_.ids.size())};
    if (!read_all_states(cfg_.rw_timeout_ms, pos_unit, js.q, js.dq)) {
      std::fprintf(stderr,
                   "execute_traj_full(deque): read_all_states failed "
                   "after waypoint %zu\n",
                   i);
      return false;
    }
    // approximate acceleration using first order derivative of velocity
    if (!meas_traj.empty()) {
      js.ddq
          = (js.dq - meas_traj.back().dq) / (js.time - meas_traj.back().time);
    }

    meas_traj.push_back(std::move(js));
  }

  if (cfg_.record_timing_stats) {
  std::fprintf(stderr, //print sample error, will convert all print statements to std::cout later
             "max sample time error = %.3f ms\n",
             max_sample_time_err_ms);
  }

  return true;
}
