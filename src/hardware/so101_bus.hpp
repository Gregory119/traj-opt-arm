#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "calibration.hpp"
#include "traj_element.hpp"

// SO101Bus: feetech bus operations adn position only trajectory execution
class SO101Bus {
public:
  // state
  struct ServoStateBasic {
    // parsed fields
    uint8_t raw[8]{};
    uint8_t error{0};
    uint16_t present_position{0};
    int16_t present_speed{0};
    int16_t present_load{0};
    uint8_t present_voltage_raw{0};
    uint8_t present_temp_c{0};
  };

  struct ReplyPacket {
        uint16_t initial;
        uint8_t id;
        uint8_t data_length;
        uint8_t error_status;
        std::vector<uint8_t> parameters;
        uint8_t check_sum;
  };

    static const std::vector<ServoPosRange> sid_to_pos_tic_range;

  // configuration
  struct Config {
    std::string device{"/dev/ttyACM0"};
    std::array<uint8_t, 6> ids{{1, 2, 3, 4, 5, 6}};

    //flags
    bool ping_on_connect{true};
    bool enable_state_poll{false}; 
    bool record_timing_stats{false};

    int read_timeout_ms{1};
  };



  SO101Bus();
  explicit SO101Bus(Config cfg);
  ~SO101Bus() = default;

  SO101Bus(const SO101Bus&) = delete;
  SO101Bus& operator=(const SO101Bus&) = delete;
  SO101Bus(SO101Bus&&) noexcept = default;
  SO101Bus& operator=(SO101Bus&&) noexcept = default;

  // configuration and connection
  void set_config(const Config& cfg);
  [[nodiscard]] const Config& config() const noexcept { return cfg_; }

  bool connect();
  bool connect(const std::string& device);
  void disconnect() noexcept;

  [[nodiscard]] bool is_connected() const noexcept { return port_.is_open(); }
  

  // ping configured servo ids on the connected port
  bool ping_all();

  bool read_reply(uint8_t expected_id,int timeout_ms,ReplyPacket& reply);

    /*
     * @brief Execute a trajectory expressed as TrajElement waypoints.
     *
     * @param unit Unit of the trajectory positions.
     */
    bool execute_traj_full(const std::deque<TrajElement>& traj, const PosUnit pos_unit);

  static int  open_port_1Mbps(const char* path);

  [[nodiscard]] bool feetech_ping(uint8_t id, int timeout_ms);
  [[nodiscard]] bool feetech_write_byte(uint8_t id, uint8_t address, uint8_t value, int timeout_ms);
  [[nodiscard]] bool feetech_write_bytes(uint8_t id, uint8_t start_address, const uint8_t* data, size_t data_len,
                           int timeout_ms, uint8_t instruc_code, uint8_t* out_error);
  [[nodiscard]] bool feetech_read_bytes(uint8_t id, uint8_t start_address, std::vector<uint8_t>& out, int timeout_ms, uint8_t& out_error);
  [[nodiscard]] bool feetech_read_state_basic(uint8_t id, ServoStateBasic* out, int timeout_ms);

    /*
     * This function uses the 'SYNC WRITE' functionality in the message protocol,
     * which sends a single packet containing the write data for all servos
     * instead of one packet per servo.
     *
     * @param pos Target positions. Must be in unit tic.
     */
  [[nodiscard]] bool write_all_positions(const std::array<uint16_t, 6>& pos, int timeout_ms);

  [[nodiscard]] bool read_all_states(std::array<ServoStateBasic, 6>* out, int timeout_ms);

private:
  class Port {
  public:
    Port() = default;
    explicit Port(const char* device);
    explicit Port(const std::string& device);
    ~Port();

    Port(const Port&) = delete;
    Port& operator=(const Port&) = delete;
    Port(Port&& other) noexcept;
    Port& operator=(Port&& other) noexcept;

    bool open(const char* device);
    bool open(const std::string& device);
    void close() noexcept;

    [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }
    [[nodiscard]] int  fd() const noexcept { return fd_; }

  private:
    int fd_{-1};
  };
  bool ensure_connected_();

  Config cfg_{};
  Port   port_{};
    const Calibration calibration_;
};
