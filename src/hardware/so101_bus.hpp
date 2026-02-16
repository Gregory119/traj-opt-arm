//#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

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

  // servo position unit ranges
  struct ServoPosRange {
    int pos_min;
    int pos_max;
  };

  inline static constexpr std::array<ServoPosRange, 7> kPosRangeById{{
      {0, 0},  // unused id 0
      {2618, 5146}, // id 1
      {3372, 5698},  // id 2
      {2557,  650},// id 3 
      {0,    1017}, // id 4
      {5746, 1942},// id 5
      {3469,  834}, //
  }};

  // configuration for port object
  struct Config {
    std::string device{"/dev/ttyACM0"};
    std::array<uint8_t, 6> ids{{1, 2, 3, 4, 5, 6}};
    int ping_timeout_ms{25};
    int step_budget_ms{600};
    int final_settle_ms{1500};
  };

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
  [[nodiscard]] int  fd() const noexcept { return port_.fd(); }

  // ping configured servo ids on the connected port
  bool ping_all();

  // trajectory execution : position only no time (register0x2B) or speed (register0x2C) write
  bool execute_traj_full(const std::deque<TrajElement>& traj);

  // overload execute using a defined config 
  bool execute_traj_full(const std::deque<TrajElement>& traj, const Config& cfg);

  // feetech operations on an open file descriptor
  static int  open_port_1Mbps(const char* path);
  static bool feetech_ping(int fd, uint8_t id, int timeout_ms);
  static bool feetech_write_byte(int fd, uint8_t id, uint8_t address, uint8_t value, int timeout_ms);
  static bool feetech_write_bytes(int fd, uint8_t id, uint8_t start_address, const uint8_t* data, size_t data_len,
                                  int timeout_ms, uint8_t instruc_code, uint8_t* out_error);
  static bool feetech_read_bytes(int fd, uint8_t id, uint8_t start_address, uint8_t* out, size_t out_len,
                                 int timeout_ms, uint8_t* out_error);
  static bool feetech_read_state_basic(int fd, uint8_t id, ServoStateBasic* out, int timeout_ms);

  // position only sync write reads the first uint16_t in each cell as position
  static bool feetech_sync_write(int fd, const std::vector<std::vector<std::uint16_t>>& line);

private:
  bool ensure_connected_();
  static bool ping_all(int fd, const std::array<uint8_t, 6>& ids, int timeout_ms);

  Config cfg_{};
  Port   port_{};
};
