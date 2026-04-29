#pragma once

#include <arpa/inet.h>
#include <endian.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ur12_driver
{

/// Minimal RTDE (Real-Time Data Exchange) client — TCP port 30004.
/// Streams actual_q (positions) and actual_qd (velocities) at up to 500 Hz.
///
/// RTDE packet wire format (big-endian):
///   uint16  size    — total byte length including header
///   uint8   type    — packet type code
///   bytes   payload
///
/// Handshake sequence:
///   1. Connect TCP
///   2. REQUEST_PROTOCOL_VERSION (type=86)  → accepted?
///   3. SETUP_OUTPUTS (type=79)             → recipe id + type list
///   4. START (type=83)                     → accepted?
///   5. DATA_PACKAGE (type=85) arrives continuously

struct RobotState
{
  std::array<double, 6> joint_positions{};
  std::array<double, 6> joint_velocities{};
};

class RtdeClient
{
public:
  static constexpr int    kPort            = 30004;
  static constexpr int    kProtocolVersion = 2;
  static constexpr double kDefaultFreq     = 125.0;
  static constexpr int    kMaxRetries      = 10;

  // RTDE type codes
  static constexpr uint8_t TYPE_VERSION   = 86;  // 'V'
  static constexpr uint8_t TYPE_SETUP_OUT = 79;  // 'O'
  static constexpr uint8_t TYPE_START     = 83;  // 'S'
  static constexpr uint8_t TYPE_DATA      = 85;  // 'U'

  explicit RtdeClient(std::string host = "localhost", double frequency = kDefaultFreq,
                      int port = kPort)
  : host_(std::move(host)), frequency_(frequency), port_(port), fd_(-1), running_(false) {}

  ~RtdeClient() { stop(); }

  RtdeClient(const RtdeClient &) = delete;
  RtdeClient & operator=(const RtdeClient &) = delete;

  // ------------------------------------------------------------------
  // Public API
  // ------------------------------------------------------------------

  /// Launch a background thread that retries the RTDE handshake until it
  /// succeeds. Returns immediately — call is_connected() to check status.
  /// actual_q is only available after robot power-on; this design lets the
  /// node start serving dashboard services immediately and RTDE connects
  /// automatically once the user calls power_on + brake_release.
  bool start()
  {
    running_ = true;
    thread_  = std::thread(&RtdeClient::connect_loop, this);
    return true;  // always succeeds — connection happens in background
  }

  void stop()
  {
    running_ = false;
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    if (thread_.joinable()) thread_.join();
  }

  bool is_connected() const { return connected_; }

  RobotState get_state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  // ------------------------------------------------------------------
  // Test helpers (static so tests can exercise parsing without a live socket)
  // ------------------------------------------------------------------

  /// Parse a DATA_PACKAGE payload (recipe_id byte + 6×double pos + 6×double vel, big-endian).
  static RobotState parse_data_packet(const std::vector<uint8_t> & data)
  {
    constexpr size_t kExpected = 1 + 12 * sizeof(double);
    RobotState s{};
    if (data.size() < kExpected) return s;
    size_t off = 1;
    for (int i = 0; i < 6; ++i, off += sizeof(double)) {
      s.joint_positions[i] = read_double_be(data.data() + off);
    }
    for (int i = 0; i < 6; ++i, off += sizeof(double)) {
      s.joint_velocities[i] = read_double_be(data.data() + off);
    }
    return s;
  }

  /// Build a SETUP_OUTPUTS request payload — useful for verifying wire format in tests.
  static std::vector<uint8_t> build_setup_outputs_payload(
    double frequency, const char * vars)
  {
    uint64_t freq_bits;
    std::memcpy(&freq_bits, &frequency, 8);
    freq_bits = htobe64(freq_bits);
    std::vector<uint8_t> payload(8 + std::strlen(vars));
    std::memcpy(payload.data(), &freq_bits, 8);
    std::memcpy(payload.data() + 8, vars, std::strlen(vars));
    return payload;
  }

private:
  std::string host_;
  double      frequency_;
  int         port_;
  int         fd_;

  std::atomic<bool>  running_;
  std::atomic<bool>  connected_{false};
  mutable std::mutex mutex_;
  RobotState         state_;
  std::thread        thread_;
  uint8_t            recipe_id_{0};

  static rclcpp::Logger logger()
  {
    static auto lg = rclcpp::get_logger("RtdeClient");
    return lg;
  }

  // ------------------------------------------------------------------
  // Handshake steps
  // ------------------------------------------------------------------

  bool connect_socket()
  {
    struct addrinfo hints{}, * res = nullptr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0) {
      return false;
    }

    fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd_ < 0) { freeaddrinfo(res); return false; }

    if (::connect(fd_, res->ai_addr, res->ai_addrlen) != 0) {
      freeaddrinfo(res); ::close(fd_); fd_ = -1; return false;
    }
    freeaddrinfo(res);

    // 3-second recv timeout — generous enough for a slow URSim boot
    struct timeval tv{3, 0};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return true;
  }

  bool request_protocol_version()
  {
    uint16_t ver = htons(static_cast<uint16_t>(kProtocolVersion));
    send_packet(TYPE_VERSION, &ver, sizeof(ver));
    auto [type, data] = recv_packet();
    if (type != TYPE_VERSION || data.empty()) return false;
    return data[0] != 0;
  }

  bool setup_outputs()
  {
    static const char * vars = "actual_q,actual_qd";

    // Protocol v2 request payload: [frequency f64 BE][variable names string]
    // (recipe_id is server-assigned and returned in the response — NOT sent)
    uint64_t freq_bits;
    std::memcpy(&freq_bits, &frequency_, 8);
    freq_bits = htobe64(freq_bits);

    std::vector<uint8_t> payload(8 + strlen(vars));
    std::memcpy(payload.data(), &freq_bits, 8);
    std::memcpy(payload.data() + 8, vars, strlen(vars));

    send_packet(TYPE_SETUP_OUT, payload.data(), payload.size());

    auto [type, data] = recv_packet();
    if (type != TYPE_SETUP_OUT || data.empty()) return false;

    recipe_id_ = data[0];
    std::string types(data.begin() + 1, data.end());

    return types.find("VECTOR6D") != std::string::npos ||
           types.find("DOUBLE")   != std::string::npos ||
           types.find("INT32")    != std::string::npos ||
           types.find("UINT32")   != std::string::npos;
  }

  bool send_start()
  {
    send_packet(TYPE_START, nullptr, 0);
    auto [type, data] = recv_packet();
    if (type != TYPE_START || data.empty()) return false;
    return data[0] != 0;
  }

  // ------------------------------------------------------------------
  // Background thread: keep trying to connect, then stream data
  // ------------------------------------------------------------------

  void connect_loop()
  {
    while (running_) {
      if (fd_ >= 0) { ::close(fd_); fd_ = -1; }

      if (!connect_socket() || !request_protocol_version() ||
          !setup_outputs()  || !send_start())
      {
        RCLCPP_WARN(logger(),
          "Handshake failed (robot may be powered off) — retrying in 2s");
        sleep(2); continue;
      }

      RCLCPP_INFO(logger(),
        "Handshake successful — streaming joint states at %.0f Hz", frequency_);
      connected_ = true;

      while (running_) {
        auto [type, data] = recv_packet();
        if (type == 0) {
          RCLCPP_WARN(logger(), "Connection lost — reconnecting");
          connected_ = false;
          break;
        }
        if (type == TYPE_DATA) {
          parse_data(data);
        }
      }
    }
  }

  void parse_data(const std::vector<uint8_t> & data)
  {
    // Layout: uint8 recipe_id | 6×double actual_q | 6×double actual_qd
    constexpr size_t kExpected = 1 + 12 * sizeof(double);
    if (data.size() < kExpected) return;

    RobotState s;
    size_t off = 1;  // skip recipe id byte
    for (int i = 0; i < 6; ++i, off += sizeof(double)) {
      s.joint_positions[i] = read_double_be(data.data() + off);
    }
    for (int i = 0; i < 6; ++i, off += sizeof(double)) {
      s.joint_velocities[i] = read_double_be(data.data() + off);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = s;
  }

  // ------------------------------------------------------------------
  // Wire helpers
  // ------------------------------------------------------------------

  void send_packet(uint8_t type, const void * payload, size_t payload_len)
  {
    uint16_t size = htons(static_cast<uint16_t>(3 + payload_len));
    std::vector<uint8_t> buf(3 + payload_len);
    std::memcpy(buf.data(), &size, 2);
    buf[2] = type;
    if (payload && payload_len > 0) {
      std::memcpy(buf.data() + 3, payload, payload_len);
    }
    // Loop to guarantee all bytes are sent (send() may return short)
    size_t sent = 0;
    while (sent < buf.size()) {
      ssize_t n = ::send(fd_, buf.data() + sent, buf.size() - sent, MSG_NOSIGNAL);
      if (n <= 0) return;
      sent += n;
    }
  }

  std::pair<uint8_t, std::vector<uint8_t>> recv_packet()
  {
    uint8_t header[3];
    if (!recv_exact(header, 3)) return {0, {}};

    uint16_t size;
    std::memcpy(&size, header, 2);
    size = ntohs(size);
    uint8_t type = header[2];

    size_t payload_len = size > 3 ? size - 3 : 0;
    std::vector<uint8_t> payload(payload_len);
    if (payload_len > 0 && !recv_exact(payload.data(), payload_len)) {
      return {0, {}};
    }
    return {type, payload};
  }

  bool recv_exact(uint8_t * buf, size_t n)
  {
    size_t received = 0;
    while (received < n) {
      ssize_t r = ::recv(fd_, buf + received, n - received, 0);
      if (r <= 0) return false;
      received += r;
    }
    return true;
  }

  static double read_double_be(const uint8_t * buf)
  {
    uint64_t tmp;
    std::memcpy(&tmp, buf, 8);
    tmp = be64toh(tmp);
    double val;
    std::memcpy(&val, &tmp, 8);
    return val;
  }
};

}  // namespace ur12_driver
