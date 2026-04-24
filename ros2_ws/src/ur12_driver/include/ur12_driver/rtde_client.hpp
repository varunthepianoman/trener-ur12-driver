#pragma once

#include <arpa/inet.h>
#include <endian.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

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

  explicit RtdeClient(std::string host = "localhost", double frequency = kDefaultFreq)
  : host_(std::move(host)), frequency_(frequency), fd_(-1), running_(false) {}

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

  RobotState get_state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

private:
  std::string host_;
  double      frequency_;
  int         fd_;

  std::atomic<bool>  running_;
  mutable std::mutex mutex_;
  RobotState         state_;
  std::thread        thread_;
  uint8_t            recipe_id_{0};

  // ------------------------------------------------------------------
  // Handshake steps
  // ------------------------------------------------------------------

  bool connect_socket()
  {
    struct addrinfo hints{}, * res = nullptr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host_.c_str(), std::to_string(kPort).c_str(), &hints, &res) != 0) {
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
    fprintf(stderr, "[RtdeClient] VERSION reply: type=%d data_len=%zu\n",
      type, data.size());
    if (type != TYPE_VERSION || data.empty()) return false;
    bool accepted = data[0] != 0;
    fprintf(stderr, "[RtdeClient] Protocol v%d %s\n",
      kProtocolVersion, accepted ? "accepted" : "rejected");
    return accepted;
  }

  static void hex_dump(const char * label, const uint8_t * buf, size_t len)
  {
    fprintf(stderr, "[RtdeClient] %s (%zu bytes):", label, len);
    for (size_t i = 0; i < len; ++i) {
      if (i % 16 == 0) fprintf(stderr, "\n  ");
      fprintf(stderr, "%02X ", buf[i]);
    }
    fprintf(stderr, "\n");
  }

  bool setup_outputs()
  {
    static const char * vars = "actual_q,actual_qd";
    fprintf(stderr, "[RtdeClient] Requesting variables: \"%s\"\n", vars);

    // Protocol v2 request payload: [frequency f64 BE][variable names string]
    // (recipe_id is server-assigned and returned in the response — NOT sent)
    uint64_t freq_bits;
    std::memcpy(&freq_bits, &frequency_, 8);
    freq_bits = htobe64(freq_bits);

    std::vector<uint8_t> payload(8 + strlen(vars));
    std::memcpy(payload.data(), &freq_bits, 8);
    std::memcpy(payload.data() + 8, vars, strlen(vars));

    // Log full outgoing packet
    {
      uint16_t size = htons(static_cast<uint16_t>(3 + payload.size()));
      std::vector<uint8_t> pkt(3 + payload.size());
      std::memcpy(pkt.data(), &size, 2);
      pkt[2] = TYPE_SETUP_OUT;
      std::memcpy(pkt.data() + 3, payload.data(), payload.size());
      hex_dump("SETUP_OUT send", pkt.data(), pkt.size());
    }

    send_packet(TYPE_SETUP_OUT, payload.data(), payload.size());

    auto [type, data] = recv_packet();
    fprintf(stderr, "[RtdeClient] SETUP_OUT reply: type=%d data_len=%zu\n",
      type, data.size());
    hex_dump("SETUP_OUT recv", data.data(), data.size());
    if (type != TYPE_SETUP_OUT || data.empty()) return false;

    recipe_id_ = data[0];
    std::string types(data.begin() + 1, data.end());
    fprintf(stderr, "[RtdeClient] Recipe id=%d types=\"%s\"\n",
      recipe_id_, types.c_str());

    // Fail only if ALL variables are NOT_FOUND (total handshake failure)
    bool any_found = types.find("NOT_FOUND") == std::string::npos ||
                     types != std::string(types.size(), 'N');  // at least one non-NOT_FOUND
    // Simpler: fail only if the entire response is NOT_FOUND with no valid type
    return types.find("VECTOR6D") != std::string::npos ||
           types.find("DOUBLE")   != std::string::npos ||
           types.find("INT32")    != std::string::npos ||
           types.find("UINT32")   != std::string::npos;
  }

  bool send_start()
  {
    send_packet(TYPE_START, nullptr, 0);
    auto [type, data] = recv_packet();
    fprintf(stderr, "[RtdeClient] START reply: type=%d data_len=%zu\n",
      type, data.size());
    if (type != TYPE_START || data.empty()) return false;
    bool accepted = data[0] != 0;
    fprintf(stderr, "[RtdeClient] START %s\n", accepted ? "accepted" : "rejected");
    return accepted;
  }

  // ------------------------------------------------------------------
  // Background thread: keep trying to connect, then stream data
  // ------------------------------------------------------------------

  void connect_loop()
  {
    int attempt = 0;
    while (running_) {
      ++attempt;
      fprintf(stderr, "[RtdeClient] Handshake attempt %d\n", attempt);

      if (fd_ >= 0) { ::close(fd_); fd_ = -1; }

      if (!connect_socket()) {
        fprintf(stderr, "[RtdeClient] TCP connect failed — retrying in 2s\n");
        sleep(2); continue;
      }
      if (!request_protocol_version()) {
        fprintf(stderr, "[RtdeClient] Protocol version failed — retrying in 2s\n");
        sleep(2); continue;
      }
      if (!setup_outputs()) {
        fprintf(stderr, "[RtdeClient] Setup outputs failed (robot may be powered off) — retrying in 2s\n");
        sleep(2); continue;
      }
      if (!send_start()) {
        fprintf(stderr, "[RtdeClient] Start failed — retrying in 2s\n");
        sleep(2); continue;
      }

      fprintf(stderr, "[RtdeClient] Handshake successful — streaming joint states\n");

      // Stream until connection drops, then reconnect
      while (running_) {
        auto [type, data] = recv_packet();
        if (type == 0) {
          fprintf(stderr, "[RtdeClient] Connection lost — reconnecting\n");
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
