#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <thread>
#include <vector>

#include "ur12_driver/rtde_client.hpp"

namespace
{

// ---------------------------------------------------------------------------
// Helpers for building raw RTDE packets in tests
// ---------------------------------------------------------------------------

static std::vector<uint8_t> make_rtde_packet(uint8_t type, const std::vector<uint8_t> & payload)
{
  uint16_t size = htons(static_cast<uint16_t>(3 + payload.size()));
  std::vector<uint8_t> pkt(3 + payload.size());
  std::memcpy(pkt.data(), &size, 2);
  pkt[2] = type;
  std::memcpy(pkt.data() + 3, payload.data(), payload.size());
  return pkt;
}

static std::vector<uint8_t> double_to_be_bytes(double val)
{
  uint64_t bits;
  std::memcpy(&bits, &val, 8);
  bits = htobe64(bits);
  std::vector<uint8_t> out(8);
  std::memcpy(out.data(), &bits, 8);
  return out;
}

// ---------------------------------------------------------------------------
// Minimal mock RTDE server for handshake testing
// ---------------------------------------------------------------------------
class MockRtdeServer
{
public:
  MockRtdeServer()
  {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = 0;
    ::bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    ::listen(server_fd_, 1);

    socklen_t len = sizeof(addr);
    ::getsockname(server_fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);

    struct timeval tv{3, 0};
    setsockopt(server_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  ~MockRtdeServer()
  {
    if (server_fd_ >= 0) { ::close(server_fd_); server_fd_ = -1; }
    if (thread_.joinable()) thread_.join();
  }

  void run_async(std::function<void(int)> handler)
  {
    thread_ = std::thread([this, handler]() {
      int client_fd = ::accept(server_fd_, nullptr, nullptr);
      if (client_fd < 0) return;
      struct timeval tv{3, 0};
      setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      handler(client_fd);
      ::close(client_fd);
    });
  }

  uint16_t port() const { return port_; }

  static void send_pkt(int fd, const std::vector<uint8_t> & pkt)
  {
    ::send(fd, pkt.data(), pkt.size(), MSG_NOSIGNAL);
  }

  // Receive exactly n bytes
  static bool recv_exact(int fd, uint8_t * buf, size_t n)
  {
    size_t got = 0;
    while (got < n) {
      ssize_t r = ::recv(fd, buf + got, n - got, 0);
      if (r <= 0) return false;
      got += r;
    }
    return true;
  }

  // Receive one RTDE packet, return {type, payload}
  static std::pair<uint8_t, std::vector<uint8_t>> recv_pkt(int fd)
  {
    uint8_t hdr[3];
    if (!recv_exact(fd, hdr, 3)) return {0, {}};
    uint16_t size; std::memcpy(&size, hdr, 2); size = ntohs(size);
    uint8_t type = hdr[2];
    size_t plen  = size > 3 ? size - 3 : 0;
    std::vector<uint8_t> payload(plen);
    if (plen > 0 && !recv_exact(fd, payload.data(), plen)) return {0, {}};
    return {type, payload};
  }

private:
  int      server_fd_{-1};
  uint16_t port_{0};
  std::thread thread_;
};

}  // namespace

using ur12_driver::RtdeClient;
using ur12_driver::RobotState;

// ---------------------------------------------------------------------------
// parse_data_packet tests — no network, pure byte parsing
// ---------------------------------------------------------------------------

TEST(RtdeClientTest, ParseDataPacketExtractsPositions)
{
  const std::array<double, 6> expected_pos = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const std::array<double, 6> expected_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::vector<uint8_t> data;
  data.push_back(0x01);  // recipe_id
  for (double v : expected_pos) {
    auto b = double_to_be_bytes(v);
    data.insert(data.end(), b.begin(), b.end());
  }
  for (double v : expected_vel) {
    auto b = double_to_be_bytes(v);
    data.insert(data.end(), b.begin(), b.end());
  }

  auto state = RtdeClient::parse_data_packet(data);

  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(state.joint_positions[i], expected_pos[i], 1e-12)
      << "position[" << i << "] mismatch";
  }
}

TEST(RtdeClientTest, ParseDataPacketExtractsVelocities)
{
  const std::array<double, 6> pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 6> expected_vel = {-0.5, 1.0, 2.5, -1.2, 0.3, 0.7};

  std::vector<uint8_t> data;
  data.push_back(0x01);
  for (double v : pos) {
    auto b = double_to_be_bytes(v);
    data.insert(data.end(), b.begin(), b.end());
  }
  for (double v : expected_vel) {
    auto b = double_to_be_bytes(v);
    data.insert(data.end(), b.begin(), b.end());
  }

  auto state = RtdeClient::parse_data_packet(data);

  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(state.joint_velocities[i], expected_vel[i], 1e-12)
      << "velocity[" << i << "] mismatch";
  }
}

TEST(RtdeClientTest, ParseDataPacketHomePosition)
{
  // Home pose [0, -1.5707, 1.5707, 0, 0, 0] — a real-world sanity check
  const std::array<double, 6> home = {0.0, -1.5707, 1.5707, 0.0, 0.0, 0.0};

  std::vector<uint8_t> data;
  data.push_back(0x02);  // different recipe_id — should still parse correctly
  for (double v : home) {
    auto b = double_to_be_bytes(v);
    data.insert(data.end(), b.begin(), b.end());
  }
  for (int i = 0; i < 6; ++i) {
    auto b = double_to_be_bytes(0.0);
    data.insert(data.end(), b.begin(), b.end());
  }

  auto state = RtdeClient::parse_data_packet(data);

  EXPECT_NEAR(state.joint_positions[0], 0.0,     1e-12);
  EXPECT_NEAR(state.joint_positions[1], -1.5707, 1e-4);
  EXPECT_NEAR(state.joint_positions[2],  1.5707, 1e-4);
}

TEST(RtdeClientTest, ParseDataPacketTooShortReturnsZeros)
{
  // Payload shorter than expected — should return default-initialised state
  std::vector<uint8_t> data(5, 0x00);  // way too short
  auto state = RtdeClient::parse_data_packet(data);

  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(state.joint_positions[i],  0.0);
    EXPECT_DOUBLE_EQ(state.joint_velocities[i], 0.0);
  }
}

// ---------------------------------------------------------------------------
// build_setup_outputs_payload tests — verify protocol v2 wire format
// ---------------------------------------------------------------------------

TEST(RtdeClientTest, SetupOutputsPayloadHasNoRecipeId)
{
  // Protocol v2: request = [freq f64 BE][var names] — no recipe_id prefix
  constexpr double kFreq = 125.0;
  const char * vars = "actual_q,actual_qd";

  auto payload = RtdeClient::build_setup_outputs_payload(kFreq, vars);

  // First 8 bytes = frequency as big-endian double
  ASSERT_GE(payload.size(), 8u);
  uint64_t freq_be;
  std::memcpy(&freq_be, payload.data(), 8);
  freq_be = be64toh(freq_be);
  double freq;
  std::memcpy(&freq, &freq_be, 8);
  EXPECT_NEAR(freq, kFreq, 1e-9);

  // Remaining bytes = variable name string
  std::string var_str(payload.begin() + 8, payload.end());
  EXPECT_EQ(var_str, std::string(vars));
}

TEST(RtdeClientTest, SetupOutputsPayloadSize)
{
  const char * vars = "actual_q,actual_qd";
  auto payload = RtdeClient::build_setup_outputs_payload(125.0, vars);

  // Must be exactly 8 (freq) + len(vars) — no extra recipe_id byte
  EXPECT_EQ(payload.size(), 8u + std::strlen(vars));
}

// ---------------------------------------------------------------------------
// Handshake test — mock server exchanges VERSION + SETUP_OUTPUTS + START
// ---------------------------------------------------------------------------

TEST(RtdeClientTest, HandshakeSucceedsWithCompliantServer)
{
  MockRtdeServer server;

  server.run_async([](int fd) {
    // 1. Receive VERSION request, reply accepted
    auto [vtype, vpayload] = MockRtdeServer::recv_pkt(fd);
    EXPECT_EQ(vtype, 86u);  // TYPE_VERSION
    std::vector<uint8_t> v_reply = {1};  // accepted
    MockRtdeServer::send_pkt(fd, make_rtde_packet(86, v_reply));

    // 2. Receive SETUP_OUTPUTS, reply with recipe_id=1 + "VECTOR6D,VECTOR6D"
    auto [stype, spayload] = MockRtdeServer::recv_pkt(fd);
    EXPECT_EQ(stype, 79u);  // TYPE_SETUP_OUT
    // First 8 bytes of payload = frequency — no recipe_id prefix
    ASSERT_GE(spayload.size(), 8u);
    uint64_t freq_be; std::memcpy(&freq_be, spayload.data(), 8);
    freq_be = be64toh(freq_be); double freq;
    std::memcpy(&freq, &freq_be, 8);
    EXPECT_NEAR(freq, 125.0, 1e-9);  // verify correct frequency
    // Reply: recipe_id=1 + variable types
    std::string types = "VECTOR6D,VECTOR6D";
    std::vector<uint8_t> s_reply;
    s_reply.push_back(1);  // recipe_id
    s_reply.insert(s_reply.end(), types.begin(), types.end());
    MockRtdeServer::send_pkt(fd, make_rtde_packet(79, s_reply));

    // 3. Receive START, reply accepted
    auto [sttype, stpayload] = MockRtdeServer::recv_pkt(fd);
    EXPECT_EQ(sttype, 83u);  // TYPE_START
    std::vector<uint8_t> st_reply = {1};  // accepted
    MockRtdeServer::send_pkt(fd, make_rtde_packet(83, st_reply));

    // 4. Send one DATA_PACKAGE then close
    const std::array<double, 6> pos = {0.1, -1.5707, 1.5707, 0.0, 0.0, 0.0};
    const std::array<double, 6> vel = {0.0,  0.0,    0.0,    0.0, 0.0, 0.0};
    std::vector<uint8_t> data_payload;
    data_payload.push_back(1);  // recipe_id
    for (auto v : pos) {
      auto b = double_to_be_bytes(v); data_payload.insert(data_payload.end(), b.begin(), b.end());
    }
    for (auto v : vel) {
      auto b = double_to_be_bytes(v); data_payload.insert(data_payload.end(), b.begin(), b.end());
    }
    MockRtdeServer::send_pkt(fd, make_rtde_packet(85, data_payload));
    // Server closes after sending — client will reconnect but data is already latched
  });

  RtdeClient client("127.0.0.1", 125.0, server.port());
  client.start();

  // Poll get_state() until joint_positions[0] is non-zero (data received)
  // or 3s timeout. This avoids racing on is_connected().
  RobotState state{};
  for (int i = 0; i < 30; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    state = client.get_state();
    if (std::abs(state.joint_positions[0]) > 1e-9) break;
  }

  EXPECT_NEAR(state.joint_positions[0],  0.1,    1e-9);
  EXPECT_NEAR(state.joint_positions[1], -1.5707, 1e-4);
  EXPECT_NEAR(state.joint_positions[2],  1.5707, 1e-4);

  client.stop();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
