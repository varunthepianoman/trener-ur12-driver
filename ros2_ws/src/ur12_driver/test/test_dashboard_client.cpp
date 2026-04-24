#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <functional>
#include <string>
#include <thread>

#include "ur12_driver/dashboard_client.hpp"

namespace
{

// ---------------------------------------------------------------------------
// Minimal single-connection mock TCP server for unit testing DashboardClient
// ---------------------------------------------------------------------------
class MockDashboardServer
{
public:
  MockDashboardServer()
  {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    EXPECT_GE(server_fd_, 0);
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = 0;  // OS picks a free port
    EXPECT_EQ(::bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
    EXPECT_EQ(::listen(server_fd_, 1), 0);

    socklen_t len = sizeof(addr);
    ::getsockname(server_fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);

    struct timeval tv{3, 0};
    setsockopt(server_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  ~MockDashboardServer()
  {
    if (server_fd_ >= 0) { ::close(server_fd_); server_fd_ = -1; }
    if (thread_.joinable()) thread_.join();
  }

  // Accept one connection and run handler in a background thread
  void run_async(std::function<void(int)> handler)
  {
    thread_ = std::thread([this, handler]() {
      int client_fd = ::accept(server_fd_, nullptr, nullptr);
      if (client_fd < 0) return;
      struct timeval tv{3, 0};
      setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
      handler(client_fd);
      ::close(client_fd);
    });
  }

  uint16_t port() const { return port_; }

  static void send_str(int fd, const std::string & s)
  {
    ::send(fd, s.c_str(), s.size(), MSG_NOSIGNAL);
  }

  static std::string recv_line(int fd)
  {
    std::string buf;
    char c = 0;
    while (c != '\n') {
      ssize_t n = ::recv(fd, &c, 1, 0);
      if (n <= 0) break;
      buf += c;
    }
    return buf;
  }

private:
  int      server_fd_{-1};
  uint16_t port_{0};
  std::thread thread_;
};

}  // namespace

using ur12_driver::DashboardClient;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(DashboardClientTest, PowerOnSendsCorrectCommandAndParsesSuccess)
{
  MockDashboardServer server;
  std::string received;

  server.run_async([&received](int fd) {
    MockDashboardServer::send_str(fd, "Connected: Universal Robots Dashboard Server\n");
    received = MockDashboardServer::recv_line(fd);
    MockDashboardServer::send_str(fd, "Powering on\n");
    // consume quit on disconnect
    MockDashboardServer::recv_line(fd);
  });

  DashboardClient client("127.0.0.1", server.port());
  auto [ok, msg] = client.power_on();

  EXPECT_TRUE(ok);
  EXPECT_EQ(msg, "Powering on");
  EXPECT_EQ(received, "power on\n");
}

TEST(DashboardClientTest, BrakeReleaseSendsCorrectCommand)
{
  MockDashboardServer server;
  std::string received;

  server.run_async([&received](int fd) {
    MockDashboardServer::send_str(fd, "Connected: Universal Robots Dashboard Server\n");
    received = MockDashboardServer::recv_line(fd);
    MockDashboardServer::send_str(fd, "Brake releasing\n");
    MockDashboardServer::recv_line(fd);
  });

  DashboardClient client("127.0.0.1", server.port());
  auto [ok, msg] = client.brake_release();

  EXPECT_TRUE(ok);
  EXPECT_EQ(msg, "Brake releasing");
  EXPECT_EQ(received, "brake release\n");
}

TEST(DashboardClientTest, FailedResponseReturnsFalse)
{
  MockDashboardServer server;

  server.run_async([](int fd) {
    MockDashboardServer::send_str(fd, "Connected: Universal Robots Dashboard Server\n");
    MockDashboardServer::recv_line(fd);  // consume command
    MockDashboardServer::send_str(fd, "Failed: power on not possible\n");
    MockDashboardServer::recv_line(fd);
  });

  DashboardClient client("127.0.0.1", server.port());
  auto [ok, msg] = client.power_on();

  EXPECT_FALSE(ok);
  EXPECT_NE(msg.find("Failed"), std::string::npos);
}

TEST(DashboardClientTest, ErrorResponseReturnsFalse)
{
  MockDashboardServer server;

  server.run_async([](int fd) {
    MockDashboardServer::send_str(fd, "Connected: Universal Robots Dashboard Server\n");
    MockDashboardServer::recv_line(fd);
    MockDashboardServer::send_str(fd, "error: not connected to robot\n");
    MockDashboardServer::recv_line(fd);
  });

  DashboardClient client("127.0.0.1", server.port());
  auto [ok, msg] = client.play();

  EXPECT_FALSE(ok);
}

TEST(DashboardClientTest, ConnectionRefusedReturnsFalse)
{
  // No server listening — connect() should fail
  DashboardClient client("127.0.0.1", 19998);  // unlikely to be in use
  auto [ok, msg] = client.power_on();

  EXPECT_FALSE(ok);
  EXPECT_FALSE(msg.empty());
}

TEST(DashboardClientTest, MultipleCommandsOnSameConnection)
{
  MockDashboardServer server;

  server.run_async([](int fd) {
    MockDashboardServer::send_str(fd, "Connected: Universal Robots Dashboard Server\n");
    // power on
    MockDashboardServer::recv_line(fd);
    MockDashboardServer::send_str(fd, "Powering on\n");
    // brake release
    MockDashboardServer::recv_line(fd);
    MockDashboardServer::send_str(fd, "Brake releasing\n");
    MockDashboardServer::recv_line(fd);
  });

  DashboardClient client("127.0.0.1", server.port());
  auto [ok1, msg1] = client.power_on();
  auto [ok2, msg2] = client.brake_release();

  EXPECT_TRUE(ok1);
  EXPECT_EQ(msg1, "Powering on");
  EXPECT_TRUE(ok2);
  EXPECT_EQ(msg2, "Brake releasing");
}

TEST(DashboardClientTest, PlayPauseStopSendCorrectCommands)
{
  // Verify the command strings for play / pause / stop
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"play\n",  "Program running"},
    {"pause\n", "Pausing program"},
    {"stop\n",  "Stopped"},
  };

  for (const auto & [expected_cmd, server_reply] : cases) {
    MockDashboardServer server;
    std::string received;

    server.run_async([&received, &server_reply](int fd) {
      MockDashboardServer::send_str(fd, "Connected: Universal Robots Dashboard Server\n");
      received = MockDashboardServer::recv_line(fd);
      MockDashboardServer::send_str(fd, server_reply + "\n");
      MockDashboardServer::recv_line(fd);
    });

    DashboardClient client("127.0.0.1", server.port());
    std::pair<bool, std::string> result;
    if (expected_cmd == "play\n")  result = client.play();
    if (expected_cmd == "pause\n") result = client.pause();
    if (expected_cmd == "stop\n")  result = client.stop();

    EXPECT_EQ(received, expected_cmd) << "Wrong command for " << expected_cmd;
    EXPECT_TRUE(result.first);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
