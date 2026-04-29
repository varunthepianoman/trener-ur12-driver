// Unit tests for ScriptClient.
//
// ScriptClient takes an optional port parameter, so the mock server below
// binds to an OS-assigned port (port 0 → kernel picks one) and that port
// is injected into the client. No conflict with a running URSim on 30002,
// no flaky ordering between tests.
//
// The most important assertion is that move_first_joint and move_joints
// both wrap their call in `def prog(): ... end`, because URSim's secondary
// interface silently drops top-level calls that pass arguments.
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>

#include "ur12_driver/script_client.hpp"

namespace
{

// Spins up a TCP listener on an OS-assigned loopback port, accepts one
// connection, drains it into a string, and stores the result. Caller polls
// captured()/done() to read it back.
class MockSecondary
{
public:
  MockSecondary()
  {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = 0;  // kernel picks a free port
    if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      bind_failed_ = true;
      return;
    }
    ::listen(server_fd_, 1);

    socklen_t len = sizeof(addr);
    ::getsockname(server_fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);

    thread_ = std::thread([this]() {
      int client_fd = ::accept(server_fd_, nullptr, nullptr);
      if (client_fd < 0) return;
      char buf[4096];
      while (true) {
        ssize_t n = ::recv(client_fd, buf, sizeof(buf), 0);
        if (n <= 0) break;
        captured_.append(buf, static_cast<size_t>(n));
      }
      ::close(client_fd);
      done_ = true;
    });
  }

  ~MockSecondary()
  {
    if (server_fd_ >= 0) { ::shutdown(server_fd_, SHUT_RDWR); ::close(server_fd_); }
    if (thread_.joinable()) thread_.join();
  }

  bool        bind_failed() const { return bind_failed_; }
  bool        done()        const { return done_; }
  uint16_t    port()        const { return port_; }
  std::string captured()    const { return captured_; }

  // Wait until the connection has been closed by the client (script sent
  // and TCP ::close() called) — then we know captured_ is complete.
  bool wait_done(int timeout_ms = 1000)
  {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);
    while (!done_ && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return done_;
  }

private:
  int                server_fd_{-1};
  uint16_t           port_{0};
  std::atomic<bool>  done_{false};
  bool               bind_failed_{false};
  std::string        captured_;
  std::thread        thread_;
};

// Writes a temporary script defs file and returns the path. Caller is
// responsible for removing it.
std::string make_temp_script(const std::string & contents)
{
  char path[] = "/tmp/ur12_script_test_XXXXXX";
  int fd = mkstemp(path);
  ::write(fd, contents.data(), contents.size());
  ::close(fd);
  return std::string(path);
}

// Standard defs we use across tests, mirroring commands.script.
constexpr const char * kDefs =
  "def move_home():\n"
  "    movej([0, -1.5707, 1.5707, 0, 0, 0])\n"
  "end\n"
  "def move_first_joint(joint_val=0):\n"
  "    curr_joint = get_actual_joint_positions()\n"
  "    curr_joint[0] = curr_joint[0] + joint_val\n"
  "    movej(curr_joint)\n"
  "end\n";

}  // namespace

using ur12_driver::ScriptClient;

// ---------------------------------------------------------------------------
// Constructor reads the file and throws if absent
// ---------------------------------------------------------------------------

TEST(ScriptClientTest, ConstructorThrowsOnMissingFile)
{
  EXPECT_THROW(
    ScriptClient("127.0.0.1", "/nonexistent/path/that/should/not/exist.script"),
    std::runtime_error);
}

// ---------------------------------------------------------------------------
// move_home — sends defs followed by move_home() invocation
// ---------------------------------------------------------------------------

TEST(ScriptClientTest, MoveHomeSendsDefsPlusInvocation)
{
  MockSecondary server;
  ASSERT_FALSE(server.bind_failed());

  std::string script_path = make_temp_script(kDefs);
  ScriptClient client("127.0.0.1", script_path, server.port());
  EXPECT_TRUE(client.move_home());

  ASSERT_TRUE(server.wait_done());
  std::string sent = server.captured();
  std::remove(script_path.c_str());

  EXPECT_NE(sent.find("def move_home():"),  std::string::npos);
  EXPECT_NE(sent.find("\nmove_home()\n"),   std::string::npos);
}

// ---------------------------------------------------------------------------
// move_first_joint — must wrap call in def prog():...end
// (URSim drops top-level calls with arguments)
// ---------------------------------------------------------------------------

TEST(ScriptClientTest, MoveFirstJointWrapsCallInDefProg)
{
  MockSecondary server;
  ASSERT_FALSE(server.bind_failed());

  std::string script_path = make_temp_script(kDefs);
  ScriptClient client("127.0.0.1", script_path, server.port());
  EXPECT_TRUE(client.move_first_joint(0.75));

  ASSERT_TRUE(server.wait_done());
  std::string sent = server.captured();
  std::remove(script_path.c_str());

  // Must contain the def prog() wrapper, and inside it the invocation.
  // The wrapper is the load-bearing piece — without it URSim silently
  // drops the program.
  EXPECT_NE(sent.find("def prog():"),               std::string::npos);
  EXPECT_NE(sent.find("move_first_joint(0.75"),     std::string::npos);
  EXPECT_NE(sent.find("end\nprog()\n"),             std::string::npos);
}

// ---------------------------------------------------------------------------
// move_joints — inlined movej. The helper-via-commands.script pattern was
// tried and reverted: URSim's secondary interface silently drops scripts
// where a helper is invoked with a list-literal argument. The inlined form
// is what ships.
// ---------------------------------------------------------------------------

TEST(ScriptClientTest, MoveJointsInlinesMovej)
{
  MockSecondary server;
  ASSERT_FALSE(server.bind_failed());

  // Pass an empty defs file — move_joints must NOT depend on commands.script.
  std::string script_path = make_temp_script("");

  ScriptClient client("127.0.0.1", script_path, server.port());
  std::array<double, 6> target = {0.1, -1.5, 1.5, 0.0, 0.0, 0.5};
  EXPECT_TRUE(client.move_joints(target));

  ASSERT_TRUE(server.wait_done());
  std::string sent = server.captured();
  std::remove(script_path.c_str());

  // Wrapper present
  EXPECT_NE(sent.find("def prog():"),       std::string::npos);
  EXPECT_NE(sent.find("end\nprog()\n"),     std::string::npos);
  // movej called directly, not through a user-defined helper
  EXPECT_NE(sent.find("movej(["),           std::string::npos);
  EXPECT_EQ(sent.find("move_joints("),      std::string::npos);
  // Targets present
  EXPECT_NE(sent.find("0.100000"),          std::string::npos);
  EXPECT_NE(sent.find("-1.500000"),         std::string::npos);
  EXPECT_NE(sent.find("1.500000"),          std::string::npos);
  EXPECT_NE(sent.find("0.500000"),          std::string::npos);
}

// ---------------------------------------------------------------------------
// Connect failure — server not listening
// ---------------------------------------------------------------------------

TEST(ScriptClientTest, ReturnsFalseWhenNoServer)
{
  // Port 1 is reserved; nothing listens. ScriptClient should report failure
  // rather than throwing or hanging.
  std::string script_path = make_temp_script(kDefs);
  ScriptClient client("127.0.0.1", script_path, 1);
  EXPECT_FALSE(client.move_home());
  std::remove(script_path.c_str());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
