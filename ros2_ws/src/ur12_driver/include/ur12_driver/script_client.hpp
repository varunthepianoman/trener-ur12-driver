#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>
#include <string>
#include <utility>

namespace ur12_driver
{

/// Sends URScript programs to the robot via TCP port 30002 (Secondary Interface).
///
/// Each call opens a fresh connection, sends the script text, and closes.
/// The robot executes the script immediately upon receipt.
///
/// For the PoC we embed the function definition inline with each call so
/// there is no dependency on a pre-loaded program. The production path
/// is the External Control URCap + reverse interface at 500 Hz.
class ScriptClient
{
public:
  static constexpr int kPort    = 30002;
  static constexpr int kTimeout = 5;  // seconds

  explicit ScriptClient(std::string host = "localhost")
  : host_(std::move(host)) {}

  // ------------------------------------------------------------------
  // Motion commands (spec §2)
  // ------------------------------------------------------------------

  bool move_home()
  {
    // movej to the zero/home configuration
    return send_script(
      "def move_home():\n"
      "  movej([0, -1.5707, 1.5707, 0, 0, 0], a=1.4, v=1.05)\n"
      "end\n"
      "move_home()\n");
  }

  /// Move joint 0 by `joint_val` radians relative to its current position.
  bool move_first_joint(double joint_val)
  {
    std::ostringstream ss;
    ss << std::fixed;
    // URSim secondary interface silently drops scripts where the entry
    // function takes parameters, so we inline joint_val as a local.
    ss << "def prog():\n"
       << "  joint_val = " << joint_val << "\n"
       << "  q = get_actual_joint_positions()\n"
       << "  movej([q[0] + joint_val, q[1], q[2], q[3], q[4], q[5]], a=1.4, v=1.05)\n"
       << "end\n"
       << "prog()\n";
    return send_script(ss.str());
  }

  /// Send arbitrary URScript text to the robot.
  bool send_script(const std::string & script)
  {
    int fd = open_connection();
    if (fd < 0) return false;

    ssize_t sent = ::send(fd, script.c_str(), script.size(), MSG_NOSIGNAL);
    ::close(fd);
    return sent == static_cast<ssize_t>(script.size());
  }

private:
  std::string host_;

  int open_connection()
  {
    struct addrinfo hints{}, * res = nullptr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host_.c_str(), std::to_string(kPort).c_str(), &hints, &res) != 0) {
      return -1;
    }

    int fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) { freeaddrinfo(res); return -1; }

    struct timeval tv{kTimeout, 0};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (::connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
      freeaddrinfo(res);
      ::close(fd);
      return -1;
    }
    freeaddrinfo(res);
    return fd;
  }
};

}  // namespace ur12_driver
