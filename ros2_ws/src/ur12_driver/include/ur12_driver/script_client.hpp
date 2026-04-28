#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace ur12_driver
{

/// Sends URScript programs to the robot via TCP port 30002 (Secondary Interface).
///
/// Loads function definitions from a script file at construction time.
/// Each motion call appends the relevant function invocation and sends
/// the complete script — definitions + call — as a single transmission.
///
/// This matches the spec's intent: commands.script defines the functions,
/// the driver invokes them by name. Updating commands.script takes effect
/// on the next driver restart without recompiling.
class ScriptClient
{
public:
  static constexpr int kPort    = 30002;
  static constexpr int kTimeout = 5;  // seconds

  explicit ScriptClient(std::string host, const std::string & script_path)
  : host_(std::move(host))
  {
    std::ifstream f(script_path);
    if (!f.is_open()) {
      throw std::runtime_error("ScriptClient: cannot open script file: " + script_path);
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    script_defs_ = ss.str();
  }

  // ------------------------------------------------------------------
  // Motion commands — definitions come from commands.script
  // ------------------------------------------------------------------

  bool move_home()
  {
    return send_script(script_defs_ + "\nmove_home()\n", "move_home");
  }

  bool move_first_joint(double joint_val)
  {
    // Wrap in a no-arg def — URSim secondary interface silently drops
    // top-level calls that pass arguments directly.
    std::ostringstream ss;
    ss << std::fixed << script_defs_
       << "\ndef prog():\n  move_first_joint(" << joint_val << ")\nend\nprog()\n";
    return send_script(ss.str(), "move_first_joint");
  }

  // move_joints is a driver extension (not in commands.script) — inline the movej directly
  bool move_joints(const std::array<double, 6> & target)
  {
    std::ostringstream ss;
    ss << std::fixed
       << "def prog():\n"
       << "  movej(["
       << target[0] << "," << target[1] << "," << target[2] << ","
       << target[3] << "," << target[4] << "," << target[5]
       << "], a=1.4, v=1.05)\n"
       << "end\nprog()\n";
    return send_script(ss.str(), "move_joints");
  }

  bool send_script(const std::string & script, const std::string & tag = "")
  {
    int fd = open_connection();
    if (fd < 0) {
      fprintf(stderr, "[ScriptClient][%s] connect failed to %s:%d\n",
              tag.c_str(), host_.c_str(), kPort);
      return false;
    }
    ssize_t sent = ::send(fd, script.c_str(), script.size(), MSG_NOSIGNAL);
    ::close(fd);
    bool ok = (sent == static_cast<ssize_t>(script.size()));
    fprintf(stderr,
            "[ScriptClient][%s] sent %zd / %zu bytes (ok=%d)\n",
            tag.c_str(), sent, script.size(), ok ? 1 : 0);
    return ok;
  }

private:
  std::string host_;
  std::string script_defs_;

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
      freeaddrinfo(res); ::close(fd); return -1;
    }
    freeaddrinfo(res);
    return fd;
  }
};

}  // namespace ur12_driver
