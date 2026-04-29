#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace ur12_driver
{

/// Sends URScript programs to the robot via TCP port 30002 (Secondary Interface).
///
/// Two patterns coexist:
///
///   1. File-loaded defs + helper invocation (move_home, move_first_joint).
///      The script file (commands.script) is read once at construction;
///      each call concatenates `<defs>` then either a bare invocation
///      (move_home — parameterless) or a `def prog(): helper(scalar) end
///      prog()` wrapper (move_first_joint — scalar arg). Editing
///      commands.script tunes motion behaviour without recompiling.
///
///   2. Inlined movej (move_joints).
///      The whole `def prog(): movej([...]) end prog()` is built in C++.
///      We tried unifying through a helper in commands.script; URSim's
///      secondary interface silently dropped scripts where the helper
///      was invoked with a list-literal argument. The inlined form is
///      proven and ships.
///
/// In both patterns the entry call is wrapped in `def prog(): ... end`
/// when arguments are passed: URSim silently drops top-level calls with
/// arguments.
class ScriptClient
{
public:
  static constexpr int kPort    = 30002;
  static constexpr int kTimeout = 5;  // seconds

  explicit ScriptClient(std::string host, const std::string & script_path,
                        int port = kPort)
  : host_(std::move(host)), port_(port)
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

  bool move_joints(const std::array<double, 6> & target)
  {
    // Inlined movej rather than routing through a helper in commands.script.
    // Empirically, the helper-call-from-prog() pattern works for scalar args
    // (see move_first_joint above) but URSim's secondary interface silently
    // drops the script when the helper is invoked with a list-literal arg.
    // We don't have time to characterise that quirk; the inlined form is
    // proven and shipped.
    std::ostringstream ss;
    ss << std::fixed
       << "def prog():\n"
       << "  movej(["
       << target[0] << "," << target[1] << "," << target[2] << ","
       << target[3] << "," << target[4] << "," << target[5]
       << "])\n"
       << "end\nprog()\n";
    return send_script(ss.str(), "move_joints");
  }

  bool send_script(const std::string & script, const std::string & tag = "")
  {
    int fd = open_connection();
    if (fd < 0) {
      RCLCPP_WARN(logger(), "[%s] connect failed to %s:%d",
                  tag.c_str(), host_.c_str(), port_);
      return false;
    }
    ssize_t sent = ::send(fd, script.c_str(), script.size(), MSG_NOSIGNAL);
    ::close(fd);
    bool ok = (sent == static_cast<ssize_t>(script.size()));
    if (ok) {
      RCLCPP_INFO(logger(), "[%s] sent %zu bytes", tag.c_str(), script.size());
      // Last 120 chars of payload — useful for spotting silent URSim drops
      // (parsing failures, reserved-word collisions, etc.).
      std::string tail = script.size() > 120
        ? "..." + script.substr(script.size() - 120)
        : script;
      RCLCPP_DEBUG(logger(), "[%s] tail: %s", tag.c_str(), tail.c_str());
    } else {
      RCLCPP_WARN(logger(), "[%s] short send: %zd / %zu bytes",
                  tag.c_str(), sent, script.size());
    }
    return ok;
  }

private:
  std::string host_;
  int         port_;
  std::string script_defs_;

  static rclcpp::Logger logger()
  {
    static auto lg = rclcpp::get_logger("ScriptClient");
    return lg;
  }

  int open_connection()
  {
    struct addrinfo hints{}, * res = nullptr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0) {
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
