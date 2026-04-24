#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace ur12_driver
{

/// Talks to the UR Dashboard Server on TCP port 29999.
/// Handles the robot controller lifecycle: power, brakes, program play/pause/stop.
///
/// The dashboard protocol is line-oriented plain text:
///   client sends: "power on\n"
///   server replies: "Powering on\n"
///
/// A fresh connection is made lazily and reused. On any error the socket is
/// closed so the next call reconnects automatically.
class DashboardClient
{
public:
  static constexpr int kPort    = 29999;
  static constexpr int kTimeout = 5;  // seconds

  explicit DashboardClient(std::string host = "localhost", int port = kPort)
  : host_(std::move(host)), port_(port), fd_(-1) {}

  ~DashboardClient() { disconnect(); }

  // Non-copyable, movable
  DashboardClient(const DashboardClient &) = delete;
  DashboardClient & operator=(const DashboardClient &) = delete;

  // ------------------------------------------------------------------
  // Lifecycle commands (spec §2)
  // ------------------------------------------------------------------

  std::pair<bool, std::string> power_on()      { return cmd("power on"); }
  std::pair<bool, std::string> power_off()     { return cmd("power off"); }
  std::pair<bool, std::string> brake_release() { return cmd("brake release"); }
  std::pair<bool, std::string> play()          { return cmd("play"); }
  std::pair<bool, std::string> pause()         { return cmd("pause"); }
  std::pair<bool, std::string> stop()          { return cmd("stop"); }

  std::pair<bool, std::string> load_program(const std::string & path)
  {
    return cmd("load " + path);
  }

  std::pair<bool, std::string> get_robot_mode()    { return cmd("robotmode"); }
  std::pair<bool, std::string> get_program_state() { return cmd("programstate"); }

  bool connect()
  {
    disconnect();

    struct addrinfo hints{}, * res = nullptr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0) {
      return false;
    }

    fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd_ < 0) { freeaddrinfo(res); return false; }

    // Set send/recv timeout
    struct timeval tv{kTimeout, 0};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (::connect(fd_, res->ai_addr, res->ai_addrlen) != 0) {
      freeaddrinfo(res);
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    freeaddrinfo(res);

    // Consume the greeting line. URSim may send it a moment after TCP connect,
    // so retry a few times if the first read times out — otherwise the greeting
    // leaks into the response of the first real command.
    for (int i = 0; i < 5; ++i) {
      std::string line = recv_line();
      if (!line.empty()) return true;
    }
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  void disconnect()
  {
    if (fd_ >= 0) {
      send_line("quit");
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  std::string host_;
  int         port_;
  int         fd_;

  std::pair<bool, std::string> cmd(const std::string & command)
  {
    if (fd_ < 0 && !connect()) {
      return {false, "Not connected to dashboard server"};
    }
    if (!send_line(command)) {
      fd_ = -1;
      return {false, "Send failed"};
    }
    std::string reply = recv_line();
    if (reply.empty()) {
      fd_ = -1;
      return {false, "No reply from dashboard server"};
    }
    // Trim trailing whitespace
    while (!reply.empty() && (reply.back() == '\n' || reply.back() == '\r')) {
      reply.pop_back();
    }
    bool ok = reply.find("Failed") == std::string::npos &&
              reply.find("failed") == std::string::npos &&
              reply.find("error")  == std::string::npos;
    return {ok, reply};
  }

  bool send_line(const std::string & text)
  {
    std::string msg = text + "\n";
    ssize_t sent = ::send(fd_, msg.c_str(), msg.size(), MSG_NOSIGNAL);
    return sent == static_cast<ssize_t>(msg.size());
  }

  std::string recv_line()
  {
    std::string buf;
    char c = 0;
    while (c != '\n') {
      ssize_t n = ::recv(fd_, &c, 1, 0);
      if (n <= 0) return buf;
      buf += c;
    }
    return buf;
  }
};

}  // namespace ur12_driver
