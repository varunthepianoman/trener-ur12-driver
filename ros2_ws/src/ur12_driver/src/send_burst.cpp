// Sends N MoveJoints goals through a single rclcpp node so the gap between
// scripts hitting the driver matches the requested gap. A shell loop using
// `ros2 action send_goal &` spawns a fresh node per call, and per-call action
// discovery (~hundreds of ms) eats the requested gap.
//
// Also subscribes to /joint_states throughout the burst + a settle window
// and prints the min/max value of each joint observed. Useful to verify
// e.g. whether the robot transited through a particular target en route.
//
// Usage:
//   send_burst <gap_seconds> <settle_seconds> "<j0,j1,j2,j3,j4,j5>" [...]

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ur12_driver_msgs/action/move_joints.hpp"

using MoveJoints = ur12_driver_msgs::action::MoveJoints;

static std::array<double, 6> parse_target(const std::string & s)
{
  std::array<double, 6> t{};
  std::stringstream ss(s);
  std::string tok;
  size_t i = 0;
  while (std::getline(ss, tok, ',') && i < 6) {
    t[i++] = std::stod(tok);
  }
  if (i != 6) {
    throw std::runtime_error("target must have 6 comma-separated values: " + s);
  }
  return t;
}

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::fprintf(stderr,
      "usage: send_burst <gap_seconds> <settle_seconds> <target1> [target2 ...]\n");
    return 2;
  }
  const double gap_s    = std::stod(argv[1]);
  const double settle_s = std::stod(argv[2]);
  std::vector<std::array<double, 6>> targets;
  for (int i = 3; i < argc; ++i) targets.push_back(parse_target(argv[i]));

  rclcpp::init(argc, argv);
  auto node   = std::make_shared<rclcpp::Node>("burst_sender");
  auto client = rclcpp_action::create_client<MoveJoints>(node, "/ur/move_joints");

  std::array<double, 6> jmin, jmax;
  jmin.fill(std::numeric_limits<double>::infinity());
  jmax.fill(-std::numeric_limits<double>::infinity());
  size_t samples = 0;

  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      for (size_t i = 0; i < 6 && i < msg->position.size(); ++i) {
        jmin[i] = std::min(jmin[i], msg->position[i]);
        jmax[i] = std::max(jmax[i], msg->position[i]);
      }
      ++samples;
    });

  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "action server /ur/move_joints not available");
    rclcpp::shutdown();
    return 1;
  }

  const auto t0 = std::chrono::steady_clock::now();
  for (size_t i = 0; i < targets.size(); ++i) {
    MoveJoints::Goal goal;
    std::copy(targets[i].begin(), targets[i].end(), goal.joint_positions.begin());
    client->async_send_goal(goal);
    rclcpp::spin_some(node);

    auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
    std::printf(
      "[%6.3fs] sent goal %zu: [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\n",
      elapsed, i,
      targets[i][0], targets[i][1], targets[i][2],
      targets[i][3], targets[i][4], targets[i][5]);
    std::fflush(stdout);

    if (i + 1 < targets.size()) {
      // Spin while sleeping so /joint_states samples keep accruing.
      const auto until = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(gap_s);
      while (std::chrono::steady_clock::now() < until) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  // Settle window — keep observing while the robot finishes the winning script.
  const auto settle_until = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(settle_s);
  while (std::chrono::steady_clock::now() < settle_until) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::printf("joint_states observed: %zu samples over %.2fs\n",
              samples,
              std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count());
  for (int i = 0; i < 6; ++i) {
    std::printf("  j%d: min=%+.3f  max=%+.3f  span=%.3f\n",
                i, jmin[i], jmax[i], jmax[i] - jmin[i]);
  }

  rclcpp::shutdown();
  return 0;
}
