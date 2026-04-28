// Sends one MoveJoints goal, sleeps, then issues an action-level cancel via
// async_cancel_goal — exercises handle_move_joints_cancel in the driver.
//
// Prints the final goal status (SUCCEEDED / CANCELED / ABORTED) and the
// joint_states snapshot at the moment cancel completed.
//
// Usage:
//   cancel_goal <wait_before_cancel_seconds> "<j0,j1,j2,j3,j4,j5>"

#include <array>
#include <chrono>
#include <cstdio>
#include <future>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ur12_driver_msgs/action/move_joints.hpp"

using MoveJoints  = ur12_driver_msgs::action::MoveJoints;
using GoalHandle  = rclcpp_action::ClientGoalHandle<MoveJoints>;

static std::array<double, 6> parse_target(const std::string & s)
{
  std::array<double, 6> t{};
  std::stringstream ss(s);
  std::string tok;
  size_t i = 0;
  while (std::getline(ss, tok, ',') && i < 6) t[i++] = std::stod(tok);
  if (i != 6) throw std::runtime_error("target needs 6 values: " + s);
  return t;
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::fprintf(stderr,
      "usage: cancel_goal <wait_before_cancel_seconds> <j0,j1,j2,j3,j4,j5>\n");
    return 2;
  }
  const double wait_s = std::stod(argv[1]);
  const auto target   = parse_target(argv[2]);

  rclcpp::init(argc, argv);
  auto node   = std::make_shared<rclcpp::Node>("cancel_goal_client");
  auto client = rclcpp_action::create_client<MoveJoints>(node, "/ur/move_joints");

  std::array<double, 6> latest_pos{};
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      for (size_t i = 0; i < 6 && i < m->position.size(); ++i) latest_pos[i] = m->position[i];
    });

  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "action server unavailable");
    rclcpp::shutdown();
    return 1;
  }

  // Send goal
  MoveJoints::Goal goal;
  std::copy(target.begin(), target.end(), goal.joint_positions.begin());

  rclcpp_action::Client<MoveJoints>::SendGoalOptions opts;
  std::promise<int8_t> result_status;
  auto result_future = result_status.get_future();
  opts.result_callback = [&](const GoalHandle::WrappedResult & r) {
    result_status.set_value(static_cast<int8_t>(r.code));
  };

  auto goal_future = client->async_send_goal(goal, opts);
  if (rclcpp::spin_until_future_complete(node, goal_future, std::chrono::seconds(5))
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    std::fprintf(stderr, "goal send timed out\n");
    rclcpp::shutdown();
    return 1;
  }
  auto gh = goal_future.get();
  if (!gh) {
    std::fprintf(stderr, "goal was rejected\n");
    rclcpp::shutdown();
    return 1;
  }
  std::printf("goal accepted; waiting %.2fs before cancel...\n", wait_s);

  // Pump executor while we wait so feedback / status flow.
  const auto cancel_at = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(wait_s);
  while (std::chrono::steady_clock::now() < cancel_at) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::printf("issuing cancel at j=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\n",
    latest_pos[0], latest_pos[1], latest_pos[2],
    latest_pos[3], latest_pos[4], latest_pos[5]);
  client->async_cancel_goal(gh);

  // Wait for terminal status
  if (rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(10))
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    std::fprintf(stderr, "did not reach terminal state in 10s\n");
    rclcpp::shutdown();
    return 1;
  }

  const int8_t code = result_future.get();
  const char * label = "UNKNOWN";
  switch (static_cast<rclcpp_action::ResultCode>(code)) {
    case rclcpp_action::ResultCode::SUCCEEDED: label = "SUCCEEDED"; break;
    case rclcpp_action::ResultCode::CANCELED:  label = "CANCELED";  break;
    case rclcpp_action::ResultCode::ABORTED:   label = "ABORTED";   break;
    case rclcpp_action::ResultCode::UNKNOWN:   label = "UNKNOWN";   break;
  }
  std::printf("terminal status: %s\n", label);
  std::printf("final j=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]  target=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\n",
    latest_pos[0], latest_pos[1], latest_pos[2],
    latest_pos[3], latest_pos[4], latest_pos[5],
    target[0], target[1], target[2], target[3], target[4], target[5]);

  rclcpp::shutdown();
  return (code == static_cast<int8_t>(rclcpp_action::ResultCode::CANCELED)) ? 0 : 1;
}
