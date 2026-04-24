#include "ur12_driver/ur_driver_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

namespace ur12_driver
{

// ------------------------------------------------------------------
// Construction / destruction
// ------------------------------------------------------------------

UrDriverNode::UrDriverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ur_driver_node", options)
{
  declare_parameter("robot_host",   "host.docker.internal");
  declare_parameter("publish_rate", 125.0);
  declare_parameter("program_path", "/programs/ur12_program.script");

  // Self-configure and activate once the executor is spinning.
  // Production systems would use ros2_lifecycle_manager instead.
  init_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    [this]() {
      init_timer_.reset();  // one-shot
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    });
}

UrDriverNode::~UrDriverNode()
{
  if (rtde_)       rtde_->stop();
  if (dashboard_)  dashboard_->disconnect();
}

// ------------------------------------------------------------------
// Lifecycle: configure
// ------------------------------------------------------------------

CallbackReturn UrDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  host_         = get_parameter("robot_host").as_string();
  rate_         = get_parameter("publish_rate").as_double();
  program_path_ = get_parameter("program_path").as_string();

  RCLCPP_INFO(get_logger(), "Configuring — connecting to UR at %s", host_.c_str());

  dashboard_ = std::make_unique<DashboardClient>(host_);
  script_    = std::make_unique<ScriptClient>(host_);
  rtde_      = std::make_unique<RtdeClient>(host_, rate_);

  if (!dashboard_->connect()) {
    RCLCPP_WARN(get_logger(), "Dashboard connect failed — will retry on first service call");
  }

  // Publisher (inactive until on_activate)
  js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  // Lifecycle control services — available from configured state
  make_trigger("/ur/power_on",      [this]() { return dashboard_->power_on(); });
  make_trigger("/ur/power_off",     [this]() { return dashboard_->power_off(); });
  make_trigger("/ur/brake_release", [this]() { return dashboard_->brake_release(); });
  make_trigger("/ur/play",          [this]() { return dashboard_->play(); });
  make_trigger("/ur/pause",         [this]() { return dashboard_->pause(); });
  make_trigger("/ur/resume",        [this]() { return dashboard_->play(); });
  make_trigger("/ur/stop",          [this]() { return dashboard_->stop(); });

  // Action servers — available from configured state, reject goals when inactive
  move_home_server_ = rclcpp_action::create_server<MoveHomeAction>(
    this, "/ur/move_home",
    [this](const auto & uuid, auto goal) { return handle_move_home_goal(uuid, goal); },
    [this](auto gh)                      { return handle_move_home_cancel(gh); },
    [this](auto gh)                      { handle_move_home_accepted(gh); });

  move_joint_server_ = rclcpp_action::create_server<MoveFirstJointAction>(
    this, "/ur/move_first_joint",
    [this](const auto & uuid, auto goal) { return handle_move_joint_goal(uuid, goal); },
    [this](auto gh)                      { return handle_move_joint_cancel(gh); },
    [this](auto gh)                      { handle_move_joint_accepted(gh); });

  RCLCPP_INFO(get_logger(), "Configured");
  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------
// Lifecycle: activate
// ------------------------------------------------------------------

CallbackReturn UrDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  rclcpp_lifecycle::LifecycleNode::on_activate(state);  // activates managed publishers

  rtde_->start();
  RCLCPP_INFO(get_logger(), "RTDE connecting in background — call /ur/power_on then /ur/brake_release");

  js_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_),
    std::bind(&UrDriverNode::publish_joint_states, this));

  RCLCPP_INFO(get_logger(), "Active — publishing joint states at %.0f Hz", rate_);
  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------
// Lifecycle: deactivate
// ------------------------------------------------------------------

CallbackReturn UrDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  js_timer_.reset();
  rtde_->stop();
  rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------
// Lifecycle: cleanup
// ------------------------------------------------------------------

CallbackReturn UrDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  js_pub_.reset();
  trigger_services_.clear();
  move_home_server_.reset();
  move_joint_server_.reset();
  dashboard_->disconnect();
  dashboard_.reset();
  script_.reset();
  rtde_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------
// Lifecycle: shutdown
// ------------------------------------------------------------------

CallbackReturn UrDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (js_timer_)   js_timer_.reset();
  if (rtde_)       rtde_->stop();
  if (dashboard_)  dashboard_->disconnect();
  RCLCPP_INFO(get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------
// Joint state publisher
// ------------------------------------------------------------------

void UrDriverNode::publish_joint_states()
{
  auto state = rtde_->get_state();

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.name = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };
  msg.position = std::vector<double>(
    state.joint_positions.begin(), state.joint_positions.end());
  msg.velocity = std::vector<double>(
    state.joint_velocities.begin(), state.joint_velocities.end());

  js_pub_->publish(msg);
}

// ------------------------------------------------------------------
// Helper: Trigger service factory
// ------------------------------------------------------------------

void UrDriverNode::make_trigger(
  const std::string & name,
  std::function<std::pair<bool, std::string>()> fn)
{
  auto srv = create_service<std_srvs::srv::Trigger>(
    name,
    [fn](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      auto [ok, msg] = fn();
      res->success = ok;
      res->message = msg;
    });
  trigger_services_.push_back(srv);
}

// ------------------------------------------------------------------
// MoveHome action
// ------------------------------------------------------------------

rclcpp_action::GoalResponse UrDriverNode::handle_move_home_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveHomeAction::Goal>)
{
  if (!rtde_ || !rtde_->is_connected()) {
    RCLCPP_WARN(get_logger(), "move_home rejected — RTDE not connected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UrDriverNode::handle_move_home_cancel(
  std::shared_ptr<GoalHandleMoveHome>)
{
  RCLCPP_INFO(get_logger(), "move_home cancel requested");
  dashboard_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UrDriverNode::handle_move_home_accepted(
  std::shared_ptr<GoalHandleMoveHome> goal_handle)
{
  std::thread{[this, goal_handle]() { execute_move_home(goal_handle); }}.detach();
}

void UrDriverNode::execute_move_home(std::shared_ptr<GoalHandleMoveHome> goal_handle)
{
  constexpr double kThreshold = 0.02;   // rad
  constexpr double kTimeoutS  = 15.0;
  const std::array<double, 6> home = {0, -1.5707, 1.5707, 0, 0, 0};

  RCLCPP_INFO(get_logger(), "Executing move_home");
  script_->move_home();

  auto feedback = std::make_shared<MoveHomeAction::Feedback>();
  auto start    = std::chrono::steady_clock::now();
  rclcpp::Rate poll(10);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      auto result     = std::make_shared<MoveHomeAction::Result>();
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (elapsed > kTimeoutS) {
      auto result     = std::make_shared<MoveHomeAction::Result>();
      result->success = false;
      result->message = "Timeout";
      goal_handle->abort(result);
      return;
    }

    auto state   = rtde_->get_state();
    double max_err = 0.0;
    for (int i = 0; i < 6; ++i) {
      feedback->current_positions[i] = state.joint_positions[i];
      max_err = std::max(max_err, std::abs(state.joint_positions[i] - home[i]));
    }
    feedback->distance_to_goal = max_err;
    goal_handle->publish_feedback(feedback);

    if (max_err < kThreshold) {
      auto result     = std::make_shared<MoveHomeAction::Result>();
      result->success = true;
      result->message = "Reached home";
      goal_handle->succeed(result);
      return;
    }

    poll.sleep();
  }
}

// ------------------------------------------------------------------
// MoveFirstJoint action
// ------------------------------------------------------------------

rclcpp_action::GoalResponse UrDriverNode::handle_move_joint_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveFirstJointAction::Goal>)
{
  if (!rtde_ || !rtde_->is_connected()) {
    RCLCPP_WARN(get_logger(), "move_first_joint rejected — RTDE not connected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UrDriverNode::handle_move_joint_cancel(
  std::shared_ptr<GoalHandleMoveFirstJoint>)
{
  RCLCPP_INFO(get_logger(), "move_first_joint cancel requested");
  dashboard_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UrDriverNode::handle_move_joint_accepted(
  std::shared_ptr<GoalHandleMoveFirstJoint> goal_handle)
{
  std::thread{[this, goal_handle]() { execute_move_joint(goal_handle); }}.detach();
}

void UrDriverNode::execute_move_joint(
  std::shared_ptr<GoalHandleMoveFirstJoint> goal_handle)
{
  constexpr double kThreshold = 0.02;
  constexpr double kTimeoutS  = 15.0;

  const double joint_val = goal_handle->get_goal()->joint_val;

  // Snapshot current j0 to compute target before sending script
  const double target_j0 =
    rtde_->get_state().joint_positions[0] + joint_val;

  RCLCPP_INFO(get_logger(), "Executing move_first_joint(%.3f) → target j0=%.3f",
    joint_val, target_j0);
  script_->move_first_joint(joint_val);

  auto feedback = std::make_shared<MoveFirstJointAction::Feedback>();
  auto start    = std::chrono::steady_clock::now();
  rclcpp::Rate poll(10);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      auto result     = std::make_shared<MoveFirstJointAction::Result>();
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (elapsed > kTimeoutS) {
      auto result     = std::make_shared<MoveFirstJointAction::Result>();
      result->success = false;
      result->message = "Timeout";
      goal_handle->abort(result);
      return;
    }

    double current_j0 = rtde_->get_state().joint_positions[0];
    double dist       = std::abs(current_j0 - target_j0);
    feedback->current_position  = current_j0;
    feedback->distance_remaining = dist;
    goal_handle->publish_feedback(feedback);

    if (dist < kThreshold) {
      auto result     = std::make_shared<MoveFirstJointAction::Result>();
      result->success = true;
      result->message = "Reached target";
      goal_handle->succeed(result);
      return;
    }

    poll.sleep();
  }
}

}  // namespace ur12_driver
