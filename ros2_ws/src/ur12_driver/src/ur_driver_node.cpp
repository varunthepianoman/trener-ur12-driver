#include "ur12_driver/ur_driver_node.hpp"

#include <functional>
#include <memory>
#include <string>

namespace ur12_driver
{

UrDriverNode::UrDriverNode()
: Node("ur_driver_node")
{
  // Parameters
  declare_parameter("robot_host",   "host.docker.internal");
  declare_parameter("publish_rate", 125.0);
  declare_parameter("program_path", "/programs/ur12_program.script");

  const auto host    = get_parameter("robot_host").as_string();
  const auto rate    = get_parameter("publish_rate").as_double();
  program_path_      = get_parameter("program_path").as_string();

  RCLCPP_INFO(get_logger(), "Connecting to UR at %s", host.c_str());

  // Instantiate clients
  dashboard_ = std::make_unique<DashboardClient>(host);
  script_    = std::make_unique<ScriptClient>(host);
  rtde_      = std::make_unique<RtdeClient>(host, rate);

  // Connect to dashboard (lazy reconnect on each call if this fails)
  if (!dashboard_->connect()) {
    RCLCPP_WARN(get_logger(), "Initial dashboard connect failed — will retry on first service call");
  }

  // Start RTDE background thread — retries until robot is powered on
  rtde_->start();
  RCLCPP_INFO(get_logger(), "RTDE connecting in background — call /ur/power_on then /ur/brake_release");

  // Publisher
  js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  js_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    std::bind(&UrDriverNode::publish_joint_states, this));

  // Lifecycle services
  make_trigger("/ur/power_on",      [this]() { return dashboard_->power_on(); });
  make_trigger("/ur/power_off",     [this]() { return dashboard_->power_off(); });
  make_trigger("/ur/brake_release", [this]() { return dashboard_->brake_release(); });
  make_trigger("/ur/play",          [this]() { return dashboard_->play(); });
  make_trigger("/ur/pause",         [this]() { return dashboard_->pause(); });
  make_trigger("/ur/resume",        [this]() { return dashboard_->play(); });  // resume = play
  make_trigger("/ur/stop",          [this]() { return dashboard_->stop(); });

  // Motion services
  make_trigger("/ur/move_home", [this]() -> std::pair<bool, std::string> {
    bool ok = script_->move_home();
    return {ok, ok ? "move_home sent" : "script send failed"};
  });

  move_joint_srv_ = create_service<ur12_driver_msgs::srv::MoveFirstJoint>(
    "/ur/move_first_joint",
    [this](
      const std::shared_ptr<ur12_driver_msgs::srv::MoveFirstJoint::Request> req,
      std::shared_ptr<ur12_driver_msgs::srv::MoveFirstJoint::Response> res)
    {
      res->success = script_->move_first_joint(req->joint_val);
      res->message = res->success
        ? "move_first_joint sent"
        : "script send failed";
    });

  RCLCPP_INFO(get_logger(), "UR12 driver node ready");
}

UrDriverNode::~UrDriverNode()
{
  rtde_->stop();
  dashboard_->disconnect();
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
// Helper: create a Trigger service backed by a callable
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

}  // namespace ur12_driver
