#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur12_driver_msgs/action/move_home.hpp"
#include "ur12_driver_msgs/action/move_first_joint.hpp"

#include "ur12_driver/dashboard_client.hpp"
#include "ur12_driver/rtde_client.hpp"
#include "ur12_driver/script_client.hpp"

namespace ur12_driver
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using MoveHomeAction       = ur12_driver_msgs::action::MoveHome;
using MoveFirstJointAction = ur12_driver_msgs::action::MoveFirstJoint;
using GoalHandleMoveHome       = rclcpp_action::ServerGoalHandle<MoveHomeAction>;
using GoalHandleMoveFirstJoint = rclcpp_action::ServerGoalHandle<MoveFirstJointAction>;

class UrDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit UrDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UrDriverNode() override;

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Clients
  std::unique_ptr<DashboardClient> dashboard_;
  std::unique_ptr<ScriptClient>    script_;
  std::unique_ptr<RtdeClient>      rtde_;

  // Lifecycle publisher + timer
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>> js_pub_;
  rclcpp::TimerBase::SharedPtr js_timer_;

  // Lifecycle services (power, brakes, play/pause/stop)
  std::vector<rclcpp::ServiceBase::SharedPtr> trigger_services_;

  // Action servers
  rclcpp_action::Server<MoveHomeAction>::SharedPtr       move_home_server_;
  rclcpp_action::Server<MoveFirstJointAction>::SharedPtr move_joint_server_;

  // Cached params
  std::string host_;
  double      rate_;
  std::string program_path_;

  // One-shot timer for auto configure+activate
  rclcpp::TimerBase::SharedPtr init_timer_;

  void publish_joint_states();

  void make_trigger(
    const std::string & name,
    std::function<std::pair<bool, std::string>()> fn);

  // MoveHome action callbacks
  rclcpp_action::GoalResponse handle_move_home_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveHomeAction::Goal> goal);
  rclcpp_action::CancelResponse handle_move_home_cancel(
    std::shared_ptr<GoalHandleMoveHome> goal_handle);
  void handle_move_home_accepted(std::shared_ptr<GoalHandleMoveHome> goal_handle);
  void execute_move_home(std::shared_ptr<GoalHandleMoveHome> goal_handle);

  // MoveFirstJoint action callbacks
  rclcpp_action::GoalResponse handle_move_joint_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveFirstJointAction::Goal> goal);
  rclcpp_action::CancelResponse handle_move_joint_cancel(
    std::shared_ptr<GoalHandleMoveFirstJoint> goal_handle);
  void handle_move_joint_accepted(std::shared_ptr<GoalHandleMoveFirstJoint> goal_handle);
  void execute_move_joint(std::shared_ptr<GoalHandleMoveFirstJoint> goal_handle);
};

}  // namespace ur12_driver
