#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur12_driver_msgs/srv/move_first_joint.hpp"

#include "ur12_driver/dashboard_client.hpp"
#include "ur12_driver/rtde_client.hpp"
#include "ur12_driver/script_client.hpp"

namespace ur12_driver
{

class UrDriverNode : public rclcpp::Node
{
public:
  UrDriverNode();
  ~UrDriverNode() override;

private:
  // Clients
  std::unique_ptr<DashboardClient> dashboard_;
  std::unique_ptr<ScriptClient>    script_;
  std::unique_ptr<RtdeClient>      rtde_;

  // Publisher + timer
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr js_timer_;

  // Services
  std::vector<rclcpp::ServiceBase::SharedPtr> trigger_services_;
  rclcpp::Service<ur12_driver_msgs::srv::MoveFirstJoint>::SharedPtr move_joint_srv_;

  std::string program_path_;

  void publish_joint_states();

  void make_trigger(
    const std::string & name,
    std::function<std::pair<bool, std::string>()> fn);
};

}  // namespace ur12_driver
