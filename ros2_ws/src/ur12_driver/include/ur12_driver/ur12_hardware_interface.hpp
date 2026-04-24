#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ur12_driver/rtde_client.hpp"

namespace ur12_driver
{

/// ros2_control SystemInterface plugin for the UR12.
///
/// Wires RTDE joint positions + velocities into the ros2_control
/// state interface buffers so standard controllers (JointStateBroadcaster,
/// JointTrajectoryController) can consume them without knowing about RTDE.
///
/// The write() path is a scaffold — it logs commands but does not yet
/// forward them to the robot. Production path: External Control URCap
/// reverse interface at 500 Hz.
class Ur12HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Ur12HardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<RtdeClient> rtde_;

  // Buffers shared with ros2_control via StateInterface / CommandInterface pointers
  std::array<double, 6> hw_positions_{};
  std::array<double, 6> hw_velocities_{};
  std::array<double, 6> hw_commands_{};  // scaffold — not forwarded to robot yet

  std::string host_;
  double      rate_{125.0};
};

}  // namespace ur12_driver
