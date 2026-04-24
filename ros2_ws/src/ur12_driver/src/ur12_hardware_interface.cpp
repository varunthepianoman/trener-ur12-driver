#include "ur12_driver/ur12_hardware_interface.hpp"

#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ur12_driver
{

// ------------------------------------------------------------------
// on_init — validate URDF hardware info and cache parameters
// ------------------------------------------------------------------

hardware_interface::CallbackReturn Ur12HardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(rclcpp::get_logger("Ur12HardwareInterface"),
      "Expected 6 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read hardware parameters from URDF <param> tags
  host_ = info_.hardware_parameters.count("robot_host")
    ? info_.hardware_parameters.at("robot_host")
    : "localhost";

  rate_ = info_.hardware_parameters.count("publish_rate")
    ? std::stod(info_.hardware_parameters.at("publish_rate"))
    : 125.0;

  // Validate each joint has position+velocity state and position command
  for (const auto & joint : info_.joints) {
    bool has_pos_state = false, has_vel_state = false, has_pos_cmd = false;
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) has_pos_state = true;
      if (si.name == hardware_interface::HW_IF_VELOCITY) has_vel_state = true;
    }
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) has_pos_cmd = true;
    }
    if (!has_pos_state || !has_vel_state || !has_pos_cmd) {
      RCLCPP_ERROR(rclcpp::get_logger("Ur12HardwareInterface"),
        "Joint '%s' missing required interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialise command buffer to NaN so write() can detect uninitialised commands
  hw_commands_.fill(std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(rclcpp::get_logger("Ur12HardwareInterface"),
    "Initialised — robot_host=%s rate=%.0f Hz", host_.c_str(), rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------
// on_configure — create RTDE client and start background thread
// ------------------------------------------------------------------

hardware_interface::CallbackReturn Ur12HardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  rtde_ = std::make_unique<RtdeClient>(host_, rate_);
  rtde_->start();
  RCLCPP_INFO(rclcpp::get_logger("Ur12HardwareInterface"),
    "RTDE connecting in background (robot must be powered on)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------
// on_activate / on_deactivate
// ------------------------------------------------------------------

hardware_interface::CallbackReturn Ur12HardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("Ur12HardwareInterface"), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ur12HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  rtde_->stop();
  RCLCPP_INFO(rclcpp::get_logger("Ur12HardwareInterface"), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------
// export_state_interfaces — expose position + velocity per joint
// ------------------------------------------------------------------

std::vector<hardware_interface::StateInterface>
Ur12HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return interfaces;
}

// ------------------------------------------------------------------
// export_command_interfaces — expose position command per joint (scaffold)
// ------------------------------------------------------------------

std::vector<hardware_interface::CommandInterface>
Ur12HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return interfaces;
}

// ------------------------------------------------------------------
// read — pull latest state from RTDE into ros2_control buffers
// ------------------------------------------------------------------

hardware_interface::return_type Ur12HardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  auto state = rtde_->get_state();
  for (size_t i = 0; i < 6; ++i) {
    hw_positions_[i]  = state.joint_positions[i];
    hw_velocities_[i] = state.joint_velocities[i];
  }
  return hardware_interface::return_type::OK;
}

// ------------------------------------------------------------------
// write — scaffold: log commands, do not forward to robot yet
// Production path: stream joint targets via External Control URCap
// reverse interface (port 50001) at 500 Hz.
// ------------------------------------------------------------------

hardware_interface::return_type Ur12HardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // No-op until reverse interface is implemented
  return hardware_interface::return_type::OK;
}

}  // namespace ur12_driver

PLUGINLIB_EXPORT_CLASS(
  ur12_driver::Ur12HardwareInterface,
  hardware_interface::SystemInterface)
