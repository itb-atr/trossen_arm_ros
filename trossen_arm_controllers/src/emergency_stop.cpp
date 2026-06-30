// Copyright 2026
// BSD-3-Clause, matching the surrounding Trossen Robotics packages.

#include "trossen_arm_controllers/emergency_stop.hpp"

#include <functional>
#include <memory>

namespace trossen_arm_controllers
{

CallbackReturn EmergencyStopController::on_init()
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn EmergencyStopController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  engage_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/emergency_stop_engage",
    std::bind(
      &EmergencyStopController::handle_engage_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  release_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/emergency_stop_release",
    std::bind(
      &EmergencyStopController::handle_release_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured emergency stop controller with services '~/emergency_stop_engage' and '~/emergency_stop_release'.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn EmergencyStopController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_command_id_ = 0;
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EmergencyStopController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    std::string(trossen_arm_hardware::EMERGENCY_STOP_COMPONENT_NAME) + "/" +
      trossen_arm_hardware::HW_IF_EMERGENCY_STOP_ENGAGE,
    std::string(trossen_arm_hardware::EMERGENCY_STOP_COMPONENT_NAME) + "/" +
      trossen_arm_hardware::HW_IF_EMERGENCY_STOP_RELEASE,
    std::string(trossen_arm_hardware::EMERGENCY_STOP_COMPONENT_NAME) + "/" +
      trossen_arm_hardware::HW_IF_EMERGENCY_STOP_COMMAND_ID};
  return config;
}

controller_interface::InterfaceConfiguration
EmergencyStopController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::return_type EmergencyStopController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto * command = command_buffer_.readFromRT();
  if (command == nullptr || command->id == 0 || command->id == last_command_id_) {
    return controller_interface::return_type::OK;
  }

  if (command_interfaces_.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 3 command interfaces but got %zu.", command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  if (!set_command_interface(command_interfaces_[0], command->engage ? 1.0 : 0.0, get_node()->get_logger()) ||
      !set_command_interface(command_interfaces_[1], command->release ? 1.0 : 0.0, get_node()->get_logger()) ||
      !set_command_interface(command_interfaces_[2], static_cast<double>(command->id), get_node()->get_logger()))
  {
    return controller_interface::return_type::ERROR;
  }

  last_command_id_ = command->id;
  return controller_interface::return_type::OK;
}

void EmergencyStopController::handle_engage_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  Command command;
  command.engage = true;
  command.release = false;
  command.id = ++next_command_id_;
  command_buffer_.writeFromNonRT(command);

  response->success = true;
  response->message = "Emergency stop engage command queued.";
}

void EmergencyStopController::handle_release_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  Command command;
  command.engage = false;
  command.release = true;
  command.id = ++next_command_id_;
  command_buffer_.writeFromNonRT(command);

  response->success = true;
  response->message = "Emergency stop release command queued.";
}

bool EmergencyStopController::set_command_interface(
  hardware_interface::LoanedCommandInterface & command_interface,
  double value,
  const rclcpp::Logger & logger)
{
  if (!command_interface.set_value(value)) {
    RCLCPP_ERROR(logger, "Failed to set command interface '%s'.", command_interface.get_name().c_str());
    return false;
  }
  return true;
}

}  // namespace trossen_arm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  trossen_arm_controllers::EmergencyStopController,
  controller_interface::ControllerInterface)
