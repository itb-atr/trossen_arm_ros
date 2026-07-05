// Copyright 2026
// BSD-3-Clause, matching the surrounding Trossen Robotics packages.

#include "trossen_arm_controllers/cartesian_external_effort.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <utility>

namespace
{

bool cartesian_wrench_command_values_are_finite(
  const trossen_arm_msgs::msg::CartesianWrenchCommand & msg)
{
  return std::isfinite(msg.wrench.force.x) &&
         std::isfinite(msg.wrench.force.y) &&
         std::isfinite(msg.wrench.force.z) &&
         std::isfinite(msg.wrench.torque.x) &&
         std::isfinite(msg.wrench.torque.y) &&
         std::isfinite(msg.wrench.torque.z) &&
         std::isfinite(msg.goal_time);
}

}  // namespace

namespace trossen_arm_controllers
{

CallbackReturn CartesianExternalEffortController::on_init()
{
  try {
    auto_declare<std::string>("cartesian_interface_name", trossen_arm_hardware::CARTESIAN_COMPONENT_NAME);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianExternalEffortController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    cartesian_interface_name_ = get_node()->get_parameter("cartesian_interface_name").as_string();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (cartesian_interface_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'cartesian_interface_name' must not be empty.");
    return CallbackReturn::ERROR;
  }

  command_subscriber_ = get_node()->create_subscription<trossen_arm_msgs::msg::CartesianWrenchCommand>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianExternalEffortController::command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured Cartesian external effort controller. Publish trossen_arm_msgs/msg/CartesianWrenchCommand to '~/command'.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianExternalEffortController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_command_id_ = 0;
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianExternalEffortController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_EXTERNAL_EFFORT_FX,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_EXTERNAL_EFFORT_FY,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_EXTERNAL_EFFORT_FZ,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_EXTERNAL_EFFORT_TX,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_EXTERNAL_EFFORT_TY,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_EXTERNAL_EFFORT_TZ,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_EXTERNAL_EFFORT_GOAL_TIME,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_EXTERNAL_EFFORT_INTERPOLATION_SPACE,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_EXTERNAL_EFFORT_COMMAND_ID};
  return config;
}

controller_interface::InterfaceConfiguration
CartesianExternalEffortController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::return_type CartesianExternalEffortController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto * command = command_buffer_.readFromRT();
  if (command == nullptr || command->id == 0 || command->id == last_command_id_) {
    return controller_interface::return_type::OK;
  }

  if (command_interfaces_.size() != 9) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 9 command interfaces but got %zu.", command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < command->wrench.size(); ++i) {
    if (!set_command_interface(command_interfaces_[i], command->wrench[i], get_node()->get_logger())) {
      return controller_interface::return_type::ERROR;
    }
  }

  if (!set_command_interface(command_interfaces_[6], command->goal_time, get_node()->get_logger()) ||
      !set_command_interface(command_interfaces_[7], command->interpolation_space, get_node()->get_logger()) ||
      !set_command_interface(command_interfaces_[8], static_cast<double>(command->id), get_node()->get_logger()))
  {
    return controller_interface::return_type::ERROR;
  }

  last_command_id_ = command->id;
  return controller_interface::return_type::OK;
}

void CartesianExternalEffortController::command_callback(
  const trossen_arm_msgs::msg::CartesianWrenchCommand::SharedPtr msg)
{
  if (!cartesian_wrench_command_values_are_finite(*msg)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cartesian wrench command contains a non-finite value.");
    return;
  }

  Command command;
  command.wrench[0] = msg->wrench.force.x;
  command.wrench[1] = msg->wrench.force.y;
  command.wrench[2] = msg->wrench.force.z;
  command.wrench[3] = msg->wrench.torque.x;
  command.wrench[4] = msg->wrench.torque.y;
  command.wrench[5] = msg->wrench.torque.z;

  if (msg->goal_time < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cartesian wrench command goal_time must be >= 0.0.");
    return;
  }

  const double interpolation_space_command_value = interpolation_space_to_command_value(msg->interpolation_space);
  if (interpolation_space_command_value < 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cartesian wrench command interpolation_space must be either 'joint' or 'cartesian'. Got '%s'.",
      msg->interpolation_space.c_str());
    return;
  }

  command.goal_time = msg->goal_time;
  command.interpolation_space = interpolation_space_command_value;
  command.id = ++next_command_id_;
  command_buffer_.writeFromNonRT(command);
}

double CartesianExternalEffortController::interpolation_space_to_command_value(const std::string & value)
{
  if (value == "joint") {
    return 0.0;
  }
  if (value == "cartesian") {
    return 1.0;
  }
  return -1.0;
}

bool CartesianExternalEffortController::set_command_interface(
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
  trossen_arm_controllers::CartesianExternalEffortController,
  controller_interface::ControllerInterface)
