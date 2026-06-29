// Copyright 2026
// BSD-3-Clause, matching the surrounding Trossen Robotics packages.

#include "trossen_arm_controllers/cartesian_position.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <utility>

namespace
{
constexpr size_t kExpectedCommandSize = 6;
}

namespace trossen_arm_controllers
{

CallbackReturn CartesianPositionController::on_init()
{
  try {
    auto_declare<std::string>("cartesian_interface_name", trossen_arm_hardware::CARTESIAN_COMPONENT_NAME);
    auto_declare<std::string>("interpolation_space", "cartesian");
    auto_declare<double>("goal_time", 2.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPositionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    cartesian_interface_name_ = get_node()->get_parameter("cartesian_interface_name").as_string();
    interpolation_space_name_ = get_node()->get_parameter("interpolation_space").as_string();
    goal_time_ = get_node()->get_parameter("goal_time").as_double();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (cartesian_interface_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'cartesian_interface_name' must not be empty.");
    return CallbackReturn::ERROR;
  }

  if (!std::isfinite(goal_time_) || goal_time_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'goal_time' must be finite and >= 0.0.");
    return CallbackReturn::ERROR;
  }

  interpolation_space_command_value_ = interpolation_space_to_command_value(interpolation_space_name_);
  if (interpolation_space_command_value_ < 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'interpolation_space' must be either 'joint' or 'cartesian'. Got '%s'.",
      interpolation_space_name_.c_str());
    return CallbackReturn::ERROR;
  }

  command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianPositionController::command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured Cartesian position controller. Publish [x, y, z, rx, ry, rz] to '~/command'.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_command_id_ = 0;
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_POSITION_X,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_POSITION_Y,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_POSITION_Z,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_POSITION_RX,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_POSITION_RY,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_POSITION_RZ,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_POSITION_GOAL_TIME,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_POSITION_INTERPOLATION_SPACE,
    cartesian_interface_name_ + "/" + trossen_arm_hardware::HW_IF_CARTESIAN_POSITION_COMMAND_ID};
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::return_type CartesianPositionController::update(
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

  for (size_t i = 0; i < command->pose.size(); ++i) {
    if (!set_command_interface(command_interfaces_[i], command->pose[i], get_node()->get_logger())) {
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

void CartesianPositionController::command_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != kExpectedCommandSize) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cartesian position command must contain exactly 6 values: [x, y, z, rx, ry, rz]. Got %zu.",
      msg->data.size());
    return;
  }

  Command command;
  for (size_t i = 0; i < kExpectedCommandSize; ++i) {
    if (!std::isfinite(msg->data[i])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Cartesian position command contains a non-finite value.");
      return;
    }
    command.pose[i] = msg->data[i];
  }

  command.goal_time = goal_time_;
  command.interpolation_space = interpolation_space_command_value_;
  command.id = ++next_command_id_;
  command_buffer_.writeFromNonRT(command);
}

double CartesianPositionController::interpolation_space_to_command_value(const std::string & value)
{
  if (value == "joint") {
    return 0.0;
  }
  if (value == "cartesian") {
    return 1.0;
  }
  return -1.0;
}

bool CartesianPositionController::set_command_interface(
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
  trossen_arm_controllers::CartesianPositionController,
  controller_interface::ControllerInterface)
