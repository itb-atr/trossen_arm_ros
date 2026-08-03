// Copyright 2026
// BSD-3-Clause, matching the surrounding Trossen Robotics packages.

#include "trossen_arm_controllers/cartesian_position.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <utility>

namespace
{

bool cartesian_pose_command_values_are_finite(
  const trossen_arm_msgs::msg::CartesianPoseCommand & msg)
{
  return std::isfinite(msg.pose.position.x) &&
         std::isfinite(msg.pose.position.y) &&
         std::isfinite(msg.pose.position.z) &&
         std::isfinite(msg.pose.orientation.x) &&
         std::isfinite(msg.pose.orientation.y) &&
         std::isfinite(msg.pose.orientation.z) &&
         std::isfinite(msg.pose.orientation.w) &&
         std::isfinite(msg.goal_time);
}

bool quaternion_to_rotation_vector(
  const geometry_msgs::msg::Quaternion & quaternion,
  std::array<double, 3> & rotation_vector)
{
  double x = quaternion.x;
  double y = quaternion.y;
  double z = quaternion.z;
  double w = quaternion.w;

  const double norm = std::sqrt((x * x) + (y * y) + (z * z) + (w * w));
  if (!std::isfinite(norm) || norm < 1e-9) {
    return false;
  }

  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  // q and -q encode the same orientation. Select the representation with a
  // non-negative scalar part so the resulting angle-axis vector follows the
  // shortest rotation (angle in [0, pi]).
  if (w < 0.0) {
    x = -x;
    y = -y;
    z = -z;
    w = -w;
  }

  const double vector_norm = std::sqrt((x * x) + (y * y) + (z * z));
  if (vector_norm < 1e-12) {
    rotation_vector = {0.0, 0.0, 0.0};
    return true;
  }

  const double clamped_w = std::max(-1.0, std::min(1.0, w));
  const double angle = 2.0 * std::atan2(vector_norm, clamped_w);
  const double scale = angle / vector_norm;
  rotation_vector[0] = x * scale;
  rotation_vector[1] = y * scale;
  rotation_vector[2] = z * scale;

  return std::isfinite(rotation_vector[0]) &&
         std::isfinite(rotation_vector[1]) &&
         std::isfinite(rotation_vector[2]);
}

}  // namespace

namespace trossen_arm_controllers
{

CallbackReturn CartesianPositionController::on_init()
{
  try {
    auto_declare<std::string>("cartesian_interface_name", trossen_arm_hardware::CARTESIAN_COMPONENT_NAME);
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
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (cartesian_interface_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'cartesian_interface_name' must not be empty.");
    return CallbackReturn::ERROR;
  }

  command_subscriber_ = get_node()->create_subscription<trossen_arm_msgs::msg::CartesianPoseCommand>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianPositionController::command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured Cartesian position controller. Publish trossen_arm_msgs/msg/CartesianPoseCommand to '~/command'.");

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
  const trossen_arm_msgs::msg::CartesianPoseCommand::SharedPtr msg)
{
  if (!cartesian_pose_command_values_are_finite(*msg)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cartesian pose command contains a non-finite value.");
    return;
  }

  std::array<double, 3> rotation_vector{};
  if (!quaternion_to_rotation_vector(msg->pose.orientation, rotation_vector)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cartesian pose command contains an invalid orientation quaternion.");
    return;
  }

  Command command;
  command.pose[0] = msg->pose.position.x;
  command.pose[1] = msg->pose.position.y;
  command.pose[2] = msg->pose.position.z;
  command.pose[3] = rotation_vector[0];
  command.pose[4] = rotation_vector[1];
  command.pose[5] = rotation_vector[2];

  if (msg->goal_time < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cartesian pose command goal_time must be >= 0.0.");
    return;
  }

  const double interpolation_space_command_value = interpolation_space_to_command_value(msg->interpolation_space);
  if (interpolation_space_command_value < 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cartesian pose command interpolation_space must be either 'joint' or 'cartesian'. Got '%s'.",
      msg->interpolation_space.c_str());
    return;
  }

  command.goal_time = msg->goal_time;
  command.interpolation_space = interpolation_space_command_value;
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
