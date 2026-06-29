// Copyright 2026
// BSD-3-Clause, matching the surrounding Trossen Robotics packages.

#ifndef TROSSEN_ARM_CONTROLLERS__CARTESIAN_POSITION_HPP_
#define TROSSEN_ARM_CONTROLLERS__CARTESIAN_POSITION_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trossen_arm_hardware/interface.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace trossen_arm_controllers
{

class CartesianPositionController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct Command
  {
    std::array<double, 6> pose{};  // x, y, z, rx, ry, rz; meters and angle-axis radians.
    double goal_time{0.0};
    double interpolation_space{1.0};  // 0.0 = joint, 1.0 = cartesian.
    uint64_t id{0};
  };

  static double interpolation_space_to_command_value(const std::string & value);
  static bool set_command_interface(
    hardware_interface::LoanedCommandInterface & command_interface,
    double value,
    const rclcpp::Logger & logger);

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::string cartesian_interface_name_{trossen_arm_hardware::CARTESIAN_COMPONENT_NAME};
  std::string interpolation_space_name_{"cartesian"};
  double goal_time_{2.0};
  double interpolation_space_command_value_{1.0};

  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  std::atomic<uint64_t> next_command_id_{0};
  uint64_t last_command_id_{0};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;
};

}  // namespace trossen_arm_controllers

#endif  // TROSSEN_ARM_CONTROLLERS__CARTESIAN_POSITION_HPP_
