// Copyright 2026
// BSD-3-Clause, matching the surrounding Trossen Robotics packages.

#ifndef TROSSEN_ARM_CONTROLLERS__EMERGENCY_STOP_HPP_
#define TROSSEN_ARM_CONTROLLERS__EMERGENCY_STOP_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trossen_arm_hardware/interface.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace trossen_arm_controllers
{

class EmergencyStopController : public controller_interface::ControllerInterface
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
    bool engage{false};
    bool release{false};
    uint64_t id{0};
  };

  static bool set_command_interface(
    hardware_interface::LoanedCommandInterface & command_interface,
    double value,
    const rclcpp::Logger & logger);

  void handle_engage_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_release_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  std::atomic<uint64_t> next_command_id_{0};
  uint64_t last_command_id_{0};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr engage_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
};

}  // namespace trossen_arm_controllers

#endif  // TROSSEN_ARM_CONTROLLERS__EMERGENCY_STOP_HPP_
