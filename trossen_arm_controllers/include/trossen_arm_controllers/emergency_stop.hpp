// Copyright 2026
// BSD-3-Clause, matching the surrounding Trossen Robotics packages.

#ifndef TROSSEN_ARM_CONTROLLERS__EMERGENCY_STOP_HPP_
#define TROSSEN_ARM_CONTROLLERS__EMERGENCY_STOP_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
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

  std::string controller_manager_service(const std::string & service_name) const;
  void process_pending_rearm();
  void request_active_controller_list();
  void handle_controller_list(
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future);
  void request_controller_restart(const std::vector<std::string> & controllers);
  void handle_controller_restart(
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future,
    const std::vector<std::string> & controllers);
  void schedule_rearm_retry(const std::string & reason);

  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  std::atomic<uint64_t> next_command_id_{0};
  uint64_t last_command_id_{0};

  std::string controller_manager_{"/controller_manager"};
  std::vector<std::string> rearm_controllers_;
  double controller_switch_timeout_sec_{2.0};

  std::atomic_bool rearm_requested_{false};
  std::atomic_bool rearm_ready_{false};
  std::atomic_bool rearm_in_progress_{false};
  std::atomic_uint rearm_attempt_count_{0};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr engage_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
  rclcpp::TimerBase::SharedPtr rearm_timer_;
};

}  // namespace trossen_arm_controllers

#endif  // TROSSEN_ARM_CONTROLLERS__EMERGENCY_STOP_HPP_
