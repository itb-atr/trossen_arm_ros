// Copyright 2026 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace trossen_arm_utilities
{

namespace
{

constexpr char NODE_NAME[] = "trossen_arm_recovery";
constexpr char DEFAULT_CONTROLLER_MANAGER[] = "/controller_manager";
constexpr char DEFAULT_HARDWARE_COMPONENT[] = "TrossenArmHardwareInterface";
constexpr double DEFAULT_SERVICE_TIMEOUT_SEC = 5.0;

/**
 * @brief Join controller names for log and service response messages.
 */
std::string join_names(const std::vector<std::string> & names)
{
  std::ostringstream stream;
  for (size_t index = 0; index < names.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << names[index];
  }
  return stream.str();
}

/**
 * @brief Remove empty and duplicate controller names while preserving their order.
 */
std::vector<std::string> sanitize_controller_names(const std::vector<std::string> & names)
{
  std::vector<std::string> sanitized;
  for (const auto & name : names) {
    if (name.empty() || std::find(sanitized.begin(), sanitized.end(), name) != sanitized.end()) {
      continue;
    }
    sanitized.push_back(name);
  }
  return sanitized;
}

}  // namespace

/**
 * @brief Recover a Trossen ros2_control hardware component and restore its controllers.
 */
class TrossenArmRecoveryNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the recovery node and connect it to controller_manager.
   */
  TrossenArmRecoveryNode()
  : Node(NODE_NAME)
  {
    controller_manager_ = normalize_controller_manager_name(
      declare_parameter<std::string>("controller_manager", DEFAULT_CONTROLLER_MANAGER));
    hardware_component_ =
      declare_parameter<std::string>("hardware_component", DEFAULT_HARDWARE_COMPONENT);
    service_timeout_sec_ =
      declare_parameter<double>("service_timeout", DEFAULT_SERVICE_TIMEOUT_SEC);
    default_controllers_ = sanitize_controller_names(
      declare_parameter<std::vector<std::string>>(
        "default_controllers",
        std::vector<std::string>{
          "joint_state_broadcaster", "arm_controller", "gripper_controller"}));

    if (service_timeout_sec_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'service_timeout' must be positive. Using %.1f seconds.",
        DEFAULT_SERVICE_TIMEOUT_SEC);
      service_timeout_sec_ = DEFAULT_SERVICE_TIMEOUT_SEC;
    }

    client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    set_hardware_state_client_ =
      create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
      controller_manager_service("set_hardware_component_state"),
      rclcpp::ServicesQoS(),
      client_callback_group_);
    switch_controller_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      controller_manager_service("switch_controller"),
      rclcpp::ServicesQoS(),
      client_callback_group_);
    list_controllers_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
      controller_manager_service("list_controllers"),
      rclcpp::ServicesQoS(),
      client_callback_group_);
    load_controller_client_ = create_client<controller_manager_msgs::srv::LoadController>(
      controller_manager_service("load_controller"),
      rclcpp::ServicesQoS(),
      client_callback_group_);
    configure_controller_client_ = create_client<controller_manager_msgs::srv::ConfigureController>(
      controller_manager_service("configure_controller"),
      rclcpp::ServicesQoS(),
      client_callback_group_);

    activity_subscription_ =
      create_subscription<controller_manager_msgs::msg::ControllerManagerActivity>(
      controller_manager_topic("activity"),
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&TrossenArmRecoveryNode::handle_activity, this, std::placeholders::_1));

    recover_service_ = create_service<std_srvs::srv::Trigger>(
      "~/recover",
      std::bind(
        &TrossenArmRecoveryNode::handle_recover,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Trossen Arm recovery utility ready for hardware component '%s' using controller "
      "manager '%s'.",
      hardware_component_.c_str(),
      controller_manager_.c_str());
  }

private:
  using ControllerStates = std::map<std::string, std::string>;

  /**
   * @brief Normalize the configured controller_manager node name.
   */
  std::string normalize_controller_manager_name(const std::string & name) const
  {
    std::string normalized = name.empty() ? DEFAULT_CONTROLLER_MANAGER : name;
    if (normalized.front() != '/') {
      normalized.insert(normalized.begin(), '/');
    }
    while (normalized.size() > 1 && normalized.back() == '/') {
      normalized.pop_back();
    }
    return normalized;
  }

  /**
   * @brief Build a fully qualified controller_manager service name.
   */
  std::string controller_manager_service(const std::string & service_name) const
  {
    return controller_manager_ + "/" + service_name;
  }

  /**
   * @brief Build a fully qualified controller_manager topic name.
   */
  std::string controller_manager_topic(const std::string & topic_name) const
  {
    return controller_manager_ + "/" + topic_name;
  }

  /**
   * @brief Return the timeout used for controller_manager service discovery and calls.
   */
  std::chrono::duration<double> service_timeout() const
  {
    return std::chrono::duration<double>(service_timeout_sec_);
  }

  /**
   * @brief Call a controller_manager service with bounded discovery and response waits.
   */
  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call_service(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request,
    const std::string & service_name,
    std::string & error_message)
  {
    if (!client->wait_for_service(service_timeout())) {
      error_message = "Service '" + service_name + "' is not available.";
      return nullptr;
    }

    auto future = client->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      error_message = "Timed out waiting for service '" + service_name + "'.";
      return nullptr;
    }

    try {
      return future.get();
    } catch (const std::exception & exception) {
      error_message = "Service '" + service_name + "' failed: " + exception.what();
      return nullptr;
    }
  }

  /**
   * @brief Track the last active controller set while the target hardware is healthy.
   */
  void handle_activity(
    const controller_manager_msgs::msg::ControllerManagerActivity::SharedPtr message)
  {
    if (recovery_in_progress_) {
      return;
    }

    const auto hardware = std::find_if(
      message->hardware_components.begin(),
      message->hardware_components.end(),
      [this](const auto & component) {return component.name == hardware_component_;});
    if (hardware == message->hardware_components.end()) {
      return;
    }

    std::vector<std::string> active_controllers;
    for (const auto & controller : message->controllers) {
      if (controller.state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        active_controllers.push_back(controller.name);
      }
    }
    active_controllers = sanitize_controller_names(active_controllers);

    std::lock_guard<std::mutex> lock(activity_mutex_);
    const bool hardware_is_active =
      hardware->state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;

    if (hardware_is_active) {
      if (!active_controllers.empty()) {
        last_active_controllers_ = std::move(active_controllers);
        has_active_controller_snapshot_ = true;
      }
      hardware_was_active_ = true;
      return;
    }

    if (!active_controllers.empty() && hardware_was_active_) {
      last_active_controllers_ = std::move(active_controllers);
      has_active_controller_snapshot_ = true;
    }
    hardware_was_active_ = false;
  }

  /**
   * @brief Handle the public recovery service without allowing concurrent recoveries.
   */
  void handle_recover(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::unique_lock<std::mutex> recovery_lock(recovery_mutex_, std::try_to_lock);
    if (!recovery_lock.owns_lock()) {
      response->success = false;
      response->message = "Trossen Arm recovery is already in progress.";
      return;
    }

    recovery_in_progress_ = true;
    std::string recovery_message;
    bool recovered = false;
    try {
      recovered = recover(recovery_message);
    } catch (const std::exception & exception) {
      recovery_message = std::string("Unexpected error during Trossen Arm recovery: ") +
        exception.what();
      RCLCPP_ERROR(get_logger(), "%s", recovery_message.c_str());
    } catch (...) {
      recovery_message = "Unknown error during Trossen Arm recovery.";
      RCLCPP_ERROR(get_logger(), "%s", recovery_message.c_str());
    }
    recovery_in_progress_ = false;

    response->success = recovered;
    response->message = recovery_message;
  }

  /**
   * @brief Execute the hardware recovery and controller restoration sequence.
   */
  bool recover(std::string & recovery_message)
  {
    std::vector<std::string> remembered_controllers;
    {
      std::lock_guard<std::mutex> lock(activity_mutex_);
      if (has_active_controller_snapshot_) {
        remembered_controllers = last_active_controllers_;
      }
    }

    std::string error_message;
    if (!activate_hardware(error_message)) {
      recovery_message = "Failed to recover Trossen Arm hardware: " + error_message;
      return false;
    }

    if (!remembered_controllers.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "Restoring previously active controllers: [%s].",
        join_names(remembered_controllers).c_str());

      if (activate_existing_controllers(remembered_controllers, error_message)) {
        refresh_active_controller_snapshot();
        recovery_message =
          "Trossen Arm hardware recovered and previously active controllers restored.";
        return true;
      }

      RCLCPP_WARN(
        get_logger(),
        "Failed to restore previously active controllers: %s Falling back to configured "
        "default controllers.",
        error_message.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No previously active controller set is available. Falling back to configured default "
        "controllers.");
    }

    if (default_controllers_.empty()) {
      recovery_message =
        "Trossen Arm hardware recovered, but no default controllers are configured for "
        "fallback recovery.";
      return false;
    }

    error_message.clear();
    if (!restore_default_controllers(error_message)) {
      recovery_message =
        "Trossen Arm hardware recovered, but controller recovery failed: " + error_message;
      return false;
    }

    refresh_active_controller_snapshot();
    recovery_message = "Trossen Arm hardware and fallback controllers recovered successfully.";
    return true;
  }

  /**
   * @brief Request the target hardware component to become active.
   */
  bool activate_hardware(std::string & error_message)
  {
    auto request =
      std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
    request->name = hardware_component_;
    request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    request->target_state.label = "active";

    const auto service_name = controller_manager_service("set_hardware_component_state");
    const auto response = call_service<controller_manager_msgs::srv::SetHardwareComponentState>(
      set_hardware_state_client_, request, service_name, error_message);
    if (!response) {
      return false;
    }
    if (!response->ok ||
      response->state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      error_message =
        "Controller Manager could not set hardware component '" + hardware_component_ +
        "' to active. Current state is '" + response->state.label + "'.";
      return false;
    }

    return true;
  }

  /**
   * @brief Query the current lifecycle state of all loaded controllers.
   */
  bool list_controller_states(ControllerStates & states, std::string & error_message)
  {
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    const auto service_name = controller_manager_service("list_controllers");
    const auto response = call_service<controller_manager_msgs::srv::ListControllers>(
      list_controllers_client_, request, service_name, error_message);
    if (!response) {
      return false;
    }

    states.clear();
    for (const auto & controller : response->controller) {
      states[controller.name] = controller.state;
    }
    return true;
  }

  /**
   * @brief Activate controllers that are already loaded and configured.
   */
  bool activate_existing_controllers(
    const std::vector<std::string> & controllers,
    std::string & error_message)
  {
    ControllerStates states;
    if (!list_controller_states(states, error_message)) {
      return false;
    }

    std::vector<std::string> controllers_to_activate;
    for (const auto & controller : controllers) {
      const auto state = states.find(controller);
      if (state == states.end()) {
        error_message = "Controller '" + controller + "' is not loaded.";
        return false;
      }
      if (state->second == "active") {
        continue;
      }
      if (state->second != "inactive") {
        error_message =
          "Controller '" + controller + "' is in state '" + state->second +
          "' instead of inactive.";
        return false;
      }
      controllers_to_activate.push_back(controller);
    }

    if (controllers_to_activate.empty()) {
      return true;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = controllers_to_activate;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout = rclcpp::Duration::from_seconds(service_timeout_sec_);

    const auto service_name = controller_manager_service("switch_controller");
    const auto response = call_service<controller_manager_msgs::srv::SwitchController>(
      switch_controller_client_, request, service_name, error_message);
    if (!response) {
      return false;
    }
    if (!response->ok) {
      error_message = response->message.empty() ?
        "Controller Manager rejected the controller activation." : response->message;
      return false;
    }

    return true;
  }

  /**
   * @brief Load a fallback controller that is not currently loaded.
   */
  bool load_controller(const std::string & controller, std::string & error_message)
  {
    auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
    request->name = controller;

    const auto service_name = controller_manager_service("load_controller");
    const auto response = call_service<controller_manager_msgs::srv::LoadController>(
      load_controller_client_, request, service_name, error_message);
    if (!response) {
      return false;
    }
    if (!response->ok) {
      error_message = "Controller Manager could not load controller '" + controller + "'.";
      return false;
    }
    return true;
  }

  /**
   * @brief Configure a loaded fallback controller before activation.
   */
  bool configure_controller(const std::string & controller, std::string & error_message)
  {
    auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
    request->name = controller;

    const auto service_name = controller_manager_service("configure_controller");
    const auto response = call_service<controller_manager_msgs::srv::ConfigureController>(
      configure_controller_client_, request, service_name, error_message);
    if (!response) {
      return false;
    }
    if (!response->ok) {
      error_message = "Controller Manager could not configure controller '" + controller + "'.";
      return false;
    }
    return true;
  }

  /**
   * @brief Load, configure, and activate the configured fallback controller set.
   */
  bool restore_default_controllers(std::string & error_message)
  {
    RCLCPP_INFO(
      get_logger(),
      "Restoring fallback controllers: [%s].",
      join_names(default_controllers_).c_str());

    ControllerStates states;
    if (!list_controller_states(states, error_message)) {
      return false;
    }

    for (const auto & controller : default_controllers_) {
      if (states.find(controller) == states.end()) {
        if (!load_controller(controller, error_message)) {
          return false;
        }
      }
    }

    if (!list_controller_states(states, error_message)) {
      return false;
    }

    for (const auto & controller : default_controllers_) {
      const auto state = states.find(controller);
      if (state == states.end()) {
        error_message = "Controller '" + controller + "' is still unavailable after loading.";
        return false;
      }
      if (state->second == "unconfigured") {
        if (!configure_controller(controller, error_message)) {
          return false;
        }
      } else if (state->second != "inactive" && state->second != "active") {
        error_message =
          "Fallback controller '" + controller + "' is in unrecoverable state '" +
          state->second + "'.";
        return false;
      }
    }

    return activate_existing_controllers(default_controllers_, error_message);
  }

  /**
   * @brief Refresh the remembered controller set after a successful recovery.
   */
  void refresh_active_controller_snapshot()
  {
    ControllerStates states;
    std::string error_message;
    if (!list_controller_states(states, error_message)) {
      RCLCPP_WARN(
        get_logger(),
        "Recovery succeeded, but the active controller snapshot could not be refreshed: %s",
        error_message.c_str());
      return;
    }

    std::vector<std::string> active_controllers;
    for (const auto & controller_state : states) {
      if (controller_state.second == "active") {
        active_controllers.push_back(controller_state.first);
      }
    }

    if (active_controllers.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(activity_mutex_);
    last_active_controllers_ = std::move(active_controllers);
    has_active_controller_snapshot_ = true;
    hardware_was_active_ = true;
  }

  std::string controller_manager_;
  std::string hardware_component_;
  double service_timeout_sec_{DEFAULT_SERVICE_TIMEOUT_SEC};
  std::vector<std::string> default_controllers_;

  std::mutex activity_mutex_;
  std::vector<std::string> last_active_controllers_;
  bool has_active_controller_snapshot_{false};
  bool hardware_was_active_{false};

  std::mutex recovery_mutex_;
  std::atomic_bool recovery_in_progress_{false};

  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    set_hardware_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr
    load_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    configure_controller_client_;
  rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr
    activity_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_service_;
};

}  // namespace trossen_arm_utilities

/**
 * @brief Run the Trossen Arm recovery utility using a multithreaded executor.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<trossen_arm_utilities::TrossenArmRecoveryNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
