// Copyright 2026
// BSD-3-Clause, matching the surrounding Trossen Robotics packages.

#include "trossen_arm_controllers/emergency_stop.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace trossen_arm_controllers
{

namespace
{

constexpr char DEFAULT_CONTROLLER_MANAGER[] = "/controller_manager";
constexpr double DEFAULT_CONTROLLER_SWITCH_TIMEOUT_SEC = 2.0;
constexpr unsigned int MAX_REARM_ATTEMPTS = 5;
constexpr std::chrono::milliseconds REARM_POLL_PERIOD{50};

const std::vector<std::string> DEFAULT_REARM_CONTROLLERS{
  "arm_controller",
  "gripper_controller",
  "gravity_compensation_controller",
  "gripper_external_effort_controller",
  "cartesian_position_controller",
  "cartesian_external_effort_controller"};

}  // namespace

CallbackReturn EmergencyStopController::on_init()
{
  try {
    auto_declare<std::string>("controller_manager", DEFAULT_CONTROLLER_MANAGER);
    auto_declare<std::vector<std::string>>("rearm_controllers", DEFAULT_REARM_CONTROLLERS);
    auto_declare<double>(
      "controller_switch_timeout", DEFAULT_CONTROLLER_SWITCH_TIMEOUT_SEC);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to declare emergency-stop controller parameters: %s",
      exception.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EmergencyStopController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  controller_manager_ = get_node()->get_parameter("controller_manager").as_string();
  if (controller_manager_.empty()) {
    controller_manager_ = DEFAULT_CONTROLLER_MANAGER;
  }
  if (controller_manager_.front() != '/') {
    controller_manager_.insert(controller_manager_.begin(), '/');
  }
  while (controller_manager_.size() > 1 && controller_manager_.back() == '/') {
    controller_manager_.pop_back();
  }

  const auto configured_rearm_controllers =
    get_node()->get_parameter("rearm_controllers").as_string_array();
  rearm_controllers_.clear();
  for (const auto & controller : configured_rearm_controllers) {
    if (controller.empty() ||
      std::find(rearm_controllers_.begin(), rearm_controllers_.end(), controller) !=
      rearm_controllers_.end())
    {
      continue;
    }
    rearm_controllers_.push_back(controller);
  }

  controller_switch_timeout_sec_ =
    get_node()->get_parameter("controller_switch_timeout").as_double();
  if (controller_switch_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Parameter 'controller_switch_timeout' must be positive. Using %.1f seconds.",
      DEFAULT_CONTROLLER_SWITCH_TIMEOUT_SEC);
    controller_switch_timeout_sec_ = DEFAULT_CONTROLLER_SWITCH_TIMEOUT_SEC;
  }

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

  list_controllers_client_ =
    get_node()->create_client<controller_manager_msgs::srv::ListControllers>(
    controller_manager_service("list_controllers"));
  switch_controller_client_ =
    get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
    controller_manager_service("switch_controller"));

  rearm_timer_ = get_node()->create_wall_timer(
    REARM_POLL_PERIOD,
    std::bind(&EmergencyStopController::process_pending_rearm, this));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured emergency stop controller with services '~/emergency_stop_engage' and "
    "'~/emergency_stop_release'. Post-release controller rearming is enabled for %zu "
    "configured controller names.",
    rearm_controllers_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn EmergencyStopController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_command_id_ = 0;
  rearm_requested_ = false;
  rearm_ready_ = false;
  rearm_in_progress_ = false;
  rearm_attempt_count_ = 0;
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
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    std::string(trossen_arm_hardware::EMERGENCY_STOP_COMPONENT_NAME) + "/" +
      trossen_arm_hardware::HW_IF_EMERGENCY_STOP_ENGAGED};
  return config;
}

controller_interface::return_type EmergencyStopController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (state_interfaces_.size() != 1) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 1 emergency-stop state interface but got %zu.",
      state_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  const auto * command = command_buffer_.readFromRT();
  if (command != nullptr && command->id != 0 && command->id != last_command_id_) {
    if (command_interfaces_.size() != 3) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Expected 3 command interfaces but got %zu.",
        command_interfaces_.size());
      return controller_interface::return_type::ERROR;
    }

    if (!set_command_interface(
        command_interfaces_[0], command->engage ? 1.0 : 0.0, get_node()->get_logger()) ||
      !set_command_interface(
        command_interfaces_[1], command->release ? 1.0 : 0.0, get_node()->get_logger()) ||
      !set_command_interface(
        command_interfaces_[2], static_cast<double>(command->id), get_node()->get_logger()))
    {
      return controller_interface::return_type::ERROR;
    }

    last_command_id_ = command->id;
  }

  if (rearm_requested_.load(std::memory_order_relaxed)) {
    const auto emergency_stop_engaged = state_interfaces_[0].get_optional<double>();
    if (emergency_stop_engaged.has_value() && emergency_stop_engaged.value() <= 0.5) {
      rearm_requested_.store(false, std::memory_order_relaxed);
      rearm_ready_.store(true, std::memory_order_release);
    }
  }

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

  rearm_requested_ = false;
  rearm_ready_ = false;
  rearm_attempt_count_ = 0;

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

  rearm_attempt_count_ = 0;
  rearm_ready_ = false;
  rearm_requested_ = true;

  response->success = true;
  response->message =
    "Emergency stop release command queued. Active arm controllers will be safely rearmed.";
}

std::string EmergencyStopController::controller_manager_service(
  const std::string & service_name) const
{
  return controller_manager_ + "/" + service_name;
}

void EmergencyStopController::process_pending_rearm()
{
  if (!rearm_ready_.load(std::memory_order_acquire) ||
    rearm_in_progress_.load(std::memory_order_relaxed))
  {
    return;
  }

  if (!list_controllers_client_ || !list_controllers_client_->service_is_ready()) {
    return;
  }

  if (!rearm_ready_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  if (rearm_in_progress_.exchange(true, std::memory_order_acq_rel)) {
    rearm_ready_ = true;
    return;
  }

  ++rearm_attempt_count_;
  request_active_controller_list();
}

void EmergencyStopController::request_active_controller_list()
{
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  list_controllers_client_->async_send_request(
    request,
    [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
      handle_controller_list(std::move(future));
    });
}

void EmergencyStopController::handle_controller_list(
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future)
{
  controller_manager_msgs::srv::ListControllers::Response::SharedPtr response;
  try {
    response = future.get();
  } catch (const std::exception & exception) {
    schedule_rearm_retry(
      std::string("Failed to query active controllers after emergency-stop release: ") +
      exception.what());
    return;
  }

  std::vector<std::string> controllers_to_rearm;
  for (const auto & controller : response->controller) {
    if (controller.state != "active" || controller.name == get_node()->get_name()) {
      continue;
    }
    if (std::find(
        rearm_controllers_.begin(), rearm_controllers_.end(), controller.name) ==
      rearm_controllers_.end())
    {
      continue;
    }
    controllers_to_rearm.push_back(controller.name);
  }

  if (controllers_to_rearm.empty()) {
    rearm_in_progress_ = false;
    rearm_attempt_count_ = 0;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Emergency stop released. No configured active command controllers require rearming.");
    return;
  }

  if (!switch_controller_client_ || !switch_controller_client_->service_is_ready()) {
    schedule_rearm_retry(
      "Controller Manager switch_controller service is not available for post-release rearming.");
    return;
  }

  request_controller_restart(controllers_to_rearm);
}

void EmergencyStopController::request_controller_restart(
  const std::vector<std::string> & controllers)
{
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers = controllers;
  request->activate_controllers = controllers;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration::from_seconds(controller_switch_timeout_sec_);

  switch_controller_client_->async_send_request(
    request,
    [this, controllers](
      rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
      handle_controller_restart(std::move(future), controllers);
    });
}

void EmergencyStopController::handle_controller_restart(
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future,
  const std::vector<std::string> & controllers)
{
  controller_manager_msgs::srv::SwitchController::Response::SharedPtr response;
  try {
    response = future.get();
  } catch (const std::exception & exception) {
    schedule_rearm_retry(
      std::string("Controller restart after emergency-stop release failed: ") + exception.what());
    return;
  }

  if (!response->ok) {
    schedule_rearm_retry(
      response->message.empty() ?
      "Controller Manager rejected the post-release controller restart." : response->message);
    return;
  }

  rearm_in_progress_ = false;
  rearm_attempt_count_ = 0;

  std::string controller_names;
  for (size_t index = 0; index < controllers.size(); ++index) {
    if (index > 0) {
      controller_names += ", ";
    }
    controller_names += controllers[index];
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Emergency-stop release rearmed active command controllers: [%s].",
    controller_names.c_str());
}

void EmergencyStopController::schedule_rearm_retry(const std::string & reason)
{
  rearm_in_progress_ = false;
  const auto attempt = rearm_attempt_count_.load();
  if (attempt >= MAX_REARM_ATTEMPTS) {
    rearm_ready_ = false;
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to rearm controllers after emergency-stop release after %u attempts: %s",
      attempt,
      reason.c_str());
    return;
  }

  rearm_ready_ = true;
  RCLCPP_WARN(
    get_node()->get_logger(),
    "Post-release controller rearm attempt %u failed: %s Retrying.",
    attempt,
    reason.c_str());
}

bool EmergencyStopController::set_command_interface(
  hardware_interface::LoanedCommandInterface & command_interface,
  double value,
  const rclcpp::Logger & logger)
{
  if (!command_interface.set_value(value)) {
    RCLCPP_ERROR(
      logger,
      "Failed to set command interface '%s'.",
      command_interface.get_name().c_str());
    return false;
  }
  return true;
}

}  // namespace trossen_arm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  trossen_arm_controllers::EmergencyStopController,
  controller_interface::ControllerInterface)
