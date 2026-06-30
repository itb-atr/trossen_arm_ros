// Copyright 2025 Trossen Robotics
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

#include "trossen_arm_hardware/interface.hpp"

namespace trossen_arm_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
TrossenArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Get robot model
  try {
    robot_model_ = trossen_arm::Model(std::stoi(info.hardware_parameters.at("robot_model")));
    RCLCPP_INFO(
      get_logger(),
      "Parameter 'robot_model' set to '%d'.",
      static_cast<int>(robot_model_));
  } catch (const std::out_of_range & /*e*/) {
    RCLCPP_FATAL(
      get_logger(),
      "Required parameter 'robot_model' not specified.");
    return CallbackReturn::FAILURE;
  } catch (const std::invalid_argument & /*e*/) {
    RCLCPP_FATAL(
      get_logger(),
      "Invalid 'robot_model' value specified: '%d'.", static_cast<int>(robot_model_));
    return CallbackReturn::FAILURE;
  }

  // Get robot end effector
  try {
    end_effector_str_ = info.hardware_parameters.at("end_effector");
    if (end_effector_str_ == END_EFFECTOR_BASE) {
      end_effector_ = trossen_arm::StandardEndEffector::wxai_v0_base;
    } else if (end_effector_str_ == END_EFFECTOR_FOLLOWER) {
      end_effector_ = trossen_arm::StandardEndEffector::wxai_v0_follower;
    } else if (end_effector_str_ == END_EFFECTOR_LEADER) {
      end_effector_ = trossen_arm::StandardEndEffector::wxai_v0_leader;
    } else {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid 'end_effector' value specified: '%s'.", end_effector_str_.c_str());
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
      get_logger(),
      "Parameter 'end_effector' set to '%s'.", end_effector_str_.c_str());
  } catch (const std::out_of_range & /*e*/) {
    RCLCPP_FATAL(
      get_logger(),
      "Required parameter 'end_effector' not specified.");
    return CallbackReturn::FAILURE;
  }

  // Get robot IP address
  try {
    driver_ip_address_ = info.hardware_parameters.at("ip_address");
    RCLCPP_INFO(
      get_logger(),
      "Parameter 'ip_address' set to '%s'.",
      driver_ip_address_.c_str());
  } catch (const std::out_of_range & /*e*/) {
    RCLCPP_FATAL(
      get_logger(),
      "Parameter 'ip_address' not specified. Defaulting to '%s'.",
      DRIVER_IP_ADDRESS_DEFAULT);
    driver_ip_address_ = DRIVER_IP_ADDRESS_DEFAULT;
  }

  // Joint state interfaces
  joint_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Joint command interfaces
  joint_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_external_effort_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Cartesian state and command interfaces.
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  cartesian_positions_.fill(nan);
  cartesian_velocities_.fill(nan);
  cartesian_external_efforts_.fill(nan);

  cartesian_position_commands_.fill(0.0);
  cartesian_position_goal_time_command_ = 0.0;
  cartesian_position_interpolation_space_command_ = 1.0;
  cartesian_position_command_id_ = 0.0;
  last_cartesian_position_command_id_ = 0.0;

  cartesian_external_effort_commands_.fill(0.0);
  cartesian_external_effort_goal_time_command_ = 0.0;
  cartesian_external_effort_interpolation_space_command_ = 1.0;
  cartesian_external_effort_command_id_ = 0.0;
  last_cartesian_external_effort_command_id_ = 0.0;

  emergency_stop_engage_command_ = 0.0;
  emergency_stop_release_command_ = 0.0;
  emergency_stop_command_id_ = 0.0;
  last_emergency_stop_command_id_ = 0.0;
  emergency_stop_engaged_ = false;
  normal_commands_suspended_after_emergency_stop_ = false;

  for (const auto & joint : info_.joints) {
    // Each joint has 3 command interfaces: position, velocity, external effort (in that order)
    // Expect exactly three command interfaces
    if (joint.command_interfaces.size() != COUNT_COMMAND_INTERFACES_) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has %zu command interfaces found. %zu expected.",
        joint.name.c_str(),
        joint.command_interfaces.size(),
        COUNT_COMMAND_INTERFACES_);
      return CallbackReturn::ERROR;
    }

    // Position first
    if (joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_POSITION_).name != HW_IF_POSITION) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_POSITION_).name.c_str(),
        INDEX_COMMAND_INTERFACE_POSITION_,
        HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // Velocity second
    if (joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_VELOCITY_).name != HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_VELOCITY_).name.c_str(),
        INDEX_COMMAND_INTERFACE_VELOCITY_,
        HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // External effort third
    if (
      joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_EXTERNAL_EFFORT_).name !=
      HW_IF_EXTERNAL_EFFORT)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_EXTERNAL_EFFORT_).name.c_str(),
        INDEX_COMMAND_INTERFACE_EXTERNAL_EFFORT_,
        HW_IF_EXTERNAL_EFFORT);
      return CallbackReturn::ERROR;
    }

    // Each joint has 3 state interfaces: position, velocity, effort (in that order)
    // Expect exactly three state interfaces
    if (joint.state_interfaces.size() != COUNT_STATE_INTERFACES_) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has %zu state interfaces found. %zu expected.",
        joint.name.c_str(),
        joint.state_interfaces.size(),
        COUNT_STATE_INTERFACES_);
      return CallbackReturn::ERROR;
    }

    // Position first
    if (joint.state_interfaces.at(INDEX_STATE_INTERFACE_POSITION_).name != HW_IF_POSITION) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(INDEX_STATE_INTERFACE_POSITION_).name.c_str(),
        INDEX_STATE_INTERFACE_POSITION_,
        HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // Velocity second
    if (joint.state_interfaces.at(INDEX_STATE_INTERFACE_VELOCITY_).name != HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(INDEX_STATE_INTERFACE_VELOCITY_).name.c_str(),
        INDEX_STATE_INTERFACE_VELOCITY_,
        HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // Effort third
    if (joint.state_interfaces.at(INDEX_STATE_INTERFACE_EFFORT_).name != HW_IF_EFFORT) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(INDEX_STATE_INTERFACE_EFFORT_).name.c_str(),
        INDEX_STATE_INTERFACE_EFFORT_,
        HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TrossenArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION, &joint_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &joint_velocities_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT, &joint_efforts_[i]);
  }

  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_X, &cartesian_positions_[0]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_Y, &cartesian_positions_[1]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_Z, &cartesian_positions_[2]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_RX, &cartesian_positions_[3]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_RY, &cartesian_positions_[4]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_RZ, &cartesian_positions_[5]);

  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_VELOCITY_X, &cartesian_velocities_[0]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_VELOCITY_Y, &cartesian_velocities_[1]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_VELOCITY_Z, &cartesian_velocities_[2]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_VELOCITY_RX, &cartesian_velocities_[3]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_VELOCITY_RY, &cartesian_velocities_[4]);
  state_interfaces.emplace_back(CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_VELOCITY_RZ, &cartesian_velocities_[5]);

  state_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_FX, &cartesian_external_efforts_[0]);
  state_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_FY, &cartesian_external_efforts_[1]);
  state_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_FZ, &cartesian_external_efforts_[2]);
  state_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_TX, &cartesian_external_efforts_[3]);
  state_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_TY, &cartesian_external_efforts_[4]);
  state_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_TZ, &cartesian_external_efforts_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TrossenArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION, &joint_position_commands_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &joint_velocity_commands_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, HW_IF_EXTERNAL_EFFORT, &joint_external_effort_commands_[i]);
  }

  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_X, &cartesian_position_commands_[0]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_Y, &cartesian_position_commands_[1]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_Z, &cartesian_position_commands_[2]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_RX, &cartesian_position_commands_[3]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_RY, &cartesian_position_commands_[4]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_RZ, &cartesian_position_commands_[5]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_GOAL_TIME,
    &cartesian_position_goal_time_command_);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_INTERPOLATION_SPACE,
    &cartesian_position_interpolation_space_command_);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_POSITION_COMMAND_ID,
    &cartesian_position_command_id_);

  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_FX,
    &cartesian_external_effort_commands_[0]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_FY,
    &cartesian_external_effort_commands_[1]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_FZ,
    &cartesian_external_effort_commands_[2]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_TX,
    &cartesian_external_effort_commands_[3]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_TY,
    &cartesian_external_effort_commands_[4]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_TZ,
    &cartesian_external_effort_commands_[5]);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_GOAL_TIME,
    &cartesian_external_effort_goal_time_command_);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_INTERPOLATION_SPACE,
    &cartesian_external_effort_interpolation_space_command_);
  command_interfaces.emplace_back(
    CARTESIAN_COMPONENT_NAME, HW_IF_CARTESIAN_EXTERNAL_EFFORT_COMMAND_ID,
    &cartesian_external_effort_command_id_);

  command_interfaces.emplace_back(
    EMERGENCY_STOP_COMPONENT_NAME, HW_IF_EMERGENCY_STOP_ENGAGE,
    &emergency_stop_engage_command_);
  command_interfaces.emplace_back(
    EMERGENCY_STOP_COMPONENT_NAME, HW_IF_EMERGENCY_STOP_RELEASE,
    &emergency_stop_release_command_);
  command_interfaces.emplace_back(
    EMERGENCY_STOP_COMPONENT_NAME, HW_IF_EMERGENCY_STOP_COMMAND_ID,
    &emergency_stop_command_id_);

  return command_interfaces;
}

CallbackReturn
TrossenArmHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring the Trossen Arm Driver...");
  try {
    arm_driver_ = std::make_unique<TrossenArmDriver>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create TrossenArmDriver: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (!arm_driver_) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create TrossenArmDriver.");
    return CallbackReturn::ERROR;
  }

  try {
    arm_driver_->configure(robot_model_, end_effector_, driver_ip_address_.c_str(), true);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to configure TrossenArmDriver: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Update the robot output
  robot_output_ = arm_driver_->get_robot_output();

  RCLCPP_INFO(
    get_logger(),
    "TrossenArmDriver configured with model %d, IP Address '%s', End Effector '%s'.",
    static_cast<int>(robot_model_),
    driver_ip_address_.c_str(),
    end_effector_str_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TrossenArmHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  first_update_ = true;

  // Update the state of the robot
  this->read(rclcpp::Time(0.0), rclcpp::Duration(0, 0));

  RCLCPP_INFO(get_logger(), "TrossenArmDriver enabled.");

  return CallbackReturn::SUCCESS;
}

return_type
TrossenArmHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  robot_output_ = arm_driver_->get_robot_output();

  // Get joint positions
  joint_positions_ = robot_output_.joint.all.positions;

  // Get joint velocities
  joint_velocities_ = robot_output_.joint.all.velocities;

  // Get joint efforts
  joint_efforts_ = robot_output_.joint.all.efforts;

  cartesian_positions_ = robot_output_.cartesian.positions;
  cartesian_velocities_ = robot_output_.cartesian.velocities;
  cartesian_external_efforts_ = robot_output_.cartesian.external_efforts;

  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // If first time writing to the hardware, set all commands to the current positions
  if (first_update_) {
    RCLCPP_DEBUG(
      get_logger(),
      "First write update. Setting joint position commands to current positions.");
    joint_position_commands_ = joint_positions_;
    cartesian_position_commands_ = cartesian_positions_;
    first_update_ = false;
  }

  try {
    if (is_new_command(emergency_stop_command_id_, last_emergency_stop_command_id_)) {
      if (emergency_stop_engage_command_ > 0.5 && emergency_stop_release_command_ <= 0.5) {
        const auto result = engage_emergency_stop();
        if (result != return_type::OK) {
          return result;
        }
      } else if (emergency_stop_release_command_ > 0.5 && emergency_stop_engage_command_ <= 0.5) {
        const auto result = release_emergency_stop();
        if (result != return_type::OK) {
          return result;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Invalid emergency stop command: exactly one of engage or release must be set.");
        return return_type::ERROR;
      }
      last_emergency_stop_command_id_ = emergency_stop_command_id_;
    }

    if (emergency_stop_engaged_) {
      std::fill(joint_external_effort_commands_.begin(), joint_external_effort_commands_.end(), 0.0);
      arm_driver_->set_all_external_efforts(joint_external_effort_commands_, 0.0, false);
      return return_type::OK;
    }

    if (normal_commands_suspended_after_emergency_stop_) {
      return return_type::OK;
    }

    if (arm_position_mode_running_) {
      arm_driver_->set_all_positions(joint_position_commands_, 0.0, false);
    } else if (arm_velocity_mode_running_) {
      RCLCPP_ERROR(get_logger(), "Velocity mode not implemented yet.");
      return return_type::ERROR;
    } else if (arm_external_effort_mode_running_) {
      arm_driver_->set_all_external_efforts(joint_external_effort_commands_, 0.0, false);
    } else if (cartesian_position_mode_running_) {
      if (!is_new_command(cartesian_position_command_id_, last_cartesian_position_command_id_)) {
        return return_type::OK;
      }

      if (!all_finite(cartesian_position_commands_)) {
        RCLCPP_ERROR(get_logger(), "Cartesian position command contains a non-finite value.");
        return return_type::ERROR;
      }

      if (!std::isfinite(cartesian_position_goal_time_command_) ||
        cartesian_position_goal_time_command_ < 0.0)
      {
        RCLCPP_ERROR(get_logger(), "Cartesian position goal_time must be finite and >= 0.0.");
        return return_type::ERROR;
      }

      arm_driver_->set_cartesian_positions(
        cartesian_position_commands_,
        interpolation_space_from_command(cartesian_position_interpolation_space_command_),
        cartesian_position_goal_time_command_,
        false);
      last_cartesian_position_command_id_ = cartesian_position_command_id_;
    } else if (cartesian_external_effort_mode_running_) {
      if (!is_new_command(
          cartesian_external_effort_command_id_, last_cartesian_external_effort_command_id_))
      {
        return return_type::OK;
      }

      if (!all_finite(cartesian_external_effort_commands_)) {
        RCLCPP_ERROR(get_logger(), "Cartesian external effort command contains a non-finite value.");
        return return_type::ERROR;
      }

      if (!std::isfinite(cartesian_external_effort_goal_time_command_) ||
        cartesian_external_effort_goal_time_command_ < 0.0)
      {
        RCLCPP_ERROR(get_logger(), "Cartesian external effort goal_time must be finite and >= 0.0.");
        return return_type::ERROR;
      }

      arm_driver_->set_cartesian_external_efforts(
        cartesian_external_effort_commands_,
        interpolation_space_from_command(cartesian_external_effort_interpolation_space_command_),
        cartesian_external_effort_goal_time_command_,
        false);
      last_cartesian_external_effort_command_id_ = cartesian_external_effort_command_id_;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to write command to TrossenArmDriver: %s", e.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // If nothing new requested, nothing to validate
  if (start_interfaces.empty()) {
    RCLCPP_DEBUG(get_logger(), "No start interfaces requested. Nothing to prepare.");
    return return_type::OK;
  }

  std::string requested_mode;
  for (const auto & iface : start_interfaces) {
    const auto slash_pos = iface.rfind('/');
    const std::string type = (slash_pos == std::string::npos) ? iface : iface.substr(slash_pos + 1);
    const std::string mode = command_mode_from_interface_type(type);

    if (mode.empty()) {
      RCLCPP_ERROR(get_logger(), "Unsupported command interface '%s' requested.", type.c_str());
      return return_type::ERROR;
    }

    if (requested_mode.empty()) {
      requested_mode = mode;
    } else if (requested_mode != mode) {
      RCLCPP_ERROR(
        get_logger(), "Mixed command modes requested in a single mode switch: '%s' and '%s'.",
        requested_mode.c_str(), mode.c_str());
      return return_type::ERROR;
    }
  }

  if (requested_mode == HW_IF_EMERGENCY_STOP) {
    RCLCPP_DEBUG(get_logger(), "Emergency stop controller command interfaces requested.");
    return return_type::OK;
  }

  if (requested_mode == HW_IF_VELOCITY) {
    RCLCPP_ERROR(get_logger(), "Velocity mode requested but not implemented.");
    return return_type::ERROR;
  }

  const std::set<std::string> active_modes = {
    arm_position_mode_running_ ? std::string(HW_IF_POSITION) : std::string(),
    arm_velocity_mode_running_ ? std::string(HW_IF_VELOCITY) : std::string(),
    arm_external_effort_mode_running_ ? std::string(HW_IF_EXTERNAL_EFFORT) : std::string(),
    cartesian_position_mode_running_ ? std::string(HW_IF_CARTESIAN_POSITION) : std::string(),
    cartesian_external_effort_mode_running_ ? std::string(HW_IF_CARTESIAN_EXTERNAL_EFFORT) : std::string()};

  for (const auto & active_mode : active_modes) {
    if (active_mode.empty() || active_mode == requested_mode) {
      continue;
    }

    if (!interface_mode_in_stop(stop_interfaces, active_mode)) {
      RCLCPP_ERROR(
        get_logger(), "Mode '%s' is active but not requested to stop before switching to '%s'.",
        active_mode.c_str(), requested_mode.c_str());
      return return_type::ERROR;
    }
  }

  RCLCPP_DEBUG(
    get_logger(), "Command mode switch preparation successful. New mode requested: '%s'.",
    requested_mode.c_str());
  return return_type::OK;
}

return_type TrossenArmHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto stop_modes = interface_types_from_list(stop_interfaces);
  const auto start_modes = interface_types_from_list(start_interfaces);

  if (stop_modes.count(HW_IF_POSITION)) {
    arm_position_mode_running_ = false;
  }
  if (stop_modes.count(HW_IF_VELOCITY)) {
    arm_velocity_mode_running_ = false;
  }
  if (stop_modes.count(HW_IF_EXTERNAL_EFFORT)) {
    arm_external_effort_mode_running_ = false;
  }
  if (stop_modes.count(HW_IF_CARTESIAN_POSITION)) {
    cartesian_position_mode_running_ = false;
  }
  if (stop_modes.count(HW_IF_CARTESIAN_EXTERNAL_EFFORT)) {
    cartesian_external_effort_mode_running_ = false;
  }
  if (stop_modes.count(HW_IF_EMERGENCY_STOP)) {
    emergency_stop_controller_running_ = false;
    }

  try {
    if (start_modes.count(HW_IF_EMERGENCY_STOP)) {
      emergency_stop_controller_running_ = true;
      RCLCPP_INFO(get_logger(), "Emergency stop controller command interfaces active.");
    }


    if (start_modes.count(HW_IF_POSITION)) {
      normal_commands_suspended_after_emergency_stop_ = false;
      arm_position_mode_running_ = true;
      arm_velocity_mode_running_ = false;
      arm_external_effort_mode_running_ = false;
      cartesian_position_mode_running_ = false;
      cartesian_external_effort_mode_running_ = false;
      joint_position_commands_ = joint_positions_;
      arm_driver_->set_all_modes(trossen_arm::Mode::position);
      RCLCPP_INFO(get_logger(), "Switched to position command mode.");
    } else if (start_modes.count(HW_IF_VELOCITY)) {
      RCLCPP_ERROR(get_logger(), "Velocity mode requested but not implemented.");
      return return_type::ERROR;
    } else if (start_modes.count(HW_IF_EXTERNAL_EFFORT)) {
      normal_commands_suspended_after_emergency_stop_ = false;
      arm_position_mode_running_ = false;
      arm_velocity_mode_running_ = false;
      arm_external_effort_mode_running_ = true;
      cartesian_position_mode_running_ = false;
      cartesian_external_effort_mode_running_ = false;
      std::fill(joint_external_effort_commands_.begin(), joint_external_effort_commands_.end(), 0.0);
      arm_driver_->set_all_modes(trossen_arm::Mode::external_effort);
      RCLCPP_INFO(get_logger(), "Switched to external effort command mode.");
    } else if (start_modes.count(HW_IF_CARTESIAN_POSITION)) {
      normal_commands_suspended_after_emergency_stop_ = false;
      arm_position_mode_running_ = false;
      arm_velocity_mode_running_ = false;
      arm_external_effort_mode_running_ = false;
      cartesian_position_mode_running_ = true;
      cartesian_external_effort_mode_running_ = false;
      cartesian_position_commands_ = cartesian_positions_;
      last_cartesian_position_command_id_ = cartesian_position_command_id_;
      arm_driver_->set_all_modes(trossen_arm::Mode::position);
      RCLCPP_INFO(get_logger(), "Switched to Cartesian position command mode.");
    } else if (start_modes.count(HW_IF_CARTESIAN_EXTERNAL_EFFORT)) {
      normal_commands_suspended_after_emergency_stop_ = false;
      arm_position_mode_running_ = false;
      arm_velocity_mode_running_ = false;
      arm_external_effort_mode_running_ = false;
      cartesian_position_mode_running_ = false;
      cartesian_external_effort_mode_running_ = true;
      cartesian_external_effort_commands_.fill(0.0);
      last_cartesian_external_effort_command_id_ = cartesian_external_effort_command_id_;
      arm_driver_->set_all_modes(trossen_arm::Mode::external_effort);
      RCLCPP_INFO(get_logger(), "Switched to Cartesian external effort command mode.");
    }

    if (start_modes.empty() && !emergency_stop_engaged_ &&
      !arm_position_mode_running_ && !arm_velocity_mode_running_ &&
      !arm_external_effort_mode_running_ && !cartesian_position_mode_running_ &&
      !cartesian_external_effort_mode_running_)
    {
      arm_driver_->set_all_modes(trossen_arm::Mode::idle);
      RCLCPP_INFO(get_logger(), "All command modes stopped. Driver set to idle.");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed during command mode switch: %s", e.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

CallbackReturn
TrossenArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_driver_->set_all_modes(trossen_arm::Mode::idle);

  arm_position_mode_running_ = false;
  arm_velocity_mode_running_ = false;
  arm_external_effort_mode_running_ = false;
  cartesian_position_mode_running_ = false;
  cartesian_external_effort_mode_running_ = false;
  emergency_stop_controller_running_ = false;
  normal_commands_suspended_after_emergency_stop_ = false;

  RCLCPP_INFO(get_logger(), "TrossenArmDriver disabled.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
TrossenArmHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  robot_output_ = trossen_arm::RobotOutput();
  emergency_stop_engaged_ = false;
  normal_commands_suspended_after_emergency_stop_ = false;
  arm_driver_.reset();
  return CallbackReturn::SUCCESS;
}

return_type
TrossenArmHardwareInterface::engage_emergency_stop()
{
  std::fill(joint_external_effort_commands_.begin(), joint_external_effort_commands_.end(), 0.0);
  arm_driver_->set_all_modes(trossen_arm::Mode::external_effort);
  arm_driver_->set_all_external_efforts(joint_external_effort_commands_, 0.0, false);
  emergency_stop_engaged_ = true;
  normal_commands_suspended_after_emergency_stop_ = false;

  RCLCPP_WARN(get_logger(), "Emergency stop engaged. Hardware commands from other controllers are ignored.");
  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::release_emergency_stop()
{
  emergency_stop_engaged_ = false;
  normal_commands_suspended_after_emergency_stop_ = true;

  RCLCPP_WARN(
    get_logger(),
    "Emergency stop released. Motion commands remain suspended until a motion controller is restarted.");
  return return_type::OK;
}

bool TrossenArmHardwareInterface::interface_type_in_stop(
  const std::vector<std::string> & stop_interfaces,
  const std::string & type)
{
  return interface_mode_in_stop(stop_interfaces, type);
}

std::set<std::string> TrossenArmHardwareInterface::interface_types_from_list(
  const std::vector<std::string> & ifaces)
{
  std::set<std::string> types;

  for (const auto & iface : ifaces) {
    const auto slash_pos = iface.rfind('/');
    const std::string type = (slash_pos == std::string::npos) ? iface : iface.substr(slash_pos + 1);
    const std::string mode = command_mode_from_interface_type(type);
    if (!mode.empty()) {
      types.insert(mode);
    }
  }

  return types;
}

std::string TrossenArmHardwareInterface::command_mode_from_interface_type(
  const std::string & type) const
{
  if (type == HW_IF_POSITION || type == HW_IF_VELOCITY || type == HW_IF_EXTERNAL_EFFORT) {
    return type;
  }

  if (type == HW_IF_EMERGENCY_STOP_ENGAGE || type == HW_IF_EMERGENCY_STOP_RELEASE ||
    type == HW_IF_EMERGENCY_STOP_COMMAND_ID)
  {
    return HW_IF_EMERGENCY_STOP;
  }

  if (has_prefix(type, HW_IF_CARTESIAN_POSITION_PREFIX)) {
    return HW_IF_CARTESIAN_POSITION;
  }

  if (has_prefix(type, HW_IF_CARTESIAN_EXTERNAL_EFFORT_PREFIX)) {
    return HW_IF_CARTESIAN_EXTERNAL_EFFORT;
  }

  return {};
}

rclcpp::Logger TrossenArmHardwareInterface::get_logger() const
{
  return rclcpp::get_logger("trossen_arm_hardware");
}

bool TrossenArmHardwareInterface::interface_mode_in_stop(
  const std::vector<std::string> & stop_interfaces,
  const std::string & mode)
{
  for (const auto & iface : stop_interfaces) {
    const auto slash_pos = iface.rfind('/');
    const std::string stop_type =
      (slash_pos == std::string::npos) ? iface : iface.substr(slash_pos + 1);
    if (command_mode_from_interface_type(stop_type) == mode) {
      return true;
    }
  }
  return false;
}

bool TrossenArmHardwareInterface::has_prefix(
  const std::string & value, const std::string & prefix) const
{
  return value.rfind(prefix, 0) == 0;
}

bool TrossenArmHardwareInterface::is_new_command(double command_id, double last_command_id) const
{
  return std::isfinite(command_id) && command_id > 0.0 && command_id != last_command_id;
}

bool TrossenArmHardwareInterface::all_finite(const std::array<double, 6> & values) const
{
  for (const auto value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

trossen_arm::InterpolationSpace TrossenArmHardwareInterface::interpolation_space_from_command(
  double value) const
{
  if (value < 0.5) {
    return trossen_arm::InterpolationSpace::joint;
  }
  return trossen_arm::InterpolationSpace::cartesian;
}

}  // namespace trossen_arm_hardware

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  trossen_arm_hardware::TrossenArmHardwareInterface,
  hardware_interface::SystemInterface)
