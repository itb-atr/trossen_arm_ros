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

#include <chrono>
#include <thread>

namespace trossen_arm_hardware
{

namespace
{

constexpr auto DRIVER_MODE_SWITCH_TIMEOUT = std::chrono::milliseconds(500);
constexpr auto DRIVER_MODE_POLL_INTERVAL = std::chrono::milliseconds(2);

/**
 * @brief Wait until the requested mode is reported for a contiguous group of joints.
 */
void wait_for_driver_mode(
  TrossenArmDriver & arm_driver,
  size_t first_joint_index,
  size_t joint_count,
  trossen_arm::Mode expected_mode,
  const char * group_name,
  const char * mode_name)
{
  const auto deadline = std::chrono::steady_clock::now() + DRIVER_MODE_SWITCH_TIMEOUT;

  while (true) {
    const auto modes = arm_driver.get_modes();
    if (modes.size() < first_joint_index + joint_count) {
      throw std::runtime_error(
              std::string("Driver returned too few joint modes while switching the ") +
              group_name + ".");
    }

    if (std::all_of(
        modes.begin() + first_joint_index,
        modes.begin() + first_joint_index + joint_count,
        [expected_mode](trossen_arm::Mode mode) {return mode == expected_mode;}))
    {
      return;
    }

    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error(
              std::string("Timed out waiting for the ") + group_name + " to enter " +
              mode_name + " mode.");
    }

    std::this_thread::sleep_for(DRIVER_MODE_POLL_INTERVAL);
  }
}

}  // namespace

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

  if (info_.joints.size() < 2) {
    RCLCPP_FATAL(
      get_logger(),
      "The Trossen Arm hardware interface requires at least one arm joint and one gripper "
      "joint, but the ros2_control description contains %zu joints.",
      info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // The driver's all-joint vectors contain the arm joints first and the
  // gripper joint last. Preserve that ordering while allowing ros2_control to
  // switch the two groups independently.
  arm_joint_count_ = info_.joints.size() - 1;
  gripper_joint_index_ = arm_joint_count_;
  gripper_joint_name_ = info_.joints[gripper_joint_index_].name;
  arm_position_command_buffer_.resize(arm_joint_count_, 0.0);
  arm_external_effort_command_buffer_.resize(arm_joint_count_, 0.0);

  RCLCPP_INFO(
    get_logger(),
    "Using %zu arm joints and gripper joint '%s' as independent command groups.",
    arm_joint_count_,
    gripper_joint_name_.c_str());

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
  arm_commands_suspended_after_emergency_stop_ = false;
  gripper_commands_suspended_after_emergency_stop_ = false;

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

  if (static_cast<size_t>(arm_driver_->get_num_joints()) != info_.joints.size()) {
    RCLCPP_FATAL(
      get_logger(),
      "The driver reports %u joints, but the ros2_control description contains %zu. "
      "The arm/gripper command split cannot be applied safely.",
      static_cast<unsigned int>(arm_driver_->get_num_joints()),
      info_.joints.size());
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
  if (this->read(rclcpp::Time(0.0), rclcpp::Duration(0, 0)) != return_type::OK) {
    RCLCPP_ERROR(get_logger(), "Failed to read the initial Trossen Arm state.");
    return CallbackReturn::ERROR;
  }

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
  // Seed every position command from the measured state before any controller
  // can write to the exported interfaces. This prevents a mode activation from
  // replaying the NaN initialization values.
  if (first_update_) {
    RCLCPP_DEBUG(
      get_logger(),
      "First write update. Setting joint and Cartesian position commands to current positions.");
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
      } else if (emergency_stop_release_command_ > 0.5 &&
        emergency_stop_engage_command_ <= 0.5)
      {
        const auto result = release_emergency_stop();
        if (result != return_type::OK) {
          return result;
        }
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Invalid emergency stop command: exactly one of engage or release must be set.");
        return return_type::ERROR;
      }
      last_emergency_stop_command_id_ = emergency_stop_command_id_;
    }

    if (emergency_stop_engaged_) {
      std::fill(
        joint_external_effort_commands_.begin(), joint_external_effort_commands_.end(), 0.0);
      arm_driver_->set_all_external_efforts(joint_external_effort_commands_, 0.0, false);
      return return_type::OK;
    }

    // The arm and gripper have independent mode APIs in the Trossen driver.
    // Never use an all-joint command here: a gripper effort controller must not
    // make the six arm joints compliant.
    if (!arm_commands_suspended_after_emergency_stop_) {
      if (arm_position_mode_running_) {
        std::copy_n(
          joint_position_commands_.begin(), arm_joint_count_,
          arm_position_command_buffer_.begin());
        arm_driver_->set_arm_positions(arm_position_command_buffer_, 0.0, false);
      } else if (arm_velocity_mode_running_) {
        RCLCPP_ERROR(get_logger(), "Arm velocity mode is not implemented yet.");
        return return_type::ERROR;
      } else if (arm_external_effort_mode_running_) {
        std::copy_n(
          joint_external_effort_commands_.begin(), arm_joint_count_,
          arm_external_effort_command_buffer_.begin());
        arm_driver_->set_arm_external_efforts(
          arm_external_effort_command_buffer_, 0.0, false);
      } else if (cartesian_position_mode_running_ &&
        is_new_command(cartesian_position_command_id_, last_cartesian_position_command_id_))
      {
        bool command_is_valid = true;
        if (!all_finite(cartesian_position_commands_)) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring Cartesian position command %.0f because it contains a non-finite value.",
            cartesian_position_command_id_);
          command_is_valid = false;
        } else if (!std::isfinite(cartesian_position_goal_time_command_) ||
          cartesian_position_goal_time_command_ < 0.0)
        {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring Cartesian position command %.0f because goal_time is non-finite or < 0.0.",
            cartesian_position_command_id_);
          command_is_valid = false;
        } else if (!std::isfinite(cartesian_position_interpolation_space_command_)) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring Cartesian position command %.0f because interpolation_space is non-finite.",
            cartesian_position_command_id_);
          command_is_valid = false;
        }

        if (command_is_valid) {
          try {
            arm_driver_->set_cartesian_positions(
              cartesian_position_commands_,
              interpolation_space_from_command(cartesian_position_interpolation_space_command_),
              cartesian_position_goal_time_command_,
              false);
          } catch (const std::exception & e) {
            RCLCPP_ERROR(
              get_logger(),
              "Ignoring Cartesian position command %.0f because the driver rejected it: %s. "
              "Command was [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f], goal_time %.3f.",
              cartesian_position_command_id_,
              e.what(),
              cartesian_position_commands_[0],
              cartesian_position_commands_[1],
              cartesian_position_commands_[2],
              cartesian_position_commands_[3],
              cartesian_position_commands_[4],
              cartesian_position_commands_[5],
              cartesian_position_goal_time_command_);
          }
        }
        last_cartesian_position_command_id_ = cartesian_position_command_id_;
      } else if (cartesian_external_effort_mode_running_ &&
        is_new_command(
          cartesian_external_effort_command_id_, last_cartesian_external_effort_command_id_))
      {
        bool command_is_valid = true;
        if (!all_finite(cartesian_external_effort_commands_)) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring Cartesian external effort command %.0f because it contains a non-finite "
            "value.",
            cartesian_external_effort_command_id_);
          command_is_valid = false;
        } else if (!std::isfinite(cartesian_external_effort_goal_time_command_) ||
          cartesian_external_effort_goal_time_command_ < 0.0)
        {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring Cartesian external effort command %.0f because goal_time is non-finite or "
            "< 0.0.",
            cartesian_external_effort_command_id_);
          command_is_valid = false;
        } else if (!std::isfinite(cartesian_external_effort_interpolation_space_command_)) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring Cartesian external effort command %.0f because interpolation_space is "
            "non-finite.",
            cartesian_external_effort_command_id_);
          command_is_valid = false;
        }

        if (command_is_valid) {
          try {
            arm_driver_->set_cartesian_external_efforts(
              cartesian_external_effort_commands_,
              interpolation_space_from_command(
                cartesian_external_effort_interpolation_space_command_),
              cartesian_external_effort_goal_time_command_,
              false);
          } catch (const std::exception & e) {
            RCLCPP_ERROR(
              get_logger(),
              "Ignoring Cartesian external effort command %.0f because the driver rejected it: "
              "%s. Command was [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f], goal_time %.3f.",
              cartesian_external_effort_command_id_,
              e.what(),
              cartesian_external_effort_commands_[0],
              cartesian_external_effort_commands_[1],
              cartesian_external_effort_commands_[2],
              cartesian_external_effort_commands_[3],
              cartesian_external_effort_commands_[4],
              cartesian_external_effort_commands_[5],
              cartesian_external_effort_goal_time_command_);
          }
        }
        last_cartesian_external_effort_command_id_ = cartesian_external_effort_command_id_;
      }
    }

    if (!gripper_commands_suspended_after_emergency_stop_) {
      if (gripper_position_mode_running_) {
        arm_driver_->set_gripper_position(
          joint_position_commands_[gripper_joint_index_], 0.0, false);
      } else if (gripper_velocity_mode_running_) {
        RCLCPP_ERROR(get_logger(), "Gripper velocity mode is not implemented yet.");
        return return_type::ERROR;
      } else if (gripper_external_effort_mode_running_) {
        arm_driver_->set_gripper_external_effort(
          joint_external_effort_commands_[gripper_joint_index_], 0.0, false);
      }
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
  CommandModeSelection start_modes;
  CommandModeSelection stop_modes;
  if (!command_modes_from_list(start_interfaces, start_modes) ||
    !command_modes_from_list(stop_interfaces, stop_modes))
  {
    return return_type::ERROR;
  }

  if (start_modes.arm_modes.size() > 1) {
    RCLCPP_ERROR(
      get_logger(),
      "Mixed arm command modes requested in one switch. Arm modes and gripper modes may be "
      "combined, but the arm itself may only use one mode at a time.");
    return return_type::ERROR;
  }

  if (start_modes.gripper_modes.size() > 1) {
    RCLCPP_ERROR(
      get_logger(),
      "Mixed gripper command modes requested in one switch. The gripper may only use one mode "
      "at a time.");
    return return_type::ERROR;
  }

  if (start_modes.arm_modes.count(HW_IF_VELOCITY) > 0) {
    RCLCPP_ERROR(get_logger(), "Arm velocity mode requested but not implemented.");
    return return_type::ERROR;
  }

  if (start_modes.gripper_modes.count(HW_IF_VELOCITY) > 0) {
    RCLCPP_ERROR(get_logger(), "Gripper velocity mode requested but not implemented.");
    return return_type::ERROR;
  }

  if (!start_modes.arm_modes.empty()) {
    const auto & requested_arm_mode = *start_modes.arm_modes.begin();
    const auto active_mode = active_arm_mode();
    if (!active_mode.empty() && active_mode != requested_arm_mode &&
      stop_modes.arm_modes.count(active_mode) == 0)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Arm mode '%s' is active but was not requested to stop before switching the arm to '%s'.",
        active_mode.c_str(),
        requested_arm_mode.c_str());
      return return_type::ERROR;
    }
  }

  if (!start_modes.gripper_modes.empty()) {
    const auto & requested_gripper_mode = *start_modes.gripper_modes.begin();
    const auto active_mode = active_gripper_mode();
    if (!active_mode.empty() && active_mode != requested_gripper_mode &&
      stop_modes.gripper_modes.count(active_mode) == 0)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Gripper mode '%s' is active but was not requested to stop before switching the gripper "
        "to '%s'.",
        active_mode.c_str(),
        requested_gripper_mode.c_str());
      return return_type::ERROR;
    }
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Command mode switch preparation successful (arm start modes: %zu, gripper start modes: "
    "%zu, emergency stop interfaces: %s).",
    start_modes.arm_modes.size(),
    start_modes.gripper_modes.size(),
    start_modes.emergency_stop ? "yes" : "no");
  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  CommandModeSelection start_modes;
  CommandModeSelection stop_modes;
  if (!command_modes_from_list(start_interfaces, start_modes) ||
    !command_modes_from_list(stop_interfaces, stop_modes))
  {
    return return_type::ERROR;
  }

  // prepare_command_mode_switch() normally catches these conditions. Keep the
  // checks here as well so a direct or unusual controller-manager call cannot
  // leave the local mode flags in an ambiguous state.
  if (start_modes.arm_modes.size() > 1 || start_modes.gripper_modes.size() > 1 ||
    start_modes.arm_modes.count(HW_IF_VELOCITY) > 0 ||
    start_modes.gripper_modes.count(HW_IF_VELOCITY) > 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid command-mode combination reached perform_command_mode_switch().");
    return return_type::ERROR;
  }

  const std::string old_active_arm_mode = active_arm_mode();
  const std::string old_active_gripper_mode = active_gripper_mode();

  const bool old_arm_position_mode_running = arm_position_mode_running_;
  const bool old_arm_velocity_mode_running = arm_velocity_mode_running_;
  const bool old_arm_external_effort_mode_running = arm_external_effort_mode_running_;
  const bool old_cartesian_position_mode_running = cartesian_position_mode_running_;
  const bool old_cartesian_external_effort_mode_running =
    cartesian_external_effort_mode_running_;
  const bool old_gripper_position_mode_running = gripper_position_mode_running_;
  const bool old_gripper_velocity_mode_running = gripper_velocity_mode_running_;
  const bool old_gripper_external_effort_mode_running =
    gripper_external_effort_mode_running_;
  const bool old_emergency_stop_controller_running = emergency_stop_controller_running_;
  const bool old_arm_commands_suspended = arm_commands_suspended_after_emergency_stop_;
  const bool old_gripper_commands_suspended =
    gripper_commands_suspended_after_emergency_stop_;

  const bool arm_stop_requested = !stop_modes.arm_modes.empty();
  const bool gripper_stop_requested = !stop_modes.gripper_modes.empty();
  bool arm_driver_mode_may_have_changed = false;
  bool gripper_driver_mode_may_have_changed = false;

  if (stop_modes.arm_modes.count(HW_IF_POSITION) > 0) {
    arm_position_mode_running_ = false;
  }
  if (stop_modes.arm_modes.count(HW_IF_VELOCITY) > 0) {
    arm_velocity_mode_running_ = false;
  }
  if (stop_modes.arm_modes.count(HW_IF_EXTERNAL_EFFORT) > 0) {
    arm_external_effort_mode_running_ = false;
  }
  if (stop_modes.arm_modes.count(HW_IF_CARTESIAN_POSITION) > 0) {
    cartesian_position_mode_running_ = false;
  }
  if (stop_modes.arm_modes.count(HW_IF_CARTESIAN_EXTERNAL_EFFORT) > 0) {
    cartesian_external_effort_mode_running_ = false;
  }

  if (stop_modes.gripper_modes.count(HW_IF_POSITION) > 0) {
    gripper_position_mode_running_ = false;
  }
  if (stop_modes.gripper_modes.count(HW_IF_VELOCITY) > 0) {
    gripper_velocity_mode_running_ = false;
  }
  if (stop_modes.gripper_modes.count(HW_IF_EXTERNAL_EFFORT) > 0) {
    gripper_external_effort_mode_running_ = false;
  }

  if (stop_modes.emergency_stop) {
    emergency_stop_controller_running_ = false;
  }

  try {
    if (start_modes.emergency_stop) {
      emergency_stop_controller_running_ = true;
      RCLCPP_INFO(get_logger(), "Emergency stop controller command interfaces active.");
    }

    if (!start_modes.arm_modes.empty()) {
      const auto & requested_mode = *start_modes.arm_modes.begin();

      arm_position_mode_running_ = false;
      arm_velocity_mode_running_ = false;
      arm_external_effort_mode_running_ = false;
      cartesian_position_mode_running_ = false;
      cartesian_external_effort_mode_running_ = false;

      if (requested_mode == HW_IF_POSITION) {
        std::copy_n(
          joint_positions_.begin(), arm_joint_count_, joint_position_commands_.begin());
      } else if (requested_mode == HW_IF_EXTERNAL_EFFORT) {
        std::fill_n(joint_external_effort_commands_.begin(), arm_joint_count_, 0.0);
      } else if (requested_mode == HW_IF_CARTESIAN_POSITION) {
        cartesian_position_commands_ = cartesian_positions_;
        last_cartesian_position_command_id_ = cartesian_position_command_id_;
      } else if (requested_mode == HW_IF_CARTESIAN_EXTERNAL_EFFORT) {
        cartesian_external_effort_commands_.fill(0.0);
        last_cartesian_external_effort_command_id_ =
          cartesian_external_effort_command_id_;
      }

      if (!emergency_stop_engaged_) {
        arm_driver_mode_may_have_changed = true;
        apply_safe_arm_driver_mode(requested_mode);
        arm_commands_suspended_after_emergency_stop_ = false;
      }

      if (requested_mode == HW_IF_POSITION) {
        arm_position_mode_running_ = true;
      } else if (requested_mode == HW_IF_EXTERNAL_EFFORT) {
        arm_external_effort_mode_running_ = true;
      } else if (requested_mode == HW_IF_CARTESIAN_POSITION) {
        cartesian_position_mode_running_ = true;
      } else if (requested_mode == HW_IF_CARTESIAN_EXTERNAL_EFFORT) {
        cartesian_external_effort_mode_running_ = true;
      }

      if (requested_mode == HW_IF_POSITION) {
        RCLCPP_INFO(get_logger(), "Switched the arm group to joint position command mode.");
      } else if (requested_mode == HW_IF_EXTERNAL_EFFORT) {
        RCLCPP_INFO(get_logger(), "Switched the arm group to external effort command mode.");
      } else if (requested_mode == HW_IF_CARTESIAN_POSITION) {
        RCLCPP_INFO(get_logger(), "Switched the arm group to Cartesian position command mode.");
      } else if (requested_mode == HW_IF_CARTESIAN_EXTERNAL_EFFORT) {
        RCLCPP_INFO(
          get_logger(), "Switched the arm group to Cartesian external effort command mode.");
      }
    } else if (arm_stop_requested && !arm_mode_running() && !emergency_stop_engaged_) {
      arm_driver_mode_may_have_changed = true;
      apply_safe_arm_driver_mode(std::string{});
      RCLCPP_INFO(get_logger(), "All arm command modes stopped. Arm group set to idle.");
    }

    if (!start_modes.gripper_modes.empty()) {
      const auto & requested_mode = *start_modes.gripper_modes.begin();

      gripper_position_mode_running_ = false;
      gripper_velocity_mode_running_ = false;
      gripper_external_effort_mode_running_ = false;

      if (requested_mode == HW_IF_POSITION) {
        joint_position_commands_[gripper_joint_index_] =
          joint_positions_[gripper_joint_index_];
      } else if (requested_mode == HW_IF_EXTERNAL_EFFORT) {
        joint_external_effort_commands_[gripper_joint_index_] = 0.0;
      }

      if (!emergency_stop_engaged_) {
        gripper_driver_mode_may_have_changed = true;
        apply_safe_gripper_driver_mode(requested_mode);
        gripper_commands_suspended_after_emergency_stop_ = false;
      }

      if (requested_mode == HW_IF_POSITION) {
        gripper_position_mode_running_ = true;
      } else if (requested_mode == HW_IF_EXTERNAL_EFFORT) {
        gripper_external_effort_mode_running_ = true;
      }

      if (requested_mode == HW_IF_POSITION) {
        RCLCPP_INFO(get_logger(), "Switched the gripper group to position command mode.");
      } else if (requested_mode == HW_IF_EXTERNAL_EFFORT) {
        RCLCPP_INFO(
          get_logger(), "Switched the gripper group to external effort command mode.");
      }
    } else if (gripper_stop_requested && !gripper_mode_running() &&
      !emergency_stop_engaged_)
    {
      gripper_driver_mode_may_have_changed = true;
      apply_safe_gripper_driver_mode(std::string{});
      RCLCPP_INFO(get_logger(), "All gripper command modes stopped. Gripper group set to idle.");
    }
  } catch (const std::exception & e) {
    arm_position_mode_running_ = old_arm_position_mode_running;
    arm_velocity_mode_running_ = old_arm_velocity_mode_running;
    arm_external_effort_mode_running_ = old_arm_external_effort_mode_running;
    cartesian_position_mode_running_ = old_cartesian_position_mode_running;
    cartesian_external_effort_mode_running_ = old_cartesian_external_effort_mode_running;
    gripper_position_mode_running_ = old_gripper_position_mode_running;
    gripper_velocity_mode_running_ = old_gripper_velocity_mode_running;
    gripper_external_effort_mode_running_ = old_gripper_external_effort_mode_running;
    emergency_stop_controller_running_ = old_emergency_stop_controller_running;
    arm_commands_suspended_after_emergency_stop_ = old_arm_commands_suspended;
    gripper_commands_suspended_after_emergency_stop_ = old_gripper_commands_suspended;

    RCLCPP_ERROR(get_logger(), "Failed during command mode switch: %s", e.what());

    if (!emergency_stop_engaged_) {
      try {
        if (arm_driver_mode_may_have_changed) {
          apply_safe_arm_driver_mode(old_active_arm_mode);
        }
        if (gripper_driver_mode_may_have_changed) {
          apply_safe_gripper_driver_mode(old_active_gripper_mode);
        }
      } catch (const std::exception & rollback_error) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to restore the previous driver mode after a rejected switch: %s",
          rollback_error.what());
      }
    }

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
  gripper_position_mode_running_ = false;
  gripper_velocity_mode_running_ = false;
  gripper_external_effort_mode_running_ = false;
  arm_commands_suspended_after_emergency_stop_ = false;
  gripper_commands_suspended_after_emergency_stop_ = false;

  RCLCPP_INFO(get_logger(), "TrossenArmDriver disabled.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
TrossenArmHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  robot_output_ = trossen_arm::RobotOutput();
  emergency_stop_engaged_ = false;
  arm_commands_suspended_after_emergency_stop_ = false;
  gripper_commands_suspended_after_emergency_stop_ = false;
  arm_driver_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
TrossenArmHardwareInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(
    get_logger(),
    "Trossen Arm hardware entered an error state. Cleaning up the driver for recovery.");

  if (arm_driver_) {
    try {
      const auto error_information = arm_driver_->get_error_information();
      if (!error_information.empty()) {
        RCLCPP_ERROR(get_logger(), "Trossen Arm error information: %s", error_information.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to read Trossen Arm error information during recovery cleanup: %s",
        e.what());
    }

    try {
      arm_driver_->cleanup(false);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to clean up TrossenArmDriver after a hardware error: %s",
        e.what());
    }
  }

  arm_position_mode_running_ = false;
  arm_velocity_mode_running_ = false;
  arm_external_effort_mode_running_ = false;
  cartesian_position_mode_running_ = false;
  cartesian_external_effort_mode_running_ = false;
  emergency_stop_controller_running_ = false;
  gripper_position_mode_running_ = false;
  gripper_velocity_mode_running_ = false;
  gripper_external_effort_mode_running_ = false;
  emergency_stop_engaged_ = false;
  arm_commands_suspended_after_emergency_stop_ = false;
  gripper_commands_suspended_after_emergency_stop_ = false;
  first_update_ = true;
  robot_output_ = trossen_arm::RobotOutput();
  arm_driver_.reset();

  RCLCPP_WARN(
    get_logger(),
    "Trossen Arm hardware is ready to be configured again through controller_manager.");
  return CallbackReturn::SUCCESS;
}

return_type
TrossenArmHardwareInterface::engage_emergency_stop()
{
  std::fill(joint_external_effort_commands_.begin(), joint_external_effort_commands_.end(), 0.0);
  arm_driver_->set_all_modes(trossen_arm::Mode::external_effort);
  arm_driver_->set_all_external_efforts(joint_external_effort_commands_, 0.0, false);
  emergency_stop_engaged_ = true;
  arm_commands_suspended_after_emergency_stop_ = false;
  gripper_commands_suspended_after_emergency_stop_ = false;

  RCLCPP_WARN(
    get_logger(),
    "Emergency stop engaged. Hardware commands from other controllers are ignored.");
  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::release_emergency_stop()
{
  emergency_stop_engaged_ = false;
  arm_commands_suspended_after_emergency_stop_ = true;
  gripper_commands_suspended_after_emergency_stop_ = true;

  RCLCPP_WARN(
    get_logger(),
    "Emergency stop released. Arm and gripper commands remain independently suspended until "
    "their controllers are restarted.");
  return return_type::OK;
}

TrossenArmHardwareInterface::ParsedCommandInterface
TrossenArmHardwareInterface::parse_command_interface(const std::string & interface_name) const
{
  const auto slash_pos = interface_name.rfind('/');
  if (slash_pos == std::string::npos || slash_pos == 0 ||
    slash_pos + 1 >= interface_name.size())
  {
    return {};
  }

  const std::string resource = interface_name.substr(0, slash_pos);
  const std::string type = interface_name.substr(slash_pos + 1);
  const std::string mode = command_mode_from_interface_type(type);
  if (mode.empty()) {
    return {};
  }

  if (resource == EMERGENCY_STOP_COMPONENT_NAME && mode == HW_IF_EMERGENCY_STOP) {
    return {CommandGroup::emergency_stop, mode};
  }

  if (resource == CARTESIAN_COMPONENT_NAME &&
    (mode == HW_IF_CARTESIAN_POSITION || mode == HW_IF_CARTESIAN_EXTERNAL_EFFORT))
  {
    return {CommandGroup::arm, mode};
  }

  if (resource == gripper_joint_name_ &&
    (mode == HW_IF_POSITION || mode == HW_IF_VELOCITY || mode == HW_IF_EXTERNAL_EFFORT))
  {
    return {CommandGroup::gripper, mode};
  }

  const auto arm_joint_end = info_.joints.begin() + static_cast<std::ptrdiff_t>(arm_joint_count_);
  const auto arm_joint = std::find_if(
    info_.joints.begin(), arm_joint_end,
    [&resource](const auto & joint) {return joint.name == resource;});
  if (arm_joint != arm_joint_end &&
    (mode == HW_IF_POSITION || mode == HW_IF_VELOCITY || mode == HW_IF_EXTERNAL_EFFORT))
  {
    return {CommandGroup::arm, mode};
  }

  return {};
}

bool TrossenArmHardwareInterface::command_modes_from_list(
  const std::vector<std::string> & interfaces,
  CommandModeSelection & selection) const
{
  selection = CommandModeSelection{};

  for (const auto & interface_name : interfaces) {
    const auto parsed = parse_command_interface(interface_name);
    switch (parsed.group) {
      case CommandGroup::arm:
        selection.arm_modes.insert(parsed.mode);
        break;
      case CommandGroup::gripper:
        selection.gripper_modes.insert(parsed.mode);
        break;
      case CommandGroup::emergency_stop:
        selection.emergency_stop = true;
        break;
      case CommandGroup::unsupported:
      default:
        RCLCPP_ERROR(
          get_logger(),
          "Unsupported command interface '%s' requested during controller mode switch.",
          interface_name.c_str());
        return false;
    }
  }

  return true;
}

std::string TrossenArmHardwareInterface::active_arm_mode() const
{
  if (arm_position_mode_running_) {
    return HW_IF_POSITION;
  }
  if (arm_velocity_mode_running_) {
    return HW_IF_VELOCITY;
  }
  if (arm_external_effort_mode_running_) {
    return HW_IF_EXTERNAL_EFFORT;
  }
  if (cartesian_position_mode_running_) {
    return HW_IF_CARTESIAN_POSITION;
  }
  if (cartesian_external_effort_mode_running_) {
    return HW_IF_CARTESIAN_EXTERNAL_EFFORT;
  }
  return {};
}

std::string TrossenArmHardwareInterface::active_gripper_mode() const
{
  if (gripper_position_mode_running_) {
    return HW_IF_POSITION;
  }
  if (gripper_velocity_mode_running_) {
    return HW_IF_VELOCITY;
  }
  if (gripper_external_effort_mode_running_) {
    return HW_IF_EXTERNAL_EFFORT;
  }
  return {};
}

bool TrossenArmHardwareInterface::arm_mode_running() const
{
  return arm_position_mode_running_ || arm_velocity_mode_running_ ||
         arm_external_effort_mode_running_ || cartesian_position_mode_running_ ||
         cartesian_external_effort_mode_running_;
}

bool TrossenArmHardwareInterface::gripper_mode_running() const
{
  return gripper_position_mode_running_ || gripper_velocity_mode_running_ ||
         gripper_external_effort_mode_running_;
}

void TrossenArmHardwareInterface::stage_current_arm_position_hold()
{
  if (joint_positions_.size() < arm_joint_count_ ||
    arm_position_command_buffer_.size() != arm_joint_count_)
  {
    throw std::runtime_error("Arm position buffers are not initialized.");
  }

  for (size_t index = 0; index < arm_joint_count_; ++index) {
    const double position = joint_positions_[index];
    if (!std::isfinite(position)) {
      throw std::runtime_error(
              "Cannot enter an arm position mode because a measured arm position is non-finite.");
    }
    arm_position_command_buffer_[index] = position;
    joint_position_commands_[index] = position;
  }
}

void TrossenArmHardwareInterface::apply_safe_arm_driver_mode(
  const std::string & logical_mode)
{
  if (logical_mode.empty()) {
    arm_driver_->set_arm_modes(trossen_arm::Mode::idle);
    wait_for_driver_mode(
      *arm_driver_, 0, arm_joint_count_, trossen_arm::Mode::idle, "arm group", "idle");
    return;
  }

  if (logical_mode == HW_IF_POSITION || logical_mode == HW_IF_CARTESIAN_POSITION) {
    stage_current_arm_position_hold();
    arm_driver_->set_arm_modes(trossen_arm::Mode::position);
    wait_for_driver_mode(
      *arm_driver_, 0, arm_joint_count_, trossen_arm::Mode::position,
      "arm group", "position");
    arm_driver_->set_arm_positions(arm_position_command_buffer_, 0.0, false);
    return;
  }

  if (logical_mode == HW_IF_EXTERNAL_EFFORT ||
    logical_mode == HW_IF_CARTESIAN_EXTERNAL_EFFORT)
  {
    std::fill(
      arm_external_effort_command_buffer_.begin(),
      arm_external_effort_command_buffer_.end(),
      0.0);
    std::fill_n(joint_external_effort_commands_.begin(), arm_joint_count_, 0.0);
    arm_driver_->set_arm_modes(trossen_arm::Mode::external_effort);
    wait_for_driver_mode(
      *arm_driver_, 0, arm_joint_count_, trossen_arm::Mode::external_effort,
      "arm group", "external effort");
    arm_driver_->set_arm_external_efforts(
      arm_external_effort_command_buffer_, 0.0, false);
    return;
  }

  throw std::invalid_argument("Unsupported logical arm command mode: " + logical_mode);
}

void TrossenArmHardwareInterface::apply_safe_gripper_driver_mode(
  const std::string & logical_mode)
{
  if (logical_mode.empty()) {
    arm_driver_->set_gripper_mode(trossen_arm::Mode::idle);
    wait_for_driver_mode(
      *arm_driver_, gripper_joint_index_, 1, trossen_arm::Mode::idle,
      "gripper group", "idle");
    return;
  }

  if (logical_mode == HW_IF_POSITION) {
    const double position = joint_positions_.at(gripper_joint_index_);
    if (!std::isfinite(position)) {
      throw std::runtime_error(
              "Cannot enter gripper position mode because its measured position is non-finite.");
    }
    joint_position_commands_[gripper_joint_index_] = position;
    arm_driver_->set_gripper_mode(trossen_arm::Mode::position);
    wait_for_driver_mode(
      *arm_driver_, gripper_joint_index_, 1, trossen_arm::Mode::position,
      "gripper group", "position");
    arm_driver_->set_gripper_position(position, 0.0, false);
    return;
  }

  if (logical_mode == HW_IF_EXTERNAL_EFFORT) {
    joint_external_effort_commands_[gripper_joint_index_] = 0.0;
    arm_driver_->set_gripper_mode(trossen_arm::Mode::external_effort);
    wait_for_driver_mode(
      *arm_driver_, gripper_joint_index_, 1, trossen_arm::Mode::external_effort,
      "gripper group", "external effort");
    arm_driver_->set_gripper_external_effort(0.0, 0.0, false);
    return;
  }

  throw std::invalid_argument("Unsupported logical gripper command mode: " + logical_mode);
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
