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

#ifndef TROSSEN_ARM_HARDWARE__INTERFACE_HPP_
#define TROSSEN_ARM_HARDWARE__INTERFACE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "libtrossen_arm/trossen_arm.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

namespace trossen_arm_hardware
{

constexpr char DRIVER_IP_ADDRESS_DEFAULT[] = "192.168.1.2";

constexpr char END_EFFECTOR_BASE[] = "base";
constexpr char END_EFFECTOR_FOLLOWER[] = "follower";
constexpr char END_EFFECTOR_LEADER[] = "leader";

constexpr char HW_IF_EXTERNAL_EFFORT[] = "external_effort";

constexpr char CARTESIAN_COMPONENT_NAME[] = "cartesian";

constexpr char HW_IF_CARTESIAN_POSITION[] = "cartesian_position";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT[] = "cartesian_external_effort";

constexpr char HW_IF_CARTESIAN_POSITION_PREFIX[] = "position.";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_PREFIX[] = "external_effort.";

constexpr char HW_IF_CARTESIAN_POSITION_X[] = "position.x";
constexpr char HW_IF_CARTESIAN_POSITION_Y[] = "position.y";
constexpr char HW_IF_CARTESIAN_POSITION_Z[] = "position.z";
constexpr char HW_IF_CARTESIAN_POSITION_RX[] = "position.rx";
constexpr char HW_IF_CARTESIAN_POSITION_RY[] = "position.ry";
constexpr char HW_IF_CARTESIAN_POSITION_RZ[] = "position.rz";
constexpr char HW_IF_CARTESIAN_POSITION_GOAL_TIME[] = "position.goal_time";
constexpr char HW_IF_CARTESIAN_POSITION_INTERPOLATION_SPACE[] = "position.interpolation_space";
constexpr char HW_IF_CARTESIAN_POSITION_COMMAND_ID[] = "position.command_id";

constexpr char HW_IF_CARTESIAN_VELOCITY_X[] = "velocity.x";
constexpr char HW_IF_CARTESIAN_VELOCITY_Y[] = "velocity.y";
constexpr char HW_IF_CARTESIAN_VELOCITY_Z[] = "velocity.z";
constexpr char HW_IF_CARTESIAN_VELOCITY_RX[] = "velocity.rx";
constexpr char HW_IF_CARTESIAN_VELOCITY_RY[] = "velocity.ry";
constexpr char HW_IF_CARTESIAN_VELOCITY_RZ[] = "velocity.rz";

constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_FX[] = "external_effort.fx";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_FY[] = "external_effort.fy";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_FZ[] = "external_effort.fz";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_TX[] = "external_effort.tx";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_TY[] = "external_effort.ty";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_TZ[] = "external_effort.tz";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_GOAL_TIME[] = "external_effort.goal_time";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_INTERPOLATION_SPACE[] =
  "external_effort.interpolation_space";
constexpr char HW_IF_CARTESIAN_EXTERNAL_EFFORT_COMMAND_ID[] = "external_effort.command_id";

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using hardware_interface::CommandInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using trossen_arm::TrossenArmDriver;

class TrossenArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrossenArmHardwareInterface)

  CallbackReturn on_init(const HardwareInfo & info) override;

  std::vector<StateInterface> export_state_interfaces() override;

  std::vector<CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & duration) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & duration) override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

protected:
  // The driver for the robot
  std::unique_ptr<TrossenArmDriver> arm_driver_{nullptr};

  // Model of the robot this hardware interface connects to
  trossen_arm::Model robot_model_;

  // End effector type of the robot this hardware interface connects to
  trossen_arm::EndEffector end_effector_;

  // String representation of the end effector type
  std::string end_effector_str_;

  // IP address of the driver this hardware interface connects to
  std::string driver_ip_address_{DRIVER_IP_ADDRESS_DEFAULT};

  // Robot output structure to hold the robot's state
  trossen_arm::RobotOutput robot_output_;

  // Joint positions in radians for the arm and meters for the gripper
  std::vector<double> joint_positions_;

  // Joint velocities in radians per second for the arm and meters per second for the gripper
  std::vector<double> joint_velocities_;

  // Joint efforts in Newton-meters for the arm and Newtons for the gripper
  std::vector<double> joint_efforts_;

  // Joint position commands in radians for the arm and meters for the gripper
  std::vector<double> joint_position_commands_;

  // Joint velocity commands in radians per second for the arm and meters per second for the gripper
  std::vector<double> joint_velocity_commands_;

  // Joint external effort commands in Nm for the arm and N for the gripper
  std::vector<double> joint_external_effort_commands_;

  // Cartesian states: x/y/z in m, rx/ry/rz in rad, forces in N, torques in Nm.
  std::array<double, 6> cartesian_positions_{};
  std::array<double, 6> cartesian_velocities_{};
  std::array<double, 6> cartesian_external_efforts_{};

  // Cartesian position command: x, y, z, rx, ry, rz.
  std::array<double, 6> cartesian_position_commands_{};
  double cartesian_position_goal_time_command_{0.0};
  double cartesian_position_interpolation_space_command_{1.0};
  double cartesian_position_command_id_{0.0};
  double last_cartesian_position_command_id_{0.0};

  // Cartesian external effort command: fx, fy, fz, tx, ty, tz.
  std::array<double, 6> cartesian_external_effort_commands_{};
  double cartesian_external_effort_goal_time_command_{0.0};
  double cartesian_external_effort_interpolation_space_command_{1.0};
  double cartesian_external_effort_command_id_{0.0};
  double last_cartesian_external_effort_command_id_{0.0};

  // Flag to indicate the first read/write update
  bool first_update_{true};

  const size_t COUNT_COMMAND_INTERFACES_ = 3;  // position, velocity, external effort
  const size_t INDEX_COMMAND_INTERFACE_POSITION_ = 0;
  const size_t INDEX_COMMAND_INTERFACE_VELOCITY_ = 1;
  const size_t INDEX_COMMAND_INTERFACE_EXTERNAL_EFFORT_ = 2;
  const size_t COUNT_STATE_INTERFACES_ = 3;  // position, velocity, effort
  const size_t INDEX_STATE_INTERFACE_POSITION_ = 0;
  const size_t INDEX_STATE_INTERFACE_VELOCITY_ = 1;
  const size_t INDEX_STATE_INTERFACE_EFFORT_ = 2;

  // Control mode flags
  bool arm_position_mode_running_{false};
  bool arm_velocity_mode_running_{false};
  bool arm_external_effort_mode_running_{false};
  bool cartesian_position_mode_running_{false};
  bool cartesian_external_effort_mode_running_{false};

  bool gripper_position_mode_running_{false};
  bool gripper_velocity_mode_running_{false};
  bool gripper_effort_mode_running_{false};

  // Logger
  rclcpp::Logger get_logger() const override;

  /**
   * @brief Check if the interface type is in the stop interfaces
   *
   * @param stop_interfaces The interfaces to stop
   * @param type The interface type to check
   * @return true if the interface type is in the stop interfaces, false otherwise
   */
  bool interface_type_in_stop(
    const std::vector<std::string> & stop_interfaces,
    const std::string & type);

  /**
   * @brief Extract the interface types from a list of interfaces
   *
   * @param ifaces The list of interfaces
   * @return A set of interface types
   */
  std::set<std::string> interface_types_from_list(const std::vector<std::string> & ifaces);

  /**
   * @brief Map an individual ros2_control command interface suffix to a logical command mode.
   */
  std::string command_mode_from_interface_type(const std::string & type) const;

  /**
   * @brief Check whether a logical command mode is included in a stop-interface list.
   */
  bool interface_mode_in_stop(
    const std::vector<std::string> & stop_interfaces,
    const std::string & mode);

  bool has_prefix(const std::string & value, const std::string & prefix) const;

  bool is_new_command(double command_id, double last_command_id) const;

  bool all_finite(const std::array<double, 6> & values) const;

  trossen_arm::InterpolationSpace interpolation_space_from_command(double value) const;
};

}  // namespace trossen_arm_hardware

#endif  // TROSSEN_ARM_HARDWARE__INTERFACE_HPP_
