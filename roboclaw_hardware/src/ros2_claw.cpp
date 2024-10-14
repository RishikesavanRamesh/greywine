// Copyright (c) 2024, Rishikesavan Ramesh
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "roboclaw_hardware/ros2_claw.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace roboclaw_hardware
{
hardware_interface::CallbackReturn Ros2Claw::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.wheel_left_name = info_.hardware_parameters["wheel_left_name"];
  cfg_.wheel_right_name = info_.hardware_parameters["wheel_right_name"];

  cfg_.device = info_.hardware_parameters["device"];

  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);


  wheel_left_.setup(cfg_.wheel_left_name, cfg_.enc_counts_per_rev);
  wheel_right_.setup(cfg_.wheel_right_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ros2Claw"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ros2Claw"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ros2Claw"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ros2Claw"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ros2Claw"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
  
}

std::vector<hardware_interface::StateInterface> Ros2Claw::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_left_.name, hardware_interface::HW_IF_POSITION, &wheel_left_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_left_.name, hardware_interface::HW_IF_VELOCITY, &wheel_left_.vel));
  
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_right_.name, hardware_interface::HW_IF_POSITION, &wheel_right_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_right_.name, hardware_interface::HW_IF_VELOCITY, &wheel_right_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Ros2Claw::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_left_.name, hardware_interface::HW_IF_VELOCITY, &wheel_left_.cmd));
    
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_right_.name, hardware_interface::HW_IF_VELOCITY, &wheel_right_.cmd));

  return command_interfaces;
}


hardware_interface::CallbackReturn Ros2Claw::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(rclcpp::get_logger("Ros2Claw"), "Configuring ...please wait...");

  comms_.connect(cfg_.device);


  RCLCPP_INFO(rclcpp::get_logger("Ros2Claw"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2Claw::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  comms_.stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2Claw::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  comms_.stop();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ros2Claw::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  
  comms_.read_encoder_values(wheel_left_.enc, wheel_right_.enc);


  double delta_seconds = period.seconds();

  double pos_prev = wheel_left_.pos;
  wheel_left_.pos = wheel_left_.calc_enc_angle();
  wheel_left_.vel = (wheel_left_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_right_.pos;
  wheel_right_.pos = wheel_right_.calc_enc_angle();
  wheel_right_.vel = (wheel_right_.pos - pos_prev) / delta_seconds;
  

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ros2Claw::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  comms_.sendCommand(static_cast<uint8_t>(wheel_left_.cmd), static_cast<uint8_t>(wheel_right_.cmd));


  return hardware_interface::return_type::OK;
}

}  // namespace roboclaw_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  roboclaw_hardware::Ros2Claw, hardware_interface::SystemInterface)
