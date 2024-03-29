// Copyright 2022 RobeeRobotics LTD
// Author: Yoav Fekete
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

#ifndef HARDWARE_INTERFACE_WRAPPERS_TRANSMISSION_WRAPPER_HPP_
#define HARDWARE_INTERFACE_WRAPPERS_TRANSMISSION_WRAPPER_HPP_

#include <memory>

#include "hardware_interface_wrappers/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

#include "transmission_manager/transmission_manager.hpp"

namespace hardware_interface_wrappers
{
  class HackableHandle : public hardware_interface::ReadOnlyHandle
  {
    public:

      HackableHandle(const hardware_interface::ReadOnlyHandle & other) : 
          hardware_interface::ReadOnlyHandle(other)
      {

      }

      double* get_ptr()  
      {
        return value_ptr_;
      }
  };

  /**
   * \brief Class for loading a Basic transmission instance from configuration data.
   */
  class TransmissionWrapper : public hardware_interface::SystemInterface
  {

    public:

      RCLCPP_SHARED_PTR_DEFINITIONS(TransmissionWrapper)

      ~TransmissionWrapper(){
          if (wrapped_interface_)  wrapped_interface_.reset();
          loader_.reset();
      }

      HARDWARE_INTERFACE_WRAPPERS_PUBLIC
      hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

      HARDWARE_INTERFACE_WRAPPERS_PUBLIC
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      HARDWARE_INTERFACE_WRAPPERS_PUBLIC
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      HARDWARE_INTERFACE_WRAPPERS_PUBLIC
      hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

      HARDWARE_INTERFACE_WRAPPERS_PUBLIC
      hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

      HARDWARE_INTERFACE_WRAPPERS_PUBLIC
      hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

      HARDWARE_INTERFACE_WRAPPERS_PUBLIC
      hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

      HARDWARE_INTERFACE_WRAPPERS_PUBLIC
      hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) override;
  
      HARDWARE_INTERFACE_WRAPPERS_PUBLIC
      hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) override;

    private:

      std::unique_ptr<hardware_interface::SystemInterface>                         wrapped_interface_;
      std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> loader_;
      std::unique_ptr<transmission_manager::TransmissionManager>                   transmission_manager_;  

      std::vector<std::string> include_interface_name;
      std::vector<std::vector<double>> hw_joint_commands_;
      std::vector<std::vector<double>> hw_joint_states_;

      //result of the wrapped read operation. defined so no new memory will be used in real_time code.
      hardware_interface::return_type read_result_;
    };


}  // namespace hardware_interface_wrappers

#endif  // HARDWARE_INTERFACE_WRAPPERS_TRANSMISSION_WRAPPER_HPP_
