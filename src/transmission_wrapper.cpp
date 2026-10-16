/*
 Copyright (c) 2023 RobeeContech, Inc.
 Author: Yoav Fekete
*/
#include "hardware_interface_wrappers/transmission_wrapper.hpp"
#include <map>
#include <limits>

const rclcpp::Logger logger = rclcpp::get_logger("TransmissionWrapper");

namespace hardware_interface_wrappers
{

  hardware_interface::CallbackReturn TransmissionWrapper::on_init(
    const hardware_interface::HardwareInfo & info)
  {
    
    RCLCPP_INFO_STREAM(logger,"Start init of Transmission Wrapper");
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        RCLCPP_FATAL(logger,"Init Of System Interface Failed!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    include_interface_name.emplace_back("position");
    // include_interface_name.emplace_back("velocity");
    // include_interface_name.emplace_back("effort");
    hw_joint_states_.resize(info_.joints.size());
    for(uint j = 0; j < info_.joints.size(); j++)
        hw_joint_states_[j].resize(info_.joints[j].state_interfaces.size(),std::numeric_limits<double>::quiet_NaN());

    hw_joint_commands_.resize(info_.joints.size());
    for(uint j = 0; j < info_.joints.size(); j++)
        hw_joint_commands_[j].resize(info_.joints[j].command_interfaces.size(),std::numeric_limits<double>::quiet_NaN());

    std::string wrapped_interface_name ="";
    for (auto x : info.hardware_parameters)
    {
       if (x.first == "wrapped_interface") wrapped_interface_name = x.second;

    }
    if (wrapped_interface_name == "")
    {
        RCLCPP_FATAL(logger,"Init Of transmission_wrapper Failed.  wrapped_interface paramater not specfied");
        return hardware_interface::CallbackReturn::ERROR;
    }

    loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
                "hardware_interface","hardware_interface::SystemInterface");
    try
    {    
      wrapped_interface_ = std::unique_ptr<hardware_interface::SystemInterface>(
          loader_->createUnmanagedInstance(wrapped_interface_name));
    }
    catch(const std::exception& e)
    {
      RCLCPP_FATAL_STREAM(logger,"Init Of transmission_wrapper Failed.  exception thrown while running load_hardware. "<<e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
   
    if (!wrapped_interface_ )
    {
       RCLCPP_FATAL(logger,"Init Of transmission_wrapper Failed.  wrapped_interface = null!!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    auto res = wrapped_interface_->on_init(info);

    RCLCPP_INFO_STREAM(logger,"Loading transmission_manager");
    transmission_manager_ = std::make_unique<transmission_manager::TransmissionManager>(); 
    if (!transmission_manager_->init(info))
    {
       RCLCPP_FATAL(logger,"Init Of transmission_wrapper Failed: init of transmission_manager_ returned false");
        return hardware_interface::CallbackReturn::ERROR;
    }
     
    RCLCPP_INFO_STREAM(logger,"Finish init TransmissionWrapper ");
    return res;
  }


  std::vector<hardware_interface::StateInterface> TransmissionWrapper::export_state_interfaces()
  {
    auto state_interfaces = wrapped_interface_->export_state_interfaces();
    std::vector<hardware_interface::StateInterface> res;
    //replace state interface of joints as are in the wrapped interface with the joint space converted vars

  // map from joint index, to all its  JointHandle (one for each used state_interfaces)
   std::map<std::string, std::vector<JointHandle>>     joint_handles_states; 
   std::map<std::string, std::vector<ActuatorHandle>>  actuator_handles_states;

    for(long unsigned int k = 0; k<state_interfaces.size(); k++) 
    {
      bool is_joint_state = false;
      for(uint j = 0; j < info_.joints.size(); j++ && !is_joint_state) 
      {
        if (state_interfaces[k].get_prefix_name() == info_.joints[j].name)
        {
          is_joint_state = true;        
          bool found_state_interface = false;
          for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++ && !found_state_interface)
          { 
            // Velocity feedback uses the same transmission as position, without its offset.
            // The wrapped driver must provide actuator position units per second.
            if (state_interfaces[k].get_interface_name() == info_.joints[j].state_interfaces[i].name &&
                (state_interfaces[k].get_interface_name() == "velocity" ||
                 std::find(include_interface_name.begin(),include_interface_name.end(),state_interfaces[k].get_interface_name()) != include_interface_name.end()))
            {
              found_state_interface = true;
              HackableHandle t(state_interfaces[k]);
              res.push_back(hardware_interface::StateInterface(
                  t.get_prefix_name(), t.get_interface_name(),&hw_joint_states_[j][i]));
              if (auto search = joint_handles_states.find(info_.joints[j].name); search != joint_handles_states.end())
              {
                search->second.emplace_back(t.get_prefix_name(),t.get_interface_name(),&hw_joint_states_[j][i]);
                actuator_handles_states[info_.joints[j].name].emplace_back(t.get_prefix_name(),t.get_interface_name(),t.get_ptr());
              }  
              else
              {
                std::vector<JointHandle>    tjh;
                std::vector<ActuatorHandle> tah;
                tjh.emplace_back(t.get_prefix_name(),t.get_interface_name(),&hw_joint_states_[j][i]);
                tah.emplace_back(t.get_prefix_name(),t.get_interface_name(),t.get_ptr());
                joint_handles_states[info_.joints[j].name]=tjh;
                actuator_handles_states[info_.joints[j].name]=tah;
              }
            }
          }
          if (!found_state_interface) {
             res.push_back(state_interfaces[k]);
          }
        }
      }
      if (!is_joint_state) res.push_back(state_interfaces[k]);
    }
    transmission_manager_->config_states_transmissions(info_ , joint_handles_states, actuator_handles_states);
    return res;
  }

  std::vector<hardware_interface::CommandInterface> TransmissionWrapper::export_command_interfaces()
  {
    auto command_interfaces =  wrapped_interface_->export_command_interfaces();
    
    std::vector<hardware_interface::CommandInterface> res;
    //replace state interface of joints as are in the wrapped interface with the joint space converted vars
    // map from joint index, to all its  JointHandle (one for each used state_interfaces)
    std::map<std::string,std::vector<JointHandle>>     joint_handles_cmds; 
    std::map<std::string, std::vector<ActuatorHandle>> actuator_handles_cmds;

    for(long unsigned int k = 0; k<command_interfaces.size(); k++) 
    {
      bool is_joint_state = false;
      for(uint j = 0; j < info_.joints.size(); j++ && !is_joint_state) 
      {
        if (command_interfaces[k].get_prefix_name() == info_.joints[j].name && std::find(include_interface_name.begin(),include_interface_name.end(),command_interfaces[k].get_interface_name()) != include_interface_name.end())
        {
          is_joint_state = true;
          bool found_state_interface = false;
          for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++ && !found_state_interface)
          {
            if (command_interfaces[k].get_interface_name() == info_.joints[j].command_interfaces[i].name)
            {
              found_state_interface = true;
              HackableHandle t(command_interfaces[k]);
              res.push_back(hardware_interface::CommandInterface(
                  t.get_prefix_name(), t.get_interface_name(),&hw_joint_commands_[j][i]));
              if (auto search = joint_handles_cmds.find(info_.joints[j].name); search != joint_handles_cmds.end())
              {
                search->second.emplace_back(t.get_prefix_name(),t.get_interface_name(),&hw_joint_commands_[j][i]);
                actuator_handles_cmds[info_.joints[j].name].emplace_back(t.get_prefix_name(),t.get_interface_name(),t.get_ptr());
              }  
              else
              {
                std::vector<JointHandle>    tjh;
                std::vector<ActuatorHandle> tah;
                tjh.emplace_back(t.get_prefix_name(),t.get_interface_name(),&hw_joint_commands_[j][i]);
                tah.emplace_back(t.get_prefix_name(),t.get_interface_name(),t.get_ptr());
                joint_handles_cmds[info_.joints[j].name]=tjh;
                actuator_handles_cmds[info_.joints[j].name]=tah;
              }
            }
          }
          if (!found_state_interface) {
              HackableHandle t(command_interfaces[k]);
              res.push_back(hardware_interface::CommandInterface(t.get_prefix_name(), t.get_interface_name(),t.get_ptr()));
          }
        }
      }
      if (!is_joint_state) {
        HackableHandle t(command_interfaces[k]);
        res.push_back(hardware_interface::CommandInterface(t.get_prefix_name(), t.get_interface_name(),t.get_ptr()));
      }
    }
    transmission_manager_->config_commands_transmissions(info_,joint_handles_cmds, actuator_handles_cmds);
    return res;
  }

  hardware_interface::CallbackReturn TransmissionWrapper::on_activate(
    const rclcpp_lifecycle::State & previous_state)
  {
    return wrapped_interface_->on_activate(previous_state);
  }

  hardware_interface::CallbackReturn TransmissionWrapper::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
  {
    return wrapped_interface_->on_deactivate(previous_state);
  }

  hardware_interface::return_type TransmissionWrapper::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    read_result_ =  wrapped_interface_->read(time,period);
    transmission_manager_->state_actuator_to_joint();
    return read_result_;
  }

  hardware_interface::return_type TransmissionWrapper::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {      
      transmission_manager_->cmd_joint_to_actuator();
      return wrapped_interface_->write(time,period);
  }

  hardware_interface::return_type TransmissionWrapper::prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) 
  {
    return wrapped_interface_->prepare_command_mode_switch(start_interfaces,stop_interfaces);
  }
  
    
  hardware_interface::return_type TransmissionWrapper::perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) 
  {
    return wrapped_interface_->perform_command_mode_switch(start_interfaces,stop_interfaces);
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hardware_interface_wrappers::TransmissionWrapper, hardware_interface::SystemInterface)