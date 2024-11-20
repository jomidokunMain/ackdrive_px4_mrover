// Copyright 2021 ros2_control Development Team
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

#include "ackdrive_px4_mrover/carlikebot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ackdrive_px4_mrover
{
hardware_interface::CallbackReturn AckDriveMroverHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("AckDriveMroverHardware"),
      "AckDriveMroverHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("AckDriveMroverHardware"), "Joint '%s' is a steering joint.",
        joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("AckDriveMroverHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("AckDriveMroverHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("AckDriveMroverHardware"),
          "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("AckDriveMroverHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("AckDriveMroverHardware"), "Joint '%s' is a drive joint.",
        joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("AckDriveMroverHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("AckDriveMroverHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("AckDriveMroverHardware"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("AckDriveMroverHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("AckDriveMroverHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"]; //steering
  // cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"]; //traction
  cfg_.steering_joint_name= info_.hardware_parameters["virtual_front_wheel_joint"]; //steering
  cfg_.traction_joint_name  = info_.hardware_parameters["virtual_rear_wheel_joint"]; //traction
  // cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  // cfg_.device = info_.hardware_parameters["device"];
  // cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  // cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  // comms_ = std::make_shared<MroverComms>();
  // Create and initialize the MroverComms node
  comms_ = std::make_shared<MroverComms>();
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(comms_);
  // Start the executor in a separate thread
  executor_thread_ = std::thread([this]() {
    RCLCPP_INFO(rclcpp::get_logger("AckDriveMroverHardware"), "Starting executor for MroverComms.");
    executor_->spin();
  });
  RCLCPP_INFO(rclcpp::get_logger("AckDriveMroverHardware"), "MROVER hardware initialized successfully.");
  // steering_joint_.setup(cfg_.steering_joint_name, cfg_.enc_counts_per_rev);
  // traction_joint_.setup(cfg_.traction_joint_name, cfg_.enc_counts_per_rev);
  steering_joint_.joint_name = cfg_.steering_joint_name;

  traction_joint_.joint_name = cfg_.traction_joint_name;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AckDriveMroverHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;


  // Add state interfaces for "steering" joint
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    steering_joint_.joint_name, hardware_interface::HW_IF_POSITION, &steering_joint_.state.position)); 
  // Add state interfaces for "traction" joint
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    traction_joint_.joint_name, hardware_interface::HW_IF_POSITION, &traction_joint_.state.position));
  // Add velocity interface only for "traction" joint
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    traction_joint_.joint_name, hardware_interface::HW_IF_VELOCITY, &traction_joint_.state.velocity));
  
  

  RCLCPP_INFO(
    rclcpp::get_logger("AckDriveMroverHardware"), "Exported %zu state interfaces.",
    state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("AckDriveMroverHardware"), "Exported state interface '%s'.",
      s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AckDriveMroverHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Add command interface for "steering" joint
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      steering_joint_.joint_name, hardware_interface::HW_IF_POSITION,
      &steering_joint_.command.position));
    // Add command interface for "traction" joint
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      traction_joint_.joint_name, hardware_interface::HW_IF_VELOCITY,
      &traction_joint_.command.velocity));

  RCLCPP_INFO(
    rclcpp::get_logger("AckDriveMroverHardware"), "Exported %zu command interfaces.",
    command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("AckDriveMroverHardware"), "Exported command interface '%s'.",
      command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AckDriveMroverHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckDriveMroverHardware"), "Activating ...please wait...");
  // comms_.connect(cfg_.timeout_ms);
  // rclcpp::spin(comms_);
  comms_->connect(cfg_.timeout_ms);
  // rclcpp::init(argc, argv);
  // auto mrover_comms_node = std::make_shared<MroverComms>();
  // mrover_comms_node->connect(1000);  // Simulated timeout

  // rclcpp::spin(comms_);

  RCLCPP_INFO(rclcpp::get_logger("AckDriveMroverHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckDriveMroverHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckDriveMroverHardware"), "Deactivating ...please wait...");

  // Stop spinning the executor and join the thread
  if (executor_ && executor_thread_.joinable()) {
      executor_->cancel();
      executor_thread_.join();
      RCLCPP_INFO(rclcpp::get_logger("AckDriveMroverHardware"), "Executor stopped.");
  }

  // Disconnect communication
  comms_->disconnect();

  RCLCPP_INFO(rclcpp::get_logger("AckDriveMroverHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AckDriveMroverHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  

  comms_->read_encoder_values(steering_joint_.state.encoder, traction_joint_.state.encoder);
  double delta_seconds = period.seconds();

  steering_joint_.state.position =  steering_joint_.state.encoder;

  double pos_prev = traction_joint_.state.position;
  traction_joint_.state.position = traction_joint_.state.encoder;
  traction_joint_.state.velocity = (traction_joint_.state.position - pos_prev) / delta_seconds;

  // RCLCPP_INFO(rclcpp::get_logger("AckDriveMroverHardware"), "state position: %f, %f", steering_joint_.state.position, traction_joint_.state.position);
  
  // node->connect(1000);  // Example: Pass a timeout

  // rclcpp::spin(node);

  // node->disconnect();

  // rclcpp::shutdown();


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ackdrive_px4_mrover ::AckDriveMroverHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  // double delta_sec = period.seconds();

  double motor_tract_vel = traction_joint_.command.velocity ;
  
  // double current_pos = steering_joint_.state.position;
  double desired_pos = steering_joint_.command.position ;
  double motor_steer_vel = desired_pos;

  comms_->set_motor_values(motor_steer_vel, motor_tract_vel);

  // // Control parameters
  // double kP = 0.557;               // Proportional gain for steering control
  // double max_steer_velocity = 1.0;  // Maximum steering motor velocity

  // // Calculate position error
  // double position_error = desired_pos - current_pos;

  // // Proportional control for steering velocity
  // double motor_steer_vel = kP * position_error;

  // // Clamp motor_steer_vel to within max limits
  // motor_steer_vel = std::min(std::max(motor_steer_vel, -max_steer_velocity), max_steer_velocity);

  // // Update motor values with calculated steering velocity and traction velocity
  // comms_->set_motor_values(motor_steer_vel, motor_tract_vel);

  // RCLCPP_INFO(rclcpp::get_logger("AckDriveMroverHardware"), "command position: %f, %f", steering_joint_.command.position, traction_joint_.command.velocity);


  // comms_.set_motor_values(motor_steer_counts_per_loop, motor_tract_vel );
  
  

  return hardware_interface::return_type::OK;
  
}

}  // namespace ackdrive_px4_mrover

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ackdrive_px4_mrover::AckDriveMroverHardware, hardware_interface::SystemInterface)
