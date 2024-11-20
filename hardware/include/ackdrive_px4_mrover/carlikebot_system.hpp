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

#ifndef ACKDRIVE_PX4_MROVER__CARLIKEBOT_SYSTEM_HPP_
#define ACKDRIVE_PX4_MROVER__CARLIKEBOT_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ackdrive_px4_mrover/mrover_comms.hpp"
// #include "ackdrive_px4_mrover/joint.hpp"

namespace ackdrive_px4_mrover
{
struct JointValue
{
  double encoder{0.0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};

  double calc_enc_value() const 
  {
  return encoder * 0.95; // Example logic
  }
};

struct Joint
{
  explicit Joint(const std::string & name) : joint_name(name)
  {
    state = JointValue();
    command = JointValue();
  }

  Joint() = default;

  std::string joint_name;
  JointValue state;
  JointValue command;

  
};
class AckDriveMroverHardware : public hardware_interface::SystemInterface
{
  struct Config
{
  // std::string left_wheel_name = ""; //steering
  // std::string right_wheel_name = ""; //traction
  std::string steering_joint_name = ""; //steering
  std::string traction_joint_name = ""; //traction
  // float loop_rate = 0.0;
  // std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  // int enc_counts_per_rev = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AckDriveMroverHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
  // std::make_shared<MroverComms>
  // Parameters for the MROVER hardware

  // MroverComms comms_;
  std::shared_ptr<MroverComms> comms_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  Config cfg_;
  Joint steering_joint_;
  Joint traction_joint_;

  

};

}  // namespace ackdrive_px4_mrover

#endif  // ACKDRIVE_PX4_MROVER__CARLIKEBOT_SYSTEM_HPP_
