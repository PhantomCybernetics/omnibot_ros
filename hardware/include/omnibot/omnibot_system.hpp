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

#ifndef OMNIBOT__SYSTEM_HPP_
#define OMNIBOT__SYSTEM_HPP_

#include <memory>
#include <string>
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
#include "rclcpp/rclcpp.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "std_msgs/msg/int8_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "omnibot/visibility_control.h"

namespace omnibot
{

  using return_type = hardware_interface::return_type;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using StateInterface = hardware_interface::StateInterface;
  using CommandInterface = hardware_interface::CommandInterface;

  using JointState = sensor_msgs::msg::JointState;
  using Int8MultiArray = std_msgs::msg::Int8MultiArray;

class OmnibotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OmnibotSystemHardware);

  OMNIBOT_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  OMNIBOT_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  OMNIBOT_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  OMNIBOT_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNIBOT_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNIBOT_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  OMNIBOT_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  OMNIBOT_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  OMNIBOT_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  void cleanup_node();

  realtime_tools::RealtimeBox<std::shared_ptr<JointState>> received_motor_state_msg_ptr_{ nullptr };

  std::shared_ptr<rclcpp::Publisher<Int8MultiArray>> motor_command_publisher_ = nullptr;

  std::shared_ptr<realtime_tools::RealtimePublisher<Int8MultiArray>> realtime_motor_command_publisher_ = nullptr;

  rclcpp::Subscription<JointState>::SharedPtr motor_state_subscriber_ = nullptr;

  std::map<std::string, double> vel_commands_;
  std::map<std::string, double> pos_state_;
  std::map<std::string, double> vel_state_;

  bool subscriber_is_active_ = false;

  std::shared_ptr<rclcpp::Node> node_;

  void motor_state_cb(const std::shared_ptr<JointState> msg);
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::unique_ptr<std::thread> executor_thread_;

  std::vector<std::string> velocity_command_joint_order_;

  uint connection_check_period_ms_;
  uint connection_timeout_ms_;
};

}  // namespace OMNIBOT

#endif  // OMNIBOT__SYSTEM_HPP_