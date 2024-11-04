// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

#include "hardware_bridge/hardware_bridge_node.hpp"

#include <pthread.h>
#include <sched.h>

#include "pluginlib/class_list_macros.hpp"

namespace tita_locomotion
{
HardwareBridge::HardwareBridge() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareBridge::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // node_ = rclcpp::Node::make_shared("hardware_bridge_node");
  // executor_.add_node(node_);
  // std::thread([this]() { executor_.spin(); }).detach();
  auto ctrl_mode = info_.hardware_parameters["ctrl_mode"];
  direct_mode_ = ctrl_mode.compare("mcu") == 0 ? false : true;
  if (direct_mode_) {
    pid_t pid = getpid();
    sched_param sched;
    sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(pid, SCHED_RR, &sched);
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched) != 0) {
      std::cerr << "Failed to set current thread's priority." << std::endl;
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (hardware_interface::ComponentInfo component : info.joints) {
    Joint joint;
    joint.name = component.name;
    mJoints.push_back(joint);
  }
  for (hardware_interface::ComponentInfo component : info.sensors) {
    std::string sensor_name = component.name;
    mImu.name = sensor_name;
  }
  robot_ = std::make_unique<tita_robot>(mJoints.size());
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HardwareBridge::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (Joint & joint : mJoints) {
    interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &(joint.position)));
    interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocity)));
    interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effort)));
  }

  for (hardware_interface::ComponentInfo component : info_.sensors) {
    if (component.name == mImu.name) {
      for (uint i = 0; i < 4; i++) {
        interfaces.emplace_back(hardware_interface::StateInterface(
          component.name, component.state_interfaces[i].name, &mImu.orientation[i]));
      }
      for (uint i = 0; i < 3; i++) {
        interfaces.emplace_back(hardware_interface::StateInterface(
          component.name, component.state_interfaces[i + 4].name, &mImu.angular_velocity[i]));
      }
      for (uint i = 0; i < 3; i++) {
        interfaces.emplace_back(hardware_interface::StateInterface(
          component.name, component.state_interfaces[i + 7].name, &mImu.linear_acceleration[i]));
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareBridge::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (Joint & joint : mJoints) {
    interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &(joint.positionCommand)));
    interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocityCommand)));
    interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effortCommand)));
    interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kp", &(joint.kp)));
    interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kd", &(joint.kd)));
  }
  return interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareBridge::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (direct_mode_) {
    robot_->set_motors_sdk(direct_mode_);
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareBridge::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HardwareBridge::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // read from can bus
  auto q = robot_->get_joint_q();
  auto v = robot_->get_joint_v();
  auto t = robot_->get_joint_t();
  for (size_t id = 0; id < mJoints.size(); id++) {
    mJoints[id].position = q[id];
    mJoints[id].velocity = v[id];
    mJoints[id].effort = t[id];
    if (id == 1 || id == 5) {
      if (mJoints[id].position < -2.5) {
        mJoints[id].position += 2 * M_PI;
      }
    }
  }
  auto quat = robot_->get_imu_quaternion();
  auto accl = robot_->get_imu_acceleration();
  auto gyro = robot_->get_imu_angular_velocity();

  for (size_t id = 0; id < 3; id++) {
    mImu.linear_acceleration[id] = accl[id];
    mImu.angular_velocity[id] = gyro[id];
    mImu.orientation[id] = quat[id];
  }

  mImu.orientation[3] = quat[3];
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HardwareBridge::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (direct_mode_) {
    std::vector<double> cmd_torque;
    for (size_t id = 0; id < mJoints.size(); id++) {
      auto torque = mJoints[id].effortCommand +
                    mJoints[id].kp * (mJoints[id].positionCommand - mJoints[id].position) +
                    mJoints[id].kd * (mJoints[id].velocityCommand - mJoints[id].velocity);
      // motor.torque = torque;
      // motor.kp = 0.0f;
      // motor.kd = 0.0f;
      // motor.velocity = 0.0f;
      // motor.position = 0.0f;
      cmd_torque.push_back(torque);
    }
    robot_->set_target_joint_t(cmd_torque);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace tita_locomotion
PLUGINLIB_EXPORT_CLASS(tita_locomotion::HardwareBridge, hardware_interface::SystemInterface)
