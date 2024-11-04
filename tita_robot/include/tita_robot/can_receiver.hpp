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

#ifndef MOTORS_CAN__MOTORS_CAN_API_HPP_
#define MOTORS_CAN__MOTORS_CAN_API_HPP_

#include <algorithm>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "protocol/can_utils.hpp"
namespace can_device
{
#define PACKED __attribute__((__packed__))
#define GRAVITY 9.81f
#define CAN_ID_MOTOR_IN 0x108
#define CAN_ID_IMU1 0x118
#define CAN_ID_STATE 0x102

  struct api_motor_in_t
  {
    uint32_t timestamp;
    float position;
    float velocity;
    float torque;
  } PACKED;
  /* 44 bytes */
  struct api_imu_data_t
  {
    uint32_t timestamp;
    float accl[3];
    float gyro[3];
    float quaternion[4]; // x y z w
    float temperature;
  } PACKED;
  
  struct api_motor_status_t
  {
    uint32_t timestamp;
    uint16_t key;
    uint8_t value[8];
  } PACKED;

  class MotorsImuCanReceiveApi
  {
  public:
    MotorsImuCanReceiveApi(size_t size)
    {
      register_motors_device_can_filter();
      api_motor_in_t default_motor_in;
      std::memset(&default_motor_in, 0x00U, sizeof(api_motor_in_t));
      motors_in_.resize(size, default_motor_in);
      std::memset(&imu_data_, 0x00U, sizeof(api_imu_data_t));
    }
    ~MotorsImuCanReceiveApi() {}
    const std::vector<api_motor_in_t> *get_motors_in() const { return &motors_in_; }
    const api_imu_data_t *get_imu_data() const { return &imu_data_; }
    const api_motor_status_t *get_motors_status() const { return &motors_status_; }

  private:
#define MIN_TIME_OUT_US 1'000L     // 1ms
#define MAX_TIME_OUT_US 3'000'000L // 3s
#define CAN_MASK_AS_MOTORS_INFO (0x7E0U)
    std::string motors_can_interface = "can0";
    std::string motors_can_name = "motors_can";
    bool motors_can_extended_frame = false;
    bool motors_can_rx_is_block = false;
    int64_t motors_timeout_us = MAX_TIME_OUT_US;
    uint8_t motors_can_id_offset = 0x00U;

    void register_motors_device_can_filter();
    can_device::socket_can::can_fd_callback motors_can_receive_callback =
        std::bind(&MotorsImuCanReceiveApi::board_can_data_callback, this, std::placeholders::_1);

    std::shared_ptr<can_device::socket_can::CanDev> motors_can_receive_api =
        std::make_shared<can_device::socket_can::CanDev>(
            motors_can_interface, motors_can_name, motors_can_extended_frame, motors_can_receive_callback,
            motors_can_rx_is_block, motors_timeout_us, motors_can_id_offset);

    void board_can_data_callback(std::shared_ptr<struct canfd_frame> recv_frame);
    void motors_data_callback(std::shared_ptr<struct canfd_frame> recv_frame);
    void imu_data_callback(std::shared_ptr<struct canfd_frame> recv_frame);
    void motors_status_callback(std::shared_ptr<struct canfd_frame> recv_frame);

    std::vector<api_motor_in_t> motors_in_;
    api_imu_data_t imu_data_;
    api_motor_status_t motors_status_;

    mutable std::shared_mutex motors_in_mutex_, imu_mutex_;
  };
} // namespace can_device

#endif // MOTORS_CAN__MOTORS_CAN_API_HPP_
