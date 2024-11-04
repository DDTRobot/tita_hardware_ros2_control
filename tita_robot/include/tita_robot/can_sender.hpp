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

#ifndef MOTORS_CAN__MOTORS_CAN_SEND_API_HPP_
#define MOTORS_CAN__MOTORS_CAN_SEND_API_HPP_

#include <algorithm>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "protocol/can_utils.hpp"

namespace can_device
{

#define CAN_ID_SEND_MOTORS (0x120U)
#define CAN_ID_CHANNEL_INPUT (0x12DU)
#define CAN_ID_RPC_REQUEST (0x170U)

  enum ApiRpcKey
  {
    RPC_UNDEFINED = 0x000,
    GET_MODEL_INFO = 0x100,
    GET_SERIAL_NUMBER = 0x101,
    SET_READY_NEXT = 0x200,
    SET_BOARDCAST = 0x201,
    SET_INPUT_MODE = 0x221,
    SET_STAND_MODE = 0x222,
    SET_HEAD_MODE = 0x223,
    SET_JUMP = 0x231,
    SET_MOTOR_ZERO = 0x280,
  };
  struct ChannelInput
  {
    uint32_t timestamp;
    float forward;
    float yaw;
    float pitch;
    float roll;
    float height;
    float split;
    float tilt;
    float forward_accel;
    float yaw_accel;
  };
  struct RpcRequest
  {
    uint32_t timestamp;
    uint16_t key;
    uint32_t value;
  };
  struct motor_out
  {
    uint32_t timestamp;
    float position;
    float kp;
    float velocity;
    float kd;
    float torque;
  };

  class MotorsCanSendApi
  {
  public:
    MotorsCanSendApi(size_t size) { (void)size; }
    ~MotorsCanSendApi() = default;
    bool send_motors_can(std::vector<motor_out> motors);
    bool send_command_can_channel_input(ChannelInput data);
    bool send_command_can_rpc_request(RpcRequest data);

  private:
#define MIN_TIME_OUT_US 1'000L     // 1ms
#define MAX_TIME_OUT_US 3'000'000L // 3s
    std::string can_interface = "can0";
    std::string can_name = "motors_can_send";
    int64_t timeout_us = MAX_TIME_OUT_US;
    uint8_t can_id_offset = 0x00U;
    bool can_extended_frame = false;
    bool can_fd_mode = true;

    std::shared_ptr<can_device::socket_can::CanDev> can_send_api_ =
        std::make_shared<can_device::socket_can::CanDev>(
            can_interface, can_name, can_extended_frame, can_fd_mode, timeout_us, can_id_offset);

    inline uint32_t get_current_time()
    {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      return std::move(tv.tv_sec * 1000000 + tv.tv_usec);
    }
  };
} // namespace can_device

#endif // MOTORS_CAN__MOTORS_CAN_SEND_API_HPP_
