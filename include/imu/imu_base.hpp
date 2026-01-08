// Copyright (c) 2024，D-Robotics.
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

#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <map>

#ifndef HOBOT_IMU_BASE_HPP_
#define HOBOT_IMU_BASE_HPP_


namespace imu_sensor
{
// 定义传感器数据结构
typedef struct {
    float ax, ay, az;     // 加速度计数据
    float gx, gy, gz;     // 陀螺仪数据
    float mx, my, mz;     // 磁力计数据
    uint64_t timestamp;       // 时间戳
    int status;           // 状态码
} ImuData_T;

typedef struct {
  std::string sensor_type;
  std::string path;
} DeviceInfo_T;

class imu_base {
  public:
    imu_base() {};
    virtual ~imu_base() {};
    virtual int init() = 0;
    virtual int read(ImuData_T *imu_data) = 0;
    virtual int deinit() = 0;
    virtual int set_path(std::vector<DeviceInfo_T> &dev_info) = 0;
};
}

#endif  // HOBOT_IMU_NODE_HPP_
