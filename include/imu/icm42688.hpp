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

#ifndef HOBOT_ICM42688_HPP_
#define HOBOT_ICM42688_HPP_

#include "imu_base.hpp"

namespace imu_sensor
{
class icm42688 : public imu_base {
  public:
    //icm42688(std::string accel_path, std::string gyro_path):accel_dev_path(accel_path),gyro_dev_path(gyro_path) {}
    icm42688() {};
    ~icm42688() {};
    int init() override;
    int read(ImuData_T *imu_data) override;
    int deinit() override;
    int set_path(std::vector<DeviceInfo_T>  &dev_info) override;
  
  private:
    int read_raw_signed(std::string dev_path, const char *type, const char *axis, int *val);
    int read_sensor_scale(std::string dev_path, const char *type, float *scale);
    uint64_t get_current_timestamp_us();
    int device_has_channels(std::string dev_path, const char *type);


    int ref_count;                   // 引用计数
    int initialized;                 // 初始化标志
    float accel_scale;               // 加速度计量程
    float gyro_scale;                // 陀螺仪量程
    //char accel_dev_path[MAX_PATH_LEN]; // 加速度计设备路径
    //char gyro_dev_path[MAX_PATH_LEN];  // 陀螺仪设备路径
    std::string accel_dev_path; // 加速度计设备路径
    std::string gyro_dev_path;  // 陀螺仪设备路径

    std::vector<DeviceInfo_T> dev_info_;
};
}

#endif  // HOBOT_ICM42688_HPP_
