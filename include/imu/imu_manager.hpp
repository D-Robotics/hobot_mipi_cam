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

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <memory>

#include "imu_base.hpp"

#include "hobot_mipi_calibration.hpp"

using mipi_cam::Imu_params;
namespace imu_sensor
{

class ImuManager {
 public:
  ImuManager() {}
  virtual ~ImuManager() {}

  int init_sensor(std::string sensor_type);
  int read_sensor_data(ImuData_T* data);
  void release_sensor();

  std::shared_ptr<imu_base> createImuDevice(const std::string &dev_name);

  bool loadCalibrationFromEeprom();

  /// 是否已有有效标定数据
  bool hasValidCalibration() const { return has_valid_calibration_; }
  /// 获取 IMU 标定参数（const 引用，零拷贝）
  const Imu_params &getCalibParams() const { return imu_calib_params_; }

private:
  int auto_detect_iio_devices();

  //std::map<std::string,std::string> devices_map_;
  std::map<std::string,std::vector<DeviceInfo_T>> devices_map_;
  std::shared_ptr<imu_base> imu_handle;

  Imu_params imu_calib_params_;
  bool has_valid_calibration_ = false;
};
}

#endif // SENSOR_MANAGER_H
