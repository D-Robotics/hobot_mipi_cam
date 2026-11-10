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
#include <rclcpp/rclcpp.hpp>

namespace imu_sensor
{
/* ========== 固定配置 ========== */
static constexpr char DEFAULT_IIO_DEV[] = "iio:device1";   // IIO 设备名
static constexpr int BUF_LENGTH = 512;                     // 缓冲区采样数

/* 单次采样的内存布局（packed，与 IIO 通道顺序无关） */
#pragma pack(push, 1)
struct Sample {
    int16_t  temp;
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    uint8_t  padding[2];
    uint64_t timestamp;           // 纳秒
};
#pragma pack(pop)

class icm42688 : public imu_base {
  public:
    icm42688() {};
    ~icm42688() {};
    int init() override;
    int read_data(ImuData_T *imu_data) override;
    int deinit() override;
    int set_path(std::vector<DeviceInfo_T>  &dev_info) override;

  private:
    struct DeviceChannel {
      std::string sensor_type;
      std::string path;
      std::string d_name;
    };

    bool write_sysfs_int(const std::string & path, int val);
    bool write_sysfs_double(const std::string & path, double val);
    bool read_sysfs_double(const std::string & path, double & out);
    bool read_sysfs_int64(const std::string & path, int64_t & out);
    bool configure_device(double odr);
    bool path_exists(const std::string &path) const;
    bool has_split_iio_layout() const;
    int read_split_data(ImuData_T *imu_data);

    std::string device_name_;
    std::string sysfs_root_;
    std::string dev_path_;
    int fd_ = -1;

    int ref_count;                   // 引用计数
    int initialized;                 // 初始化标志
    double accel_scale_ = 1.0;               // 加速度计量程
    double gyro_scale_ = 1.0;                // 陀螺仪量程
    double temp_scale_ = 1.0;               // 温度
    double temp_offset_ = 0.0;                // 温度偏差
    double gravity = 9.81;                   // 重力加速度
    std::string accel_dev_path; // 加速度计设备路径
    std::string gyro_dev_path;  // 陀螺仪设备路径
    std::string fsync_dev_path;
    double split_accel_scale_ = 1.0;
    double split_gyro_scale_ = 1.0;
    bool split_iio_enabled_ = false;

    std::vector<DeviceInfo_T> dev_info_;
};

}

#endif  // HOBOT_ICM42688_HPP_
