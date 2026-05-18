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


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <stdint.h>
#include <math.h>
#include <iostream>
#include "icm42688.hpp"
#include "inv_icm42600.h"
#include <cmath>


namespace imu_sensor
{

#define SYSFS_PATH "/sys/bus/iio/devices/"
#define MAX_PATH_LEN 512
#define MAX_DEVICES 10  // 最大支持的设备数量


int icm42688::set_path(std::vector<DeviceInfo_T> &dev_info) {
    dev_info_ = dev_info;
    return 0;
}
/* ========== sysfs 辅助函数 ========== */
bool icm42688::write_sysfs_int(const std::string & path, int val) {
    FILE *fp = fopen(path.c_str(), "w");
    if (!fp) {
        RCLCPP_ERROR(rclcpp::get_logger("icm42600"), "Failed to open %s: %s",
                     path.c_str(), strerror(errno));
        return false;
    }
    fprintf(fp, "%d", val);
    fclose(fp);
    return true;
}

bool icm42688::write_sysfs_double(const std::string & path, double val) {
    FILE *fp = fopen(path.c_str(), "w");
    if (!fp) {
        RCLCPP_ERROR(rclcpp::get_logger("icm42600"), "Failed to open %s: %s",
                     path.c_str(), strerror(errno));
        return false;
    }
    fprintf(fp, "%f", val);
    fclose(fp);
    return true;
}

bool icm42688::read_sysfs_double(const std::string & path, double & out) {
    FILE *fp = fopen(path.c_str(), "r");
    if (!fp) {
        RCLCPP_ERROR(rclcpp::get_logger("icm42600"), "Failed to open %s: %s",
                     path.c_str(), strerror(errno));
        return false;
    }
    float f;
    if (fscanf(fp, "%f", &f) != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("icm42600"), "Failed to parse %s", path.c_str());
        fclose(fp);
        return false;
    }
    out = static_cast<double>(f);
    fclose(fp);
    return true;
}


// 改进的通用初始化函数
int icm42688::init() {

    int ret;

    // 获取参数
    //device_name_ = this->get_parameter("device").as_string();
    double odr = 200.0;

    // 检查 ODR 是否合法
    if (odr != 200.0 && odr != 500.0) {
        RCLCPP_WARN(rclcpp::get_logger("icm42688"),
                    "ODR %.1f not in {200,500}, falling back to 200 Hz", odr);
        odr = 200.0;
    }

    // 构建 sysfs 路径
    sysfs_root_ = "/sys/bus/iio/devices/" + dev_info_[0].d_name;
    dev_path_   = "/dev/" + dev_info_[0].d_name;

    // ----- 初始化 IIO 设备 -----
    if (!configure_device(odr)) {
        RCLCPP_FATAL(rclcpp::get_logger("icm42688"), "Failed to configure IIO device, exiting.");
        return -1;
    }
    //usleep(1000000);
    initialized = 1;
    return 0;

}
 

// 释放函数
int icm42688::deinit() {
    // 关闭缓冲区并关闭设备
    if (initialized) {
        initialized = 0;
        write_sysfs_int(sysfs_root_ + "/buffer/enable", 0);
        if (fd_ >= 0) {
            close(fd_);
        }
    }
    return 0;
}

// 配置 IIO 设备：写入 ODR，使能通道，设置缓冲区长度，启动缓冲区
bool icm42688::configure_device(double odr) {
    // 1. 设置采样频率（ODR）
    std::string freq_path = sysfs_root_ + "/sampling_frequency";
    if (!write_sysfs_double(freq_path, odr)) {
        RCLCPP_ERROR(rclcpp::get_logger("icm42688"), "Cannot set sampling_frequency to %.1f", odr);
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("icm42688"), "Set sampling_frequency = %.1f Hz", odr);

    // 2. 使能扫描元素（通道顺序不重要，由 IIO 驱动决定）
    const std::vector<std::string> scan_elements = {
        "in_temp_en",
        "in_accel_x_en", "in_accel_y_en", "in_accel_z_en",
        "in_anglvel_x_en", "in_anglvel_y_en", "in_anglvel_z_en",
        "in_timestamp_en"
    };
    for (const auto & elem : scan_elements) {
        if (!write_sysfs_int(sysfs_root_ + "/scan_elements/" + elem, 1)) {
            return false;
        }
    }

    // 3. 设置缓冲区长度
    if (!write_sysfs_int(sysfs_root_ + "/buffer/length", BUF_LENGTH)) {
        return false;
    }

    // 4. 读取 scale / offset（用于物理量转换）
    read_sysfs_double(sysfs_root_ + "/in_accel_scale", accel_scale_);
    read_sysfs_double(sysfs_root_ + "/in_anglvel_scale", gyro_scale_);
    read_sysfs_double(sysfs_root_ + "/in_temp_scale", temp_scale_);
    read_sysfs_double(sysfs_root_ + "/in_temp_offset", temp_offset_);

    RCLCPP_INFO(rclcpp::get_logger("icm42688"),
                "Scales: accel=%.9f, gyro=%.9f, temp=%.9f, temp_offset=%.3f",
                accel_scale_, gyro_scale_, temp_scale_, temp_offset_);

    // 5. 使能缓冲区（启动传感器）
    if (!write_sysfs_int(sysfs_root_ + "/buffer/enable", 1)) {
        return false;
    }

    // 6. 打开 IIO 字符设备
    fd_ = open(dev_path_.c_str(), O_RDONLY);
    if (fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("icm42688"), "Cannot open %s: %s",
                     dev_path_.c_str(), strerror(errno));
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("icm42688"), "IIO device %s opened successfully", dev_path_.c_str());
    return true;
}


// 读取函数
int icm42688::read_data(ImuData_T *data) {
    if (!initialized || data == nullptr) {
        fprintf(stderr, "ICM42688 not initialized\n");
        return -1;
    }
    Sample sample;
    ssize_t read_len;
    read_len = read(fd_, &sample, sizeof(Sample));
    if (read_len < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("icm42688"), "read error: %s", strerror(errno));
        return -1;
    }
    if (read_len != sizeof(Sample)) {
        RCLCPP_WARN(rclcpp::get_logger("icm42688"), "Short read: %zd bytes", read_len);
        return -1;
    }

    data->ax = sample.accel_x * accel_scale_;
    data->ay = sample.accel_y * accel_scale_;
    data->az = sample.accel_z * accel_scale_;
    data->gx = sample.gyro_x * gyro_scale_;
    data->gy = sample.gyro_y * gyro_scale_;
    data->gz = sample.gyro_z * gyro_scale_;
    data->timestamp = sample.timestamp;

    // ICM42688没有磁力计
    data->mx = 0.0f;
    data->my = 0.0f;
    data->mz = 0.0f;

    // 状态正常
    data->status = 0;
    return 0;

}

}
