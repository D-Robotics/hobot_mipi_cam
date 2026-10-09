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



#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <iostream>
#include "imu_manager.hpp"
#include "icm42688.hpp"

using mipi_cam::mipi_calibration;
namespace imu_sensor
{

#define SYSFS_PATH "/sys/bus/iio/devices/"
#define MAX_PATH_LEN 512


static const std::string imu_devices[] = {
    "bmi08x",
    "icm42688",
    "icm42688-gyro",
    "icm42688-accel",
    "icm42688-fsync"
};

int ImuManager::auto_detect_iio_devices() {
    printf("\n=== Detected IIO Devices ===\n");

    DIR *dp = opendir(SYSFS_PATH);
    if (!dp) {
        fprintf(stderr, "Can't open %s: %s\n", SYSFS_PATH, strerror(errno));
        return -1;
    }

    struct dirent *ent;
    while ((ent = readdir(dp))) {
        if (strstr(ent->d_name, "iio:device")) {
            char device_path[MAX_PATH_LEN];
            snprintf(device_path, sizeof(device_path),
                    SYSFS_PATH "%s", ent->d_name);

            // 读取设备名称
            char name_path[MAX_PATH_LEN];
            strcpy(name_path, device_path);
            strcat(name_path, "/name");

            FILE *name_fp = fopen(name_path, "r");

            char device_name[64] = "unknown";
            if (name_fp) {
                if (fgets(device_name, sizeof(device_name), name_fp)) {
                    device_name[strcspn(device_name, "\n")] = '\0';
                    std::string device_name_str(device_name);
                    for (const auto& device : imu_devices) {
                        if (device_name_str.compare(0, device.length(), device) == 0) {
                            DeviceInfo_T device_info;
                            device_info.sensor_type = device_name_str;
                            device_info.path = device_path;
                            device_info.d_name = ent->d_name;
                            if (device_name_str.find("icm42688-") == 0) {
                                devices_map_["icm42688"].push_back(device_info);
                            } else {
                                devices_map_[device].push_back(device_info);
                            }
                        }
                    }
                }
                fclose(name_fp);
            }

            printf("  Device: %-15s | Name: %s\n", ent->d_name, device_name);
        }
    }
    closedir(dp);
    printf("============================\n\n");
    return 0;
}

// 初始化指定类型的传感器
int ImuManager::init_sensor(std::string sensor_type) {
    // 列出所有IIO设备
    auto_detect_iio_devices();
    if (devices_map_.empty()) {
        return -1;
    }
    std::vector<DeviceInfo_T> device_info;

    auto it = devices_map_.find(sensor_type);
    if (it != devices_map_.end()) {
        device_info = it->second;
    } else {
        auto firstE = devices_map_.begin();
        sensor_type = firstE->first;
        device_info = firstE->second;
    }
    imu_handle = createImuDevice(sensor_type);
    if (imu_handle == nullptr) {
        return -1;
    }
    imu_handle->set_path(device_info);
    auto ret = imu_handle->init();
    return ret;
}

// 读取传感器数据
int ImuManager::read_sensor_data(ImuData_T* data) {
    if (imu_handle == nullptr) {
        return -1;
    } 
    return imu_handle->read_data(data);
}

// 释放传感器资源
void ImuManager::release_sensor() {
    if (imu_handle != nullptr) {
        imu_handle->deinit();
    } 
}

std::shared_ptr<imu_base> ImuManager::createImuDevice(const std::string &dev_name) {
    std::shared_ptr<imu_base> dev_ptr;
    if (dev_name == "icm42688") {
        dev_ptr = std::make_shared<icm42688>();
    }
    return dev_ptr;
}


bool ImuManager::loadCalibrationFromEeprom() {
    //从 mipi_calibration 单例拿 CalibrationParams 引用（零拷贝）
    auto &cal_params =
        mipi_calibration::GetInstance().getCalibrationParams();

    if (cal_params.size() >= 1)
    {
        const auto &imu_info = cal_params[0].imu_info_;
        if (!imu_info.empty()) {
            // 5. 取第一组 IMU 标定参数（值拷贝到自己的成员）
            imu_calib_params_ = imu_info[0];
            has_valid_calibration_ = true;
            return true;
        }
    }
    fprintf(stderr,
            "[ImuManager] CalibrationParams is empty.\n ");
    return false;
}

}
