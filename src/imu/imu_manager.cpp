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
#include <ctype.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <iostream>
#include <algorithm>
#include <utility>
#include "imu_manager.hpp"
#include "icm42688.hpp"
#include "event_device.hpp"

using mipi_cam::mipi_calibration;
namespace imu_sensor
{

#define SYSFS_PATH "/sys/bus/iio/devices/"
#define MAX_PATH_LEN 512


static const char *kImuFamilies[] = {
    "icm42688",
    // "icm42708",  // 后续扩展：驱动就绪后放开即可
};
static const char *kEventSensorType = "event";

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
                    for (const char *family : kImuFamilies) {
                        const std::string family_str(family);
                        // 匹配 <family> 或其 split 变体 <family>-{gyro,accel,fsync}
                        if (device_name_str.compare(0, family_str.length(),
                                                    family_str) != 0) {
                            continue;
                        }
                            DeviceInfo_T device_info;
                            device_info.sensor_type = device_name_str;
                            device_info.path = device_path;
                            device_info.d_name = ent->d_name;
                        devices_map_[family_str].push_back(device_info);
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

// 探测 /dev/input/eventX 中的 IMU 设备（EVIOCGNAME 匹配设备族）
int ImuManager::auto_detect_input_devices() {
    devices_map_.erase(kEventSensorType);
    DIR *dp = opendir("/dev/input");
    if (!dp) {
        // 无 /dev/input（正常板子走 IIO），静默返回
        return -1;
    }
    std::vector<std::pair<int, DeviceInfo_T>> candidates;  // 按 eventX 数字序
    struct dirent *ent;
    while ((ent = readdir(dp))) {
        const char *p = ent->d_name;
        if (strncmp(p, "event", 5) != 0 ||
            !isdigit(static_cast<unsigned char>(p[5]))) {
            continue;
        }
        std::string dev_path = std::string("/dev/input/") + ent->d_name;
        int fd = open(dev_path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            continue;
        }
        char name[128] = {0};
        if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0) {
            // input 事件设备名为 "<imu_dev>-sensor"（如 "icm42688-sensor"），
            // 精确匹配避免误命中
            bool matched = false;
            for (const char *family : kImuFamilies) {
                const std::string expect = std::string(family) + "-sensor";
                if (name == expect) {
                    matched = true;
                    break;
                }
            }
            if (matched) {
                DeviceInfo_T info;
                info.sensor_type = kEventSensorType;
                info.path = dev_path;
                info.d_name = ent->d_name;
                candidates.emplace_back(atoi(p + 5), info);
                printf("  Event Device: %-15s | Name: %s\n", ent->d_name, name);
            }
        }
        //close(fd);
    }
    closedir(dp);
    if (candidates.empty()) {
        return -1;
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const std::pair<int, DeviceInfo_T> &a,
                 const std::pair<int, DeviceInfo_T> &b) {
                return a.first < b.first;
              });
    for (const auto &c : candidates) {
        devices_map_[kEventSensorType].push_back(c.second);
    }
    return 0;
}

// 尝试初始化 event 设备（失败返回 -1，不回退，由调用方决定）
// 成功时同时记录 event_handle_（供 read_sensor_data 卡死降级判断）
int ImuManager::tryInitEvent() {
    auto it = devices_map_.find(kEventSensorType);
    if (it == devices_map_.end() || it->second.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("imu_manager"),
                    "no supported imu input event device detected");
        return -1;
    }
    auto ev = std::make_shared<event_device>();
    ev->set_path(it->second);
    if (ev->init() == 0) {
        imu_handle = ev;
        //event_handle_ = ev;
        return 0;
    }
    RCLCPP_WARN(rclcpp::get_logger("imu_manager"),
                "event device init failed");
    return -1;
}

// IIO 原方式兜底初始化（event 卡死降级路径复用 init_sensor 的第 5 步）
void ImuManager::init_iio_fallback() {
    auto_detect_iio_devices();
    std::vector<DeviceInfo_T> device_info;
    std::string iio_type;
    for (const auto &dev : devices_map_) {
        if (dev.first == kEventSensorType) {
            continue;
        }
        iio_type = dev.first;
        device_info = dev.second;
        break;
    }
    if (device_info.empty()) {
        return;
    }
    imu_handle = createImuDevice(iio_type);
    if (imu_handle != nullptr) {
        imu_handle->set_path(device_info);
        if (imu_handle->init() != 0) {
            imu_handle.reset();
        }
    }
}

// 初始化指定类型的传感器
int ImuManager::init_sensor(std::string sensor_type) {
    if (sensor_type.rfind("/dev/input/event", 0) == 0) {
        DeviceInfo_T info;
        info.sensor_type = kEventSensorType;
        info.path = sensor_type;
        info.d_name = sensor_type.substr(strlen("/dev/input/"));
        devices_map_[kEventSensorType] = {info};
        return tryInitEvent();
    }

    auto_detect_input_devices();
    auto_detect_iio_devices();

    if (sensor_type == kEventSensorType) {
        return tryInitEvent();
    }

    if (sensor_type.empty()) {
        if (devices_map_.count(kEventSensorType) > 0) {
            if (tryInitEvent() == 0) {
                return 0;
            }
            RCLCPP_WARN(rclcpp::get_logger("imu_manager"),
                        "event device init failed, fallback to IIO");
        }
    }
    std::vector<DeviceInfo_T> device_info;
    std::string iio_type;
    auto it = devices_map_.find(sensor_type);
    if (it != devices_map_.end() && it->first != kEventSensorType) {
        iio_type = it->first;
        device_info = it->second;
    } else {
        for (const auto &dev : devices_map_) {
            if (dev.first == kEventSensorType) {
                continue;
            }
            iio_type = dev.first;
            device_info = dev.second;
            break;
        }
    }
    if (device_info.empty()) {
        return -1;
    }
    imu_handle = createImuDevice(iio_type);
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
    if (!degraded_to_iio_ && event_handle_ != nullptr &&
        event_handle_->isStalled()) {
        auto ev = std::dynamic_pointer_cast<event_device>(imu_handle);
        if (ev && ev == event_handle_) {
            RCLCPP_WARN(rclcpp::get_logger("imu_manager"),
                        "event channel stalled, degrade to IIO read");
            ev->deinit();
            ev.reset();
            imu_handle = event_handle_ = nullptr;
            // 重新探测 IIO 设备并走原 IIO 直接读方式
            init_iio_fallback();
            degraded_to_iio_ = true;
            if (imu_handle == nullptr) {
                return -1;
            }
        }
    }
    return imu_handle->read_data(data);
}

// 释放传感器资源
void ImuManager::release_sensor() {
    if (imu_handle != nullptr) {
        imu_handle->deinit();
    } 
    imu_handle.reset();
    //event_handle_.reset();
    degraded_to_iio_ = false;
}

std::shared_ptr<imu_base> ImuManager::createImuDevice(const std::string &dev_name) {
    std::shared_ptr<imu_base> dev_ptr;
    if (dev_name == kEventSensorType) {
        dev_ptr = std::make_shared<event_device>();
    } else if (dev_name == "icm42688") {
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
