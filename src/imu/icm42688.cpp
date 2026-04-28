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


/* 读取原始数据（有符号整数）*/
int icm42688::read_raw_signed(std::string dev_path, const char *type, const char *axis, int *val) {
    char raw_path[MAX_PATH_LEN];
    FILE *fp;

    snprintf(raw_path, sizeof(raw_path),
    "%s/in_%s_%s_raw", dev_path.c_str(), type, axis);

    fp = fopen(raw_path, "r");
    if (!fp) {
        fprintf(stderr, "Failed to open raw file: %s (error: %s)\n",
                raw_path, strerror(errno));
        return -1;
    }

    if (fscanf(fp, "%d", val) != 1) {
        fclose(fp);
        fprintf(stderr, "Failed to read value from: %s\n", raw_path);
        return -1;
    }
    fclose(fp);
    return 0;
}


/* 读取比例因子 */
int icm42688::read_sensor_scale(std::string dev_path, const char *type, float *scale) {
    char scale_path[MAX_PATH_LEN];
    FILE *fp;

    snprintf(scale_path, sizeof(scale_path),
    "%s/in_%s_scale", dev_path.c_str(), type);

    fp = fopen(scale_path, "r");
    if (!fp) {
        fprintf(stderr, "Failed to open scale file: %s (error: %s)\n",
                scale_path, strerror(errno));
        return -1;
    }

    if (fscanf(fp, "%f", scale) != 1) {
        fclose(fp);
        fprintf(stderr, "Failed to read scale from: %s\n", scale_path);
        return -1;
    }
    fclose(fp);
    return 0;
}

/* 获取当前时间戳（微秒） */
uint64_t icm42688::get_current_timestamp_us() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}




/* 检查设备是否包含指定类型的通道 */
int icm42688::device_has_channels(std::string dev_path, const char *type) {
    char test_path[MAX_PATH_LEN];
    const char *axes[] = {"x", "y", "z"};
    int found = 0;

    printf("Checking channels for %s in %s:\n", type, dev_path.c_str());

    for (int i = 0; i < 3; i++) {
        // 构建路径
        int len = snprintf(test_path, sizeof(test_path), "%s/in_%s_%s_raw", dev_path.c_str(), type, axes[i]);
        if (len >= (int)sizeof(test_path)) {
            fprintf(stderr, "Path too long: %s/in_%s_%s_raw\n", dev_path.c_str(), type, axes[i]);
            continue;
        }

        if (access(test_path, F_OK) == 0) {
            printf("  Found: %s\n", test_path);
            found++;
        } else {
            printf("  Missing: %s (error: %s)\n", test_path, strerror(errno));
        }
    }

    return found == 3;  // 需要所有三个轴都存在
}

// 改进的通用初始化函数
int icm42688::init() {

#if 0

    // 打印传入的设备路径 
    for (const auto& device : dev_info_) {
        if (device.sensor_type == "icm42688-accel") {
            accel_dev_path = device.path;
        } else if (device.sensor_type == "icm42688-gyro") {
            gyro_dev_path = device.path;
        }
    }

    printf("\nInitializing ICM42688 with device accel_dev_path: %s and gyro_dev_path: %s\n", accel_dev_path.c_str(), gyro_dev_path.c_str());

    // 检查设备类型
    int is_accel = device_has_channels(accel_dev_path, "accel");
    int is_gyro = device_has_channels(gyro_dev_path, "anglvel");
    if ((is_accel == 0) || (is_gyro == 0)) {
        return -1;
    }

    // 读取加速度计比例因子
    if (read_sensor_scale(accel_dev_path, "accel", &accel_scale) != 0) {
        fprintf(stderr, "Using default accel scale: 0.004788403\n");
        accel_scale = 0.004788403f; // ±16g时的默认值
        printf("\n1111111111111111111111111\n");
    }

    // 读取陀螺仪比例因子
    if (read_sensor_scale(gyro_dev_path, "anglvel", &gyro_scale) != 0) {
        fprintf(stderr, "Using default gyro scale: 0.001065\n");
        gyro_scale = 0.001065f; // ±2000dps时的默认值
        printf("\n2222222222222222222222222222222222\n");
    }

    printf("\nICM42688: Initialization complete\n");
    printf("  Accel path: %s, scale: %f\n", accel_dev_path.c_str(), accel_scale);
    printf("  Gyro path: %s, scale: %f\n", gyro_dev_path.c_str(), gyro_scale);

    // 验证通道可访问性
    printf("\nVerifying channel accessibility:\n");

    initialized = 1;
    return 0;
#else

    int ret;

    ret = inv_icm42600_i2c_fd_auto_init();
    printf("[inv_icm42600_i2c_fd_auto_init]run return: %d.\n",ret);
    if (ret != 0) {
        return -1;
    }

    ret = inv_icm42600_soft_reset();
    printf("[inv_icm42600_soft_reset]run return: %d.\n",ret);
    if (ret != 0) {
        return -1;
    }
    ret = inv_icm42600_all_sel();
    printf("[inv_icm42600_all_sel]run return: %d.\n",ret);
    if (ret != 0) {
        return -1;
    }
    ret = inv_icm42600_pwr_mgmt(
        INV_ICM_42600_GYRO_LN,
        INV_ICM_42600_ACCEL_LN,
        true,
        true);
    printf("[inv_icm42600_pwr_mgmt]run return: %d.\n",ret);
    if (ret != 0) {
        return -1;
    }
    ret = inv_icm42600_filt_setting();
    printf("[inv_icm42600_filt_setting]run return: %d.\n",ret);
    if (ret != 0) {
        return -1;
    }
    //usleep(1000000);
    initialized = 1;
    return 0;

#endif 
}

// 读取函数
int icm42688::read(ImuData_T *data) {
    if (!initialized || data == nullptr) {
        fprintf(stderr, "ICM42688 not initialized\n");
        return -1;
    }

#if 0
    int accel_x_raw, accel_y_raw, accel_z_raw;
    int gyro_x_raw, gyro_y_raw, gyro_z_raw;

    // 读取加速度计数据
    int accel_read = 0;
    if (read_raw_signed(accel_dev_path, "accel", "x", &accel_x_raw) == 0 &&
        read_raw_signed(accel_dev_path, "accel", "y", &accel_y_raw) == 0 &&
        read_raw_signed(accel_dev_path, "accel", "z", &accel_z_raw) == 0) {
        accel_read = 1;
    } else {
        fprintf(stderr, "Failed to read accelerometer data\n");
    }

    // 读取陀螺仪数据
    int gyro_read = 0;
    if (read_raw_signed(gyro_dev_path, "anglvel", "x", &gyro_x_raw) == 0 &&
        read_raw_signed(gyro_dev_path, "anglvel", "y", &gyro_y_raw) == 0 &&
        read_raw_signed(gyro_dev_path, "anglvel", "z", &gyro_z_raw) == 0) {
        gyro_read = 1;
    } else {
        fprintf(stderr, "Failed to read gyroscope data\n");
    }

    // 转换为实际物理值
    if (accel_read) {
        data->ax = accel_x_raw * accel_scale * gravity;
        data->ay = accel_y_raw * accel_scale * gravity;
        data->az = accel_z_raw * accel_scale * gravity;
    } else {
        data->ax = 0.0f;
        data->ay = 0.0f;
        data->az = 0.0f;
    }

    if (gyro_read) {
        data->gx = gyro_x_raw * gyro_scale * M_PI / 180.0;
        data->gy = gyro_y_raw * gyro_scale * M_PI / 180.0;
        data->gz = gyro_z_raw * gyro_scale * M_PI / 180.0;
    } else {
        data->gx = 0.0f;
        data->gy = 0.0f;
        data->gz = 0.0f;
    }

    // 使用系统时间作为时间戳
    data->timestamp = get_current_timestamp_us();

    // ICM42688没有磁力计
    data->mx = 0.0f;
    data->my = 0.0f;
    data->mz = 0.0f;

    // 状态正常
    data->status = 0;

    return (accel_read && gyro_read) ? 0 : 1; // 部分成功返回1，完全成功返回0
#else
    struct inv_icm42600_data_packege dtpkt = INV_ICM42600_EMPTY_PACKAGE;
    int ret;

    ret = inv_icm42600_origin_data_read(&dtpkt, true);
    if (ret != 0) {
        return -1;
    }
    // system("clear");
    //for(int i=0; i<50; i++)printf("=");printf("\n");

    //printf("[origin data read]function return: %d\n", ret);

    ret = inv_icm42600_data_process(&dtpkt);
    if (ret != 0) {
        return -1;
    }
    //printf("[data process]function return: %d\n", ret);

    //printf("[fsync test]");
    //if (dtpkt.fsync_success) {
    //printf("Successed.\n");
    // printf("[temp_data]%.3f unit:Celsius\n", dtpkt.temp_processed);
    //printf("[accel_data] x:%8.3f    y:%8.3f    z:%8.3f    unit:g\n", dtpkt.accel_data_processed.x, dtpkt.accel_data_processed.y, dtpkt.accel_data_processed.z);
    // printf("[gyro_data]  x:%8.3f    y:%8.3f    z:%8.3f    unit:dps\n", dtpkt.gyro_data_processed.x, dtpkt.gyro_data_processed.y, dtpkt.gyro_data_processed.z);
    // printf("[timestamp_data]%ld\n", dtpkt.accel_data_processed.timestamp);
    data->ax = dtpkt.accel_data_processed.x * gravity;
    data->ay = dtpkt.accel_data_processed.y * gravity;
    data->az = dtpkt.accel_data_processed.z * gravity;
    data->gx = dtpkt.gyro_data_processed.x * M_PI / 180.0;
    data->gy = dtpkt.gyro_data_processed.y * M_PI / 180.0;
    data->gz = dtpkt.gyro_data_processed.z * M_PI / 180.0;
    data->timestamp = dtpkt.accel_data_processed.timestamp * 1000;

    // ICM42688没有磁力计
    data->mx = 0.0f;
    data->my = 0.0f;
    data->mz = 0.0f;

    // 状态正常
    data->status = 0;
    return 0;

#endif
}

// 释放函数
int icm42688::deinit() {
    initialized = 0;
    return 0;
}

int icm42688::set_path(std::vector<DeviceInfo_T> &dev_info) {
    dev_info_ = dev_info;
    return 0;
}

}
