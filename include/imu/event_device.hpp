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

#ifndef HOBOT_EVENT_DEVICE_HPP_
#define HOBOT_EVENT_DEVICE_HPP_

#include "imu_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

namespace imu_sensor
{
// InvenSense ICM 系列 input event 读取（icm42688 协议，S100 实测
// 2026-10-09/10；同协议传感器族见 event_device.cpp 的 kImuFamilies，
// 后续扩展 icm42708 等只需在表中加一行）：
//   /dev/input/eventX（EVIOCGNAME 匹配设备名），一帧 = 9 个连续
//   EV_MSC/MSC_RAW 事件（按位置 index 解码）+ EV_SYN 帧边界：
//     index 0-2: accel x/y/z (int16 lsb)
//     index 3-5: gyro x/y/z (int16 lsb)
//     index 6/7: 恒 0（占位）
//     index 8:   帧计数 count（丢帧检测）
//   采样时刻在每个事件的 ev.time（帧内同值，µs 精度，≈DATA_THS_INT 中断时刻）；
//   物理量换算用 IIO 驱动导出的 scale 相乘（与 icm42688.cpp 的 IIO buffer
//   通路 sample.*_x * *_scale_ 同款）：init 时从 /sys/bus/iio/devices 本族
//   设备读 in_accel_scale（m/s²/LSB）/ in_anglvel_scale（rad/s/LSB），event
//   帧 raw 与 IIO raw 同源（同一驱动/mount-matrix/满量程，S100 实测两条
//   通路重力方向一致），直接相乘；读不到 scale 时用 icm42688 满量程
//   兜底值（±16g→0.004788403 m/s²/LSB、±2000dps→0.001065264 rad/s/LSB）。
//   事件流由驱动 DRDY 中断直接驱动，与 IIO buffer 引擎无关（不要碰
//   buffer sysfs：一旦使能 buffer，事件流耦合进引擎并引入其状态机怪癖）。
// 架构（参考 hobot_imu_sensor 的 event 读取方式）：
//   poll + 独立读取线程 + deque 队列 + 条件变量，read_data 有界阻塞出队。

class event_device : public imu_base {
  public:
    event_device() = default;
    ~event_device() override;            // 兜底：若未 deinit 则调用 deinit()

    int init() override;
    int read_data(ImuData_T *imu_data) override;
    int deinit() override;
    int set_path(std::vector<DeviceInfo_T> &dev_info) override;

  private:
    struct EventFrame {          // 队列元素（比 ImuData_T 轻，出队时再换算物理量）
        int16_t raw[6];          // accel xyz + gyro xyz
        uint64_t sync_time;      // 采样时刻(ns)，来自事件 ev.time
    };

    void pollThread();
    int detectEventDevice(std::string &dev_path);   // 扫 /dev/input/eventX + EVIOCGNAME
    int readIioScales();   // 扫 /sys/bus/iio/devices 本族设备读 scale
    void pushFrame(const EventFrame &frame);
    // poll 连续超时超过阈值 → 疑似驱动 event 通道卡死，尝试自愈：
    //   重开 event fd（处理 evdev client 侧异常）；重试多次仍无数据则报
    //   ERROR 并停止尝试。event 通道卡死期间 IIO sysfs raw 通常仍活
    //   （S100 实测），调用方（ImuManager）可查询本接口决定是否降级
    //   到 IIO 读取兜底。
    // 【禁止自动 rmmod/modprobe 驱动模块】——S100 实测 2026-10-10：
    // 卸载 inv_icm42600 会触发内核崩溃导致整板重启，用户态自愈不
    // 承担系统级风险；确需复位驱动时由人工重启板子。
    void recoverEventChannel();

  public:
    // event 通道是否已判死（reopen 重试全部失败）：true 表示此后
    // read_data 只会超时返回，调用方可据此降级到 IIO 读取
    bool isStalled() const { return recover_attempts_ > kMaxRecoverAttempts; }

    // ---- 传感器族（init 时由 EVIOCGNAME 匹配确定，如 "icm42688"）----
    std::string imu_dev_;

    // ---- 物理量换算 scale（init 时 readIioScales 从 IIO sysfs 读取，
    //      读不到保持满量程兜底值）----
    double accel_scale_ = 0.004788403;  // m/s²/LSB（±16g 兜底值）
    double gyro_scale_ = 0.001065264;   // rad/s/LSB（±2000dps 兜底值）

    // ---- 运行时 ----
    std::vector<DeviceInfo_T> dev_info_;
    std::string dev_path_;             // /dev/input/eventX
    int event_fd_ = -1;
    // 退出信号 fd（eventfd，pollThread 的 epoll 监听；deinit 写入触发退出，
    // 对应 sample_imu_input.c 里 signalfd 的角色——类内不能用进程信号）
    int exit_fd_ = -1;
    // pollThread 内创建的 epoll fd（成员化供 recoverEventChannel 换 fd 时重挂）
    int ep_fd_ = -1;
    std::atomic_bool is_running_{false};
    bool initialized_ = false;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::shared_ptr<std::thread> read_thread_;
    std::deque<EventFrame> que_;

    // 队列上限：64 帧 ≈ 320ms @200Hz；溢出丢最旧并计数
    static constexpr size_t kMaxQueueSize = 64;
    // read_data 有界阻塞上限：保证节点线程退出路径不死锁
    // （节点析构顺序是 join 在 release_sensor 之前）
    static constexpr int kReadTimeoutMs = 100;
    // poll 超时：决定 deinit join 最长等待
    static constexpr int kPollTimeoutMs = 100;

    uint64_t dropped_frames_ = 0;      // 溢出丢弃计数（节流打日志）
    uint64_t parse_errors_ = 0;        // 残帧/短读计数

    // poll 连续超时判定驱动卡死的阈值（10s = 100 次 × kPollTimeoutMs；
    // 正常无数据场景只有传感器断电，10s 足够区分）
    static constexpr int kStallTimeoutCount = 100;
    // 驱动卡死后自愈重试的最大次数（超过则放弃，read_data 持续返回超时）
    static constexpr int kMaxRecoverAttempts = 3;
    int stall_timeout_count_ = 0;      // 连续超时计数
    int recover_attempts_ = 0;         // 已自愈重试次数
};

}  // namespace imu_sensor
#endif  // HOBOT_EVENT_DEVICE_HPP_
