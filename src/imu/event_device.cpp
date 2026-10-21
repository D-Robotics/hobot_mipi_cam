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

// icm42688 input event 读取实现（协议与驱动怪癖见 event_device.hpp 头注释）。
// 架构参考 hobot_imu_sensor 的 event 方式（poll + 独立线程 + deque +
// condition_variable），并修正其已知缺陷：
//   1. 设备自动探测（EVIOCGNAME 匹配），不写死 eventX
//   2. init 失败返回错误码，绝不 exit()（会杀掉整个相机节点进程）
//   3. readn 短读/EOF 时复位帧解析状态（不复位会导致后续帧错位）
//   4. read_data 有界阻塞 wait_for，保证节点线程退出路径不死锁
//   5. 队列加上限 + 溢出丢最旧

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <poll.h>
#include <stdint.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <string>
#include <utility>

#include <linux/input.h>
#include <linux/input-event-codes.h>

#include "event_device.hpp"

namespace imu_sensor
{

static const char *kInputDevDir = "/dev/input";

// ========== 事件协议的传感器族配置（新增传感器只需在此加一行） ==========
// 同款 input event 协议（EV_MSC/MSC_RAW 9 事件帧，见 hpp 头注释）的
// InvenSense ICM 系列；imu_dev 为芯片名（用于 IIO 设备名匹配：
// "<imu_dev>-gyro" / "<imu_dev>-accel"）。

static const char *kImuFamilies[] = {
    "icm42688",
    // "icm42708",  // 后续扩展：驱动就绪后放开即可
};

// 按设备名族匹配（kImuFamilies），返回命中的 imu_dev 名。
// input 事件设备名为 "<imu_dev>-sensor"（如 "icm42688-sensor"，S100 实测），
// 用精确后缀匹配而非子串，避免设备名仅含族名片段时误命中。
static std::string matchImuFamily(const std::string &name) {
  for (const char *f : kImuFamilies) {
    if (name == std::string(f) + "-sensor") {
      return f;
    }
  }
  return "";
}

// ========== 物理量换算（IIO scale，与 icm42688.cpp 统一口径） ==========
// event 帧 raw 与 IIO sysfs raw 同源（同一驱动、同一 mount-matrix、同一
// 满量程配置，S100 实测两条通路重力方向一致），换算直接用 IIO 驱动导出的
// scale 相乘（与 IIO buffer 通路 sample.*_x * *_scale_ 同款）：
//   in_accel_scale   m/s²/LSB
//   in_anglvel_scale rad/s/LSB（单位已含在 scale 里，无需再乘 π/180）
// scale 在 init 时从 /sys/bus/iio/devices 本族设备读取（readIioScales）；
// 读不到用 icm42688 满量程兜底值（与 IIO scale 数值一致）。

static constexpr double kDefaultAccelScale = 0.004788403;  // m/s²/LSB（±16g）
static constexpr double kDefaultGyroScale = 0.001065264;   // rad/s/LSB（±2000dps）

static bool readDoubleFromFile(const std::string &path, double &out) {
  FILE *fp = fopen(path.c_str(), "r");
  if (!fp) {
    return false;
  }
  float f = 0.0f;
  bool ok = fscanf(fp, "%f", &f) == 1;
  fclose(fp);
  if (ok) {
    out = static_cast<double>(f);
  }
  return ok;
}

// 从 /sys/bus/iio/devices 扫描本传感器族的 IIO 设备读 scale：
//   "<imu_dev>-accel" / "<imu_dev>"（单设备布局）→ in_accel_scale
//   "<imu_dev>-gyro"  / "<imu_dev>"（单设备布局）→ in_anglvel_scale
// 成功读取则覆盖兜底值；一条都读不到只告警不失败（换算仍可用兜底值）。
int event_device::readIioScales() {
  const char *kIioDir = "/sys/bus/iio/devices";
  DIR *dp = opendir(kIioDir);
  if (!dp) {
    RCLCPP_WARN(rclcpp::get_logger("event_device"),
                "opendir %s: %s, use default scales",
                kIioDir, strerror(errno));
    return -1;
  }
  bool got_accel = false;
  bool got_gyro = false;
  struct dirent *ent;
  while ((ent = readdir(dp)) != nullptr) {
    if (strstr(ent->d_name, "iio:device") == nullptr) {
      continue;
    }
    const std::string base = std::string(kIioDir) + "/" + ent->d_name;
    char name[64] = {0};
    FILE *fp = fopen((base + "/name").c_str(), "r");
    if (!fp) {
      continue;
    }
    if (fgets(name, sizeof(name), fp)) {
      name[strcspn(name, "\n")] = '\0';
    }
    fclose(fp);
    const std::string dev_name(name);
    if (dev_name.rfind(imu_dev_, 0) != 0) {
      continue;    // 非本族 IIO 设备
    }
    if (!got_accel &&
        (dev_name == imu_dev_ || dev_name == imu_dev_ + "-accel")) {
      got_accel = readDoubleFromFile(base + "/in_accel_scale", accel_scale_);
    }
    if (!got_gyro &&
        (dev_name == imu_dev_ || dev_name == imu_dev_ + "-gyro")) {
      got_gyro = readDoubleFromFile(base + "/in_anglvel_scale", gyro_scale_);
    }
  }
  closedir(dp);
  if (!got_accel) {
    accel_scale_ = kDefaultAccelScale;
  }
  if (!got_gyro) {
    gyro_scale_ = kDefaultGyroScale;
  }
  RCLCPP_INFO(rclcpp::get_logger("event_device"),
              "scales: accel=%.9f m/s²/LSB, gyro=%.9f rad/s/LSB "
              "(iio: accel=%s, gyro=%s)",
              accel_scale_, gyro_scale_,
              got_accel ? "read" : "default", got_gyro ? "read" : "default");
  return 0;
}

// Stevens 式 EINTR-safe 完整读
static ssize_t readn(int fd, void *vptr, size_t n) {
  size_t nleft = n;
  ssize_t nread;
  char *ptr = static_cast<char*>(vptr);
  while (nleft > 0) {
    if ((nread = read(fd, ptr, nleft)) < 0) {
      if (errno == EINTR)
        nread = 0;    /* and call read() again */
      else
        return(-1);
    } else if (nread == 0)
      break;          /* EOF */
    nleft -= nread;
    ptr   += nread;
  }
  return(n - nleft);  /* return >= 0 */
}

// ========== 设备探测 ==========

static int eventDevIndex(const std::string &d_name) {
  // d_name 形如 "event12"，提取数字用于数值排序（字符串序会错排 event10 < event2）
  if (d_name.rfind("event", 0) != 0) return -1;
  int idx = 0;
  const char *p = d_name.c_str() + 5;
  if (!isdigit(static_cast<unsigned char>(*p))) return -1;
  while (*p) {
    if (!isdigit(static_cast<unsigned char>(*p))) return -1;
    idx = idx * 10 + (*p - '0');
    p++;
  }
  return idx;
}

static bool getEventDeviceName(const std::string &dev_path, std::string &name_out) {
  int fd = open(dev_path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    return false;
  }
  char name[128] = {0};
  bool ok = false;
  if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0 && name[0] != '\0') {
    name_out = name;
    ok = true;
  }
  close(fd);
  return ok;
}

int event_device::detectEventDevice(std::string &dev_path) {
  // 1) 优先 ImuManager 传入的显式 /dev/input/eventX 路径（须为已知 IMU 设备）
  for (const auto &info : dev_info_) {
    if (info.path.rfind("/dev/input/event", 0) == 0) {
      std::string name;
      if (getEventDeviceName(info.path, name)) {
        std::string fam = matchImuFamily(name);
        if (!fam.empty()) {
          dev_path = info.path;
          imu_dev_ = fam;
          RCLCPP_INFO(rclcpp::get_logger("event_device"),
                      "use input event device %s (%s)", info.path.c_str(),
                      name.c_str());
          return 0;
        }
      }
      RCLCPP_WARN(rclcpp::get_logger("event_device"),
                  "specified device %s is not a supported imu (name: %s), "
                  "try auto detect",
                  info.path.c_str(),
                  name.empty() ? "<unreadable>" : name.c_str());
    }
  }

  // 2) 扫描 /dev/input/eventX，EVIOCGNAME 匹配设备族
  DIR *dp = opendir(kInputDevDir);
  if (!dp) {
    RCLCPP_WARN(rclcpp::get_logger("event_device"), "opendir %s: %s",
                kInputDevDir, strerror(errno));
    return -1;
  }
  std::vector<std::pair<int, std::string>> candidates;  // (数字序, 路径)
  struct dirent *ent;
  while ((ent = readdir(dp)) != nullptr) {
    int idx = eventDevIndex(ent->d_name);
    if (idx < 0) {
      continue;
    }
    std::string p = std::string(kInputDevDir) + "/" + ent->d_name;
    std::string name;
    if (getEventDeviceName(p, name) && !matchImuFamily(name).empty()) {
      candidates.emplace_back(idx, p);
      RCLCPP_INFO(rclcpp::get_logger("event_device"),
                  "found input event device %s (%s)", p.c_str(), name.c_str());
    }
  }
  closedir(dp);

  if (candidates.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("event_device"),
                "no /dev/input/eventX device matched supported imu");
    return -1;
  }
  std::sort(candidates.begin(), candidates.end());
  if (candidates.size() > 1) {
    RCLCPP_WARN(rclcpp::get_logger("event_device"),
                "%zu input event devices matched, use %s",
                candidates.size(), candidates.front().second.c_str());
  }
  dev_path = candidates.front().second;
  std::string name;
  if (getEventDeviceName(dev_path, name)) {
    imu_dev_ = matchImuFamily(name);
  }
  return 0;
}

// ========== imu_base 接口实现 ==========

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <linux/input.h>
#include <stdint.h>
#include <time.h>
#include <inttypes.h>
#include <string.h>
#include <sys/ioctl.h>
static int setup_signalfd(void) {
  sigset_t mask;
  sigemptyset(&mask);
  sigaddset(&mask, SIGINT);
  sigaddset(&mask, SIGTERM);
  // 先阻塞信号：signalfd 拦截后默认处置不再生效
  if (sigprocmask(SIG_BLOCK, &mask, NULL) != 0) {
      perror("sigprocmask");
      return -1;
  }
  return signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);
}

int event_device::init() {
  if (initialized_) {
    return 0;
  }
  if (detectEventDevice(dev_path_) != 0) {
    return -1;
  }
  readIioScales();    // IIO scale 获取（失败不阻塞 init，用兜底值换算）
  event_fd_ = open(dev_path_.c_str(), O_RDONLY);
  if (event_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("event_device"), "open %s: %s",
                 dev_path_.c_str(), strerror(errno));
    return -1;
  }
  // 退出信号 eventfd（pollThread 的 epoll 监听；deinit 写入触发退出）
  //exit_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  exit_fd_ = setup_signalfd();
  if (exit_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("event_device"), "eventfd: %s",
                 strerror(errno));
    close(event_fd_);
    event_fd_ = -1;
    return -1;
  }
  is_running_ = true;
  read_thread_ = std::make_shared<std::thread>([this] { pollThread(); });
  initialized_ = true;
  RCLCPP_INFO(rclcpp::get_logger("event_device"),
              "event device %s opened", dev_path_.c_str());
  return 0;
}

int event_device::set_path(std::vector<DeviceInfo_T> &dev_info) {
  dev_info_ = dev_info;    // 仅使用 path 以 /dev/input/event 开头的条目
  return 0;
}

// ========== 读线程（生产者，epoll 事件驱动，与 sample_imu_input.c 同款） ==========
// epoll 同时监听两个 fd：
//   event_fd_（/dev/input/eventX，EPOLLIN = 驱动上报事件）
//   exit_fd_（eventfd，deinit 写入 = 退出信号；对应 sample 里 signalfd 的角色，
//            类内不能用进程信号，eventfd 语义等价：退出事件也是普通 fd 事件）
// 无数据时 epoll_wait 阻塞（带超时上限以驱动卡死检测），零轮询 CPU。

#if 1
void event_device::pollThread() {
  int index = 0, count = 0, last_count = -1;
  int16_t raw[6] = {0};
  uint64_t frame_time_us = 0;    // 帧采样时刻（帧内 ev.time 同值）
  struct input_event event;
  // epoll：监听 input 设备 fd + 退出 eventfd
  ep_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (ep_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("event_device"), "epoll_create1: %s",
                 strerror(errno));
    return;
  }
  struct epoll_event epev;
  memset(&epev, 0, sizeof(epev));
  epev.events = EPOLLIN;
  epev.data.fd = event_fd_;
  bool ep_ok = epoll_ctl(ep_fd_, EPOLL_CTL_ADD, event_fd_, &epev) == 0;
  epev.data.fd = exit_fd_;
  if (ep_ok) {
    ep_ok = epoll_ctl(ep_fd_, EPOLL_CTL_ADD, exit_fd_, &epev) == 0;
  }
  if (!ep_ok) {
    RCLCPP_ERROR(rclcpp::get_logger("event_device"), "epoll_ctl: %s",
                 strerror(errno));
    close(ep_fd_);
    ep_fd_ = -1;
    return;
  }

  while (is_running_) {
    struct epoll_event events[2];
    // -1 无限阻塞最优雅，但保留 kPollTimeoutMs 上限：周期性回查
    // is_running_（退出兜底）+ 驱动卡死检测（stall_timeout_count_）
    int nready = epoll_wait(ep_fd_, events, 2, kPollTimeoutMs);
    if (nready < 0) {
      if (errno == EINTR) {
        continue;    // 被无关信号打断：回查 is_running_
      }
      RCLCPP_ERROR(rclcpp::get_logger("event_device"), "epoll_wait: %s",
                   strerror(errno));
      continue;
    }
    if (nready == 0) {
      // 连续超时超阈值 → inv_icm42688 的 input 事件上报路径卡死
      // （evdev 无事件但 IIO sysfs raw 仍活，S100 实测 2026-10-10）
      if (++stall_timeout_count_ >= kStallTimeoutCount) {
        stall_timeout_count_ = 0;
        //recoverEventChannel();
      }
      continue;    // 超时 → 回查 is_running_（退出延迟 ≤ kPollTimeoutMs）
    }
    for (int i = 0; i < nready; ++i) {
      if (events[i].data.fd == exit_fd_) {
        // deinit 的退出信号：消费掉并退出（read_data 由 cv 唤醒）
        uint64_t one;
        //read(exit_fd_, &one, sizeof(one));
        is_running_ = false;
        break;
      }
      if (!(events[i].events & EPOLLIN)) {
        continue;
      }
      stall_timeout_count_ = 0;
      recover_attempts_ = 0;    // 拿到数据 = 自愈成功，重置重试额度
      ssize_t n = read(event_fd_, &event, sizeof(event));
      if (n != static_cast<ssize_t>(sizeof(event))) {
        // 短读/EOF → 丢弃半帧并复位解析状态
        if (index != 0) {
          parse_errors_++;
          index = 0;
        }
        RCLCPP_WARN(rclcpp::get_logger("event_device"),
                    "short read %zd/%zu, reset frame state",
                    n, sizeof(event));
        continue;
      }
      if (event.type == EV_SYN) {
        if (index != 0) {    // 残帧丢弃
          parse_errors_++;
          index = 0;
        }
        continue;
      }
      if (event.type != EV_MSC) {
        continue;    // 只处理 EV_MSC/MSC_RAW
      }
      // 采样时刻在 ev.time（帧内同值，µs 精度，≈DATA_THS_INT 中断时刻）
      frame_time_us = static_cast<uint64_t>(event.input_event_sec) * 1000000ull +
                      static_cast<uint64_t>(event.input_event_usec);
      // 位置协议：0-2 accel、3-5 gyro、6/7 占位、8 帧计数
      switch (index++) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
          raw[index - 1] = static_cast<int16_t>(event.value);
          break;
        case 6:
        case 7:
          break;    // 占位（恒 0）
        case 8:
          count = static_cast<int>(event.value);
          break;
        default:
          break;    // 超出 9 个事件：异常，等 EV_SYN 复位
      }
      if (index == 9) {
        index = 0;
        if (last_count >= 0 && count != last_count + 1) {
          RCLCPP_WARN(rclcpp::get_logger("event_device"),
                      "imu frame gap: count %d -> %d (lost %d)",
                      last_count, count, count - last_count - 1);
        }
        last_count = count;
        EventFrame f;
        memcpy(f.raw, raw, sizeof(f.raw));
        f.sync_time = frame_time_us * 1000ull;    // µs → ns
        pushFrame(f);
      }
    }
  }
  close(ep_fd_);
  ep_fd_ = -1;
}
#else

#define BMI088_MSC_DATA 0x04                // MSC code used by the driver (must match kernel)
#define MSC_DATA_COUNT 9                    // MSC fields per frame: 6 axes + timestamp (2 words) + IRQ count
#define ACC_X_IDX 0                         // accelerometer X index
#define ACC_Y_IDX 1                         // accelerometer Y index
#define ACC_Z_IDX 2                         // accelerometer Z index
#define GYRO_X_IDX 3                        // gyroscope X index
#define GYRO_Y_IDX 4                        // gyroscope Z index
#define GYRO_Z_IDX 5                        // gyroscope Z index
#define TS_HIGH_IDX 6                       // timestamp high 32 bits index
#define TS_LOW_IDX 7                        // timestamp low 32 bits index
#define IRQ_COUNT 8                         // IRQ count index


void event_device::pollThread() {
  int index = 0, count = 0, last_count = -1;
  int16_t raw[6] = {0};
  uint64_t frame_time_us = 0;    // 帧采样时刻（帧内 ev.time 同值）
  struct input_event event;
  // epoll：监听 input 设备 fd + 退出 eventfd
  ep_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (ep_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("event_device"), "epoll_create1: %s",
                 strerror(errno));
    return;
  }
  struct epoll_event epev;
  memset(&epev, 0, sizeof(epev));
  epev.events = EPOLLIN;
  epev.data.fd = event_fd_;
  bool ep_ok = epoll_ctl(ep_fd_, EPOLL_CTL_ADD, event_fd_, &epev) == 0;
  epev.data.fd = exit_fd_;
  if (ep_ok) {
    ep_ok = epoll_ctl(ep_fd_, EPOLL_CTL_ADD, exit_fd_, &epev) == 0;
  }
  if (!ep_ok) {
    RCLCPP_ERROR(rclcpp::get_logger("event_device"), "epoll_ctl: %s",
                 strerror(errno));
    close(ep_fd_);
    ep_fd_ = -1;
    return;
  }


  int32_t msc_data[MSC_DATA_COUNT] = {0};
  int msc_index = 0;  // Index of the next MSC field in the current frame
  struct input_event ev;
  // Parsed frame data
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  uint64_t raw_timestamp = 0;  // Assembled 64-bit hardware timestamp (ns)
  int irq_count = 0;

  // Packet-loss detection state
  int64_t diff, min_diff = INT64_MAX, max_diff = INT64_MIN;
  uint64_t timestamp_ns_last = 0, lost_count = 0;
  int frames = 0, event_count = 0;
  while (is_running_) {
    struct epoll_event events[2];
    int nready = epoll_wait(ep_fd_, events, 2, -1);
    if (nready < 0) {
        if (errno == EINTR) {
            continue;    // 被无关信号打断：回查 g_running
        }
        perror("epoll_wait");
        break;    // epoll 真错误：跳出循环走清理
    }
    for (int i = 0; i < nready; ++i) {
        if (events[i].data.fd == exit_fd_) {
            // 读取并消费 signalfd 信号（SIGINT/SIGTERM → 置退出标志）
            struct signalfd_siginfo si;
            #if 0
            while (read(exit_fd_, &si, sizeof(si)) == (ssize_t)sizeof(si)) {
                if (si.ssi_signo == SIGINT || si.ssi_signo == SIGTERM) {
                  is_running_ = 0;
                }
            }
            #endif
            is_running_ = 0;
            continue;
        }
        if (events[i].events & EPOLLIN) {
            ssize_t ret = read(event_fd_, &ev, sizeof(ev));
            if (ret != sizeof(ev)) {
                if (errno == EINTR) {
                    continue;
                }
                perror("read failed");
                is_running_ = 0;    // fd 坏/EOF：跳出外层循环走清理
                break;
            }
            ++event_count;

            switch (ev.type) {
            // Parse only EV_MSC events with BMI088_MSC_DATA
            case EV_MSC:
                if (ev.code == BMI088_MSC_DATA) {
                    if (msc_index < MSC_DATA_COUNT) {
                        msc_data[msc_index++] = ev.value;  // Store fields in order
                    } else {
                        // Overflow: too many MSC fields in this frame, reset
                        printf("[WARN] MSC data overflow, reset index\n");
                        msc_index = 0;
                    }
                }
                break;

            case EV_SYN:
                if (ev.code == SYN_REPORT) {
                    // End of frame: parse the cached MSC fields
                    if (msc_index == MSC_DATA_COUNT) {
                        // 1. Parse 6-axis raw data (int32 ev.value -> int16 sensor LSB)
                        ax = (int16_t)msc_data[ACC_X_IDX];
                        ay = (int16_t)msc_data[ACC_Y_IDX];
                        az = (int16_t)msc_data[ACC_Z_IDX];
                        gx = (int16_t)msc_data[GYRO_X_IDX];
                        gy = (int16_t)msc_data[GYRO_Y_IDX];
                        gz = (int16_t)msc_data[GYRO_Z_IDX];

                        // 2. Assemble 64-bit timestamp: (high << 32) | low
                        raw_timestamp = ((uint64_t)msc_data[TS_HIGH_IDX] << 32) | (uint32_t)msc_data[TS_LOW_IDX];

                        irq_count = (int16_t)msc_data[IRQ_COUNT];

                        // 3. Print one assembled frame (raw LSB + hardware timestamp)
                        printf("ACC(%d,%d,%d) | GYRO(%d,%d,%d) | TS:%" PRIu64 " ns | Lost: %" PRIu64 " | EventCount: %d | up_count: %d | lower_count: - \n",
                               ax, ay, az, gx, gy, gz, raw_timestamp, lost_count, event_count, irq_count);
                        frames++;

                        // 4. Detect packet loss from timestamp gaps
                        if (timestamp_ns_last != 0 && raw_timestamp != 0) {
                            diff = (int64_t)(raw_timestamp - timestamp_ns_last);
                            if (diff > 3500000) {  // Threshold: 3.5 ms
                                lost_count++;
                                printf("[ERROR] IMU data lost: last ts %.6fs, current ts %.6fs, diff %.6fs\n",
                                       timestamp_ns_last * 1e-9, raw_timestamp * 1e-9, diff * 1e-9);
                            }
                            if (diff < min_diff && diff > 0) min_diff = diff;
                            if (diff > max_diff) max_diff = diff;
                        }
                        timestamp_ns_last = raw_timestamp;
                    } else {
                        // Incomplete frame: drop and wait for next SYN_REPORT
                        printf("[WARN] MSC data incomplete (received %d/%d), skip\n", msc_index, MSC_DATA_COUNT);
                    }

                    // Reset cache for the next frame
                    msc_index = 0;
                    memset(msc_data, 0, sizeof(msc_data));
                    event_count = 0;
                }
                break;

            // EV_ABS is not used by this driver
            default:
                // Ignore other event types
                break;
          }
        }    // end if EPOLLIN
    }    // end for nready
  }
  close(ep_fd_);
  ep_fd_ = -1;
}
#endif

// 驱动 event 通道卡死时的自愈（只做安全动作）：
//   重开 event fd 处理 evdev client 侧异常。
// 若卡死在驱动侧（evdev 无事件但 IIO sysfs raw 仍活），reopen 无法恢复，
// 此时仅报 ERROR 停止重试，由 read_data 持续超时返回、上层感知。
// 【禁止自动 rmmod/modprobe 驱动模块】——S100 实测 2026-10-10：卸载
// inv_icm42600 会触发内核崩溃导致整板重启（17:11/17:15 两次复现），
// 用户态自愈不允许承担系统级风险；确需复位驱动时由人工执行。
void event_device::recoverEventChannel() {
  if (recover_attempts_ >= kMaxRecoverAttempts) {
    if (recover_attempts_ == kMaxRecoverAttempts) {
      RCLCPP_ERROR(rclcpp::get_logger("event_device"),
                   "event channel stalled: no events for %dms, "
                   "%d reopen attempts failed. Sensor may be powered off "
                   "or inv_icm42688 input path stuck (IIO sysfs raw may "
                   "still work; manual recovery: reboot the board). "
                   "read_data will keep timing out.",
                   kStallTimeoutCount * kPollTimeoutMs,
                   kMaxRecoverAttempts);
      recover_attempts_++;    // 只报一次
    }
    return;
  }
  recover_attempts_++;
  RCLCPP_WARN(rclcpp::get_logger("event_device"),
              "no events for %dms, reopen %s (attempt %d/%d)",
              kStallTimeoutCount * kPollTimeoutMs, dev_path_.c_str(),
              recover_attempts_, kMaxRecoverAttempts);
  int new_fd = open(dev_path_.c_str(), O_RDONLY | O_NONBLOCK);
  if (new_fd < 0) {
    RCLCPP_WARN(rclcpp::get_logger("event_device"), "reopen %s: %s",
                dev_path_.c_str(), strerror(errno));
    return;
  }
  // 旧 fd 先从 epoll 摘除再 close（close 一个仍在 epoll 中的 fd 会留下
  // 悬挂的 epitem，不同内核处理不一）；新 fd 挂回 epoll 继续监听
  if (ep_fd_ >= 0 && event_fd_ >= 0) {
    epoll_ctl(ep_fd_, EPOLL_CTL_DEL, event_fd_, nullptr);
  }
  if (event_fd_ >= 0) {
    close(event_fd_);
  }
  event_fd_ = new_fd;
  if (ep_fd_ >= 0) {
    struct epoll_event epev;
    memset(&epev, 0, sizeof(epev));
    epev.events = EPOLLIN;
    epev.data.fd = event_fd_;
    epoll_ctl(ep_fd_, EPOLL_CTL_ADD, event_fd_, &epev);
  }
}

void event_device::pushFrame(const EventFrame &frame) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (que_.size() >= kMaxQueueSize) {
    que_.pop_front();    // 溢出丢最旧（消费端慢于生产端时保实时性）
    dropped_frames_++;
    if (dropped_frames_ % 100 == 1) {
      RCLCPP_WARN(rclcpp::get_logger("event_device"),
                  "queue full(%zu), dropped oldest (total %llu)",
                  kMaxQueueSize, static_cast<unsigned long long>(dropped_frames_));
    }
  }
  que_.push_back(frame);
  cv_.notify_one();
}

// ========== read_data（消费者，有界阻塞） ==========

int event_device::read_data(ImuData_T *imu_data) {
  if (!initialized_ || imu_data == nullptr) {
    return -1;
  }
  std::unique_lock<std::mutex> lk(mtx_);
  // 有界阻塞（不是无界 wait）：节点析构顺序是 join 在 release_sensor 之前，
  // 若无界等待且设备停数据，节点线程将永远无法回查 is_imu_running_ → join 死锁
  bool got = cv_.wait_for(lk, std::chrono::milliseconds(kReadTimeoutMs),
                          [&] { return !que_.empty() || !is_running_; });
  if (!is_running_ || !got || que_.empty()) {
    return -1;    // deinit 唤醒 / 超时 → 错误码
  }
  const EventFrame f = que_.front();
  que_.pop_front();
  lk.unlock();

  // 物理量换算：raw × IIO scale（accel → m/s²、gyro → rad/s，scale 已含
  // 单位，无需 π/180），见文件头换算说明
  imu_data->ax = f.raw[0] * accel_scale_;
  imu_data->ay = f.raw[1] * accel_scale_;
  imu_data->az = f.raw[2] * accel_scale_;
  imu_data->gx = f.raw[3] * gyro_scale_;
  imu_data->gy = f.raw[4] * gyro_scale_;
  imu_data->gz = f.raw[5] * gyro_scale_;
  // icm42688 无磁力计
  imu_data->mx = 0.0f;
  imu_data->my = 0.0f;
  imu_data->mz = 0.0f;
  imu_data->timestamp = f.sync_time;
  imu_data->status = 0;
  return 0;
}

int event_device::deinit() {
  if (!initialized_) {
    return 0;    // 幂等
  }
  is_running_ = false;
  cv_.notify_all();    // 唤醒可能阻塞在 wait_for 的 read_data

  if (exit_fd_ >= 0) {
    // 写退出信号：pollThread 的 epoll_wait 立即返回（exit_fd 可读）→ 线程
    // 及时退出，不再依赖 kPollTimeoutMs 超时兜底
    uint64_t one = 1;
    write(exit_fd_, &one, sizeof(one));
  }

  if (read_thread_ && read_thread_->joinable()) {
    read_thread_->join();
    read_thread_ = nullptr;
  }
  // 必须关闭 fd：不关则每次 init→deinit→init 泄漏一个 evdev client
  // （内核为每个 open 分配独立事件缓冲，旧 client 继续堆事件无人读）；
  // close 走 evdev_release 正常断开驱动 client（Ctrl-C 后再启动的干净路径）
  if (event_fd_ >= 0) {
    close(event_fd_);
    event_fd_ = -1;
  }
  if (exit_fd_ >= 0) {
    close(exit_fd_);
    exit_fd_ = -1;
  }
  initialized_ = false;
  if (dropped_frames_ > 0 || parse_errors_ > 0) {
    RCLCPP_INFO(rclcpp::get_logger("event_device"),
                "deinit: dropped_frames=%llu parse_errors=%llu",
                static_cast<unsigned long long>(dropped_frames_),
                static_cast<unsigned long long>(parse_errors_));
  }
  return 0;
}

event_device::~event_device() {
  if (initialized_) {
    deinit();
  }
}

}  // namespace imu_sensor
