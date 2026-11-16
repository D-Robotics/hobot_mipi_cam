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

// 异常模拟器实现。模块地图与设计约束见 hobot_mipi_fault_injector.hpp 头注释。

#include "hobot_mipi_fault_injector.hpp"

#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace mipi_cam
{

// 字段名 <-> FaultType 映射表（对外契约，勿随意改动拼写：
// fault_inject 参数值与 /mipi_cam_fault_status 内容均以此为准）
static const struct {
  FaultInjector::FaultType type;
  const char *name;
} kFaultNames[] = {
  {FaultInjector::FaultType::IMU_STALLED,           "IMU_STALLED"},
  {FaultInjector::FaultType::IMU_TS_DRIFT,          "IMU_STALE_OR_FUTURE_TIMESTAMP"},
  {FaultInjector::FaultType::IMU_TS_NOT_INCREASING, "IMU_TIMESTAMP_NOT_INCREASING"},
  {FaultInjector::FaultType::IMAGE_STALLED,         "IMAGE_STALLED"},
  {FaultInjector::FaultType::IMAGE_TS_DRIFT,        "IMAGE_STALE_OR_FUTURE_TIMESTAMP"},
};

// 解析逗号分隔字段列表（段两侧容空白），非法项即失败（严格模式）
bool FaultInjector::parseFaultList(const std::string &fault_list,
                                   Config &cfg, std::string &err) {
  std::vector<std::string> items;
  size_t pos = 0;
  while (pos <= fault_list.size()) {
    const size_t comma = fault_list.find(',', pos);
    std::string item = fault_list.substr(
        pos, (comma == std::string::npos ? fault_list.size() : comma) - pos);
    // trim 空白
    const size_t b = item.find_first_not_of(" \t\r\n");
    if (b != std::string::npos) {
      const size_t e = item.find_last_not_of(" \t\r\n");
      items.push_back(item.substr(b, e - b + 1));
    }
    if (comma == std::string::npos) break;
    pos = comma + 1;
  }

  for (const auto &item : items) {
    bool hit = false;
    for (const auto &e : kFaultNames) {
      if (item != e.name) continue;
      switch (e.type) {
        case FaultType::IMU_STALLED:           cfg.imu_stalled = true; break;
        case FaultType::IMU_TS_DRIFT:          cfg.imu_ts_drift = true; break;
        case FaultType::IMU_TS_NOT_INCREASING: cfg.imu_ts_not_increasing = true; break;
        case FaultType::IMAGE_STALLED:         cfg.image_stalled = true; break;
        case FaultType::IMAGE_TS_DRIFT:        cfg.image_ts_drift = true; break;
      }
      hit = true;
      break;
    }
    if (!hit) {
      err = "unknown fault field '" + item +
            "', valid: IMU_STALLED, IMU_STALE_OR_FUTURE_TIMESTAMP, "
            "IMU_TIMESTAMP_NOT_INCREASING, IMAGE_STALLED, "
            "IMAGE_STALE_OR_FUTURE_TIMESTAMP";
      return false;
    }
  }
  return true;
}

FaultInjector::FaultInjector(const Config &cfg) : cfg_(cfg) {
  enabled_ = cfg_.imu_stalled || cfg_.imu_ts_drift || cfg_.imu_ts_not_increasing ||
             cfg_.image_stalled || cfg_.image_ts_drift;
  last_stall_log_tp_ = std::chrono::steady_clock::time_point::min();
}

int64_t FaultInjector::steadyNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

void FaultInjector::anchorImageStream() {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!image_anchored_) {
    image_anchored_ = true;
    image_t0_ns_ = steadyNowNs();
  }
}

void FaultInjector::anchorImuStream() {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!imu_anchored_) {
    imu_anchored_ = true;
    imu_t0_ns_ = steadyNowNs();
  }
}

bool FaultInjector::stallWindow(int64_t elapsed_ns, double period_s, int duration_ms) {
  const int64_t duration_ns = static_cast<int64_t>(duration_ms) * 1000000LL;
  if (duration_ns <= 0) {
    return true;  // duration=0 语义：锚定后永久停顿
  }
  const int64_t period_ns = static_cast<int64_t>(period_s * 1e9);
  if (period_ns <= 0) {
    // 单次模式：T0 起停顿 duration，之后永久正常
    return elapsed_ns < duration_ns;
  }
  // 周期模式：先正常 P-D 再停顿 D；duration >= period 时相位条件恒真（持续停顿）
  const int64_t phase = elapsed_ns % period_ns;
  return phase >= period_ns - duration_ns;
}

bool FaultInjector::imageStallActive() {
  std::lock_guard<std::mutex> lk(mtx_);
  return imageStallActiveLocked();
}

bool FaultInjector::imuStallActive() {
  std::lock_guard<std::mutex> lk(mtx_);
  return imuStallActiveLocked();
}

bool FaultInjector::imageStallActiveLocked() {
  if (!cfg_.image_stalled || !image_anchored_) {
    return false;  // 未锚定（尚未发过首帧）谈不上停顿
  }
  const bool active = stallWindow(steadyNowNs() - image_t0_ns_,
                                  cfg_.image_stall_period_s,
                                  cfg_.image_stall_duration_ms);
  if (active) {
    stallLogThrottled("IMAGE_STALLED", "dropping frame");
  }
  return active;
}

bool FaultInjector::imuStallActiveLocked() {
  if (!cfg_.imu_stalled || !imu_anchored_) {
    return false;
  }
  const bool active = stallWindow(steadyNowNs() - imu_t0_ns_,
                                  cfg_.imu_stall_period_s,
                                  cfg_.imu_stall_duration_ms);
  if (active) {
    stallLogThrottled("IMU_STALLED", "dropping imu sample");
  }
  return active;
}

void FaultInjector::stallLogThrottled(const char *fault, const char *action) {
  const auto now = std::chrono::steady_clock::now();
  if (now - last_stall_log_tp_ < std::chrono::seconds(1)) {
    return;
  }
  last_stall_log_tp_ = now;
  RCLCPP_WARN(rclcpp::get_logger("fault_inject"),
              "[fault-inject] %s active, %s", fault, action);
}

int64_t FaultInjector::transformImageTs(int64_t raw_ts_ns) {
  // 只读 cfg_，无需持锁
  if (cfg_.image_ts_drift) {
    return raw_ts_ns + cfg_.ts_drift_ns;
  }
  return raw_ts_ns;
}

int64_t FaultInjector::transformImuTs(int64_t raw_ts_ns) {
  std::lock_guard<std::mutex> lk(mtx_);
  int64_t ts = raw_ts_ns;
  // 流水线固定顺序：drift 先施加，重复替换最后生效
  // （重复的是"上一条最终发出的时间戳"，天然含 drift 偏移）
  if (cfg_.imu_ts_drift) {
    ts += cfg_.ts_drift_ns;
  }
  if (cfg_.imu_ts_not_increasing) {
    if (imu_has_last_emitted_ts_) {
      // 重复上一条：每条都发同一时间戳（t1,t1,t1,...），持续触发 ts<=last 判定；
      // last 不更新，"上一条发出的"即 last 本身
      return imu_last_emitted_ts_ns_;
    }
  }
  imu_has_last_emitted_ts_ = true;
  imu_last_emitted_ts_ns_ = ts;
  return ts;
}

std::string FaultInjector::activeFaultsNow() {
  std::lock_guard<std::mutex> lk(mtx_);
  std::string s;
  const auto append = [&s](const char *name) {
    if (!s.empty()) s += ",";
    s += name;
  };
  // 输出顺序与字段表一致，便于测试框架比对（持锁调 locked 版，避免重入）
  if (cfg_.imu_stalled && imuStallActiveLocked())     append("IMU_STALLED");
  if (cfg_.imu_ts_drift)                              append("IMU_STALE_OR_FUTURE_TIMESTAMP");
  if (cfg_.imu_ts_not_increasing)                     append("IMU_TIMESTAMP_NOT_INCREASING");
  if (cfg_.image_stalled && imageStallActiveLocked()) append("IMAGE_STALLED");
  if (cfg_.image_ts_drift)                            append("IMAGE_STALE_OR_FUTURE_TIMESTAMP");
  return s;
}

}  // namespace mipi_cam
