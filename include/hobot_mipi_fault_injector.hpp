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

#ifndef HOBOT_MIPI_FAULT_INJECTOR_HPP_
#define HOBOT_MIPI_FAULT_INJECTOR_HPP_

#include <string>
#include <cstdint>
#include <chrono>
#include <mutex>

namespace mipi_cam
{

// ============================================================================
// 异常模拟器（fault injection）—— 用于下游节点的异常检测测试。
//
// 模块地图：
//   parseFaultList()      解析 fault_inject 启动参数（逗号分隔字段列表）→ Config
//   imageStallActive()    图像停顿窗口判定（所有图像话题共用一个状态机）
//   imuStallActive()      IMU 停顿窗口判定
//   transformImageTs()    图像时间戳流水线：raw → drift 偏移
//   transformImuTs()      IMU 时间戳流水线：raw → drift 偏移 → 重复替换
//   activeFaultsNow()     当前激活字段集合（供 /mipi_cam_fault_status 状态话题）
//
// 设计约束（与共识规格一致，改动前先确认）：
//   - 仅在 node 发布层注入：采集线程 / 帧队列 / hbn vflow / src/x5 / src/imu
//     零改动，底层照常产数据；停顿 = 发布层取数后丢弃，恢复正常后无积压。
//   - 停顿窗口内"继续取数、丢弃发布"，保证恢复瞬间发出的是新鲜数据，
//     不引入"陈旧积压导致时间戳跳变"这种需求之外的二次异常。
//   - 停顿时序以各数据流首条数据成功发布为 T0（anchorXxx() 在发布点调用，
//     幂等），相位为"先正常 P-D，再停顿 D"循环。
//   - 时间戳流水线固定顺序：正常ts → drift偏移 → 重复替换（NOT_INCREASING
//     最后生效，重复的是上一条最终发出的时间戳，天然含 drift 偏移）。
//   - 窗口判定用 steady_clock（CLOCK_MONOTONIC），不受系统时间跳变影响；
//     时间戳变换只做 ns 算术，不涉及时钟读取。
// ============================================================================

class FaultInjector
{
public:
  // 五种异常字段（对外契约：fault_inject 参数值与 /mipi_cam_fault_status 内容
  // 均使用 kFaultNames 中的字符串，勿随意改动拼写）
  enum class FaultType {
    IMU_STALLED,              // >50ms 无 IMU（下游判定阈值）
    IMU_TS_DRIFT,             // realtime 落后>200ms 或超前>100ms（IMU_STALE_OR_FUTURE_TIMESTAMP）
    IMU_TS_NOT_INCREASING,    // 时间戳重复或倒退（IMU_TIMESTAMP_NOT_INCREASING）
    IMAGE_STALLED,            // 超过图像间隔无图像
    IMAGE_TS_DRIFT,           // 图像版 STALE_OR_FUTURE_TIMESTAMP
  };

  struct Config {
    bool imu_stalled = false;
    bool imu_ts_drift = false;
    bool imu_ts_not_increasing = false;
    bool image_stalled = false;
    bool image_ts_drift = false;
    // 时间戳偏移量(ns)，由 fault_ts_drift_mode 换算：
    //   stale  → -250ms（阈值 200ms + 50ms 余量）
    //   future → +150ms（阈值 100ms + 50ms 余量）
    int64_t ts_drift_ns = 0;
    // 停顿时序参数（IMU/图像独立两套）
    double imu_stall_period_s = 10.0;     // 0 = 只注入一次
    int imu_stall_duration_ms = 1000;     // 0 = 永久静默
    double image_stall_period_s = 10.0;
    int image_stall_duration_ms = 1000;
  };

  // 解析逗号分隔的异常字段列表（容空白）。非法字段返回 false，err 带出
  // 非法项与合法值清单（调用方据此拒绝启动，严格模式：typo 早失败）。
  static bool parseFaultList(const std::string &fault_list, Config &cfg, std::string &err);

  explicit FaultInjector(const Config &cfg);

  bool anyEnabled() const { return enabled_; }

  // ---- 锚定（幂等）：各数据流首条数据"即将发布"时调用，作为停顿时序 T0 ----
  void anchorImageStream();
  void anchorImuStream();

  // ---- 停顿窗口判定（无副作用，可在任意线程/循环里查询）----
  // 返回 true = 本条数据丢弃（不发布）。未锚定（尚未发过首条）恒为 false。
  bool imageStallActive();
  bool imuStallActive();

  // ---- 时间戳流水线（仅对真正要发布的数据调用；发布层在 pubRateAllow
  //      之后、publish 之前调用，不干扰发布帧率控制逻辑）----
  int64_t transformImageTs(int64_t raw_ts_ns);
  int64_t transformImuTs(int64_t raw_ts_ns);

  // 当前激活字段集合（逗号分隔，空串=全部正常）。
  // 恒定类异常（drift/not_increasing）启用即激活；停顿类按当前窗口。
  std::string activeFaultsNow();

private:
  // 停顿窗口计算（elapsed 相对 T0）：
  //   duration<=0        → 永久停顿
  //   period<=0（单次）   → T0 起停顿 duration 后永久正常
  //   period>0（周期）    → 相位 [P-D, P) 内停顿；D>=P 时恒停顿（数学自动成立）
  static bool stallWindow(int64_t elapsed_ns, double period_s, int duration_ms);

  static int64_t steadyNowNs();

  // 已持锁版本（activeFaultsNow 一次持锁查询全部，避免重入死锁）
  bool imageStallActiveLocked();
  bool imuStallActiveLocked();

  // 停顿持续期的心跳日志（1s 节流）；窗口进入/退出沿由状态话题覆盖
  void stallLogThrottled(const char *fault, const char *action);

  // 并发保护：图像流状态被多个发布线程（dual 模式最多 6 个 publisher）+
  // IMU 线程并发查询；锁内无 IO 阻塞，仅纳秒级窗口计算
  std::mutex mtx_;

  Config cfg_;
  bool enabled_ = false;

  // 图像流状态（所有图像话题共用：T0 取第一个话题的首帧发布时刻）
  bool image_anchored_ = false;
  int64_t image_t0_ns_ = 0;
  // IMU 流状态
  bool imu_anchored_ = false;
  int64_t imu_t0_ns_ = 0;

  // 上一条最终发出的 IMU 时间戳（NOT_INCREASING 用；只在真正发布时更新）
  bool imu_has_last_emitted_ts_ = false;
  int64_t imu_last_emitted_ts_ns_ = 0;

  // 心跳日志节流基准
  std::chrono::steady_clock::time_point last_stall_log_tp_;
};

}  // namespace mipi_cam

#endif  // HOBOT_MIPI_FAULT_INJECTOR_HPP_
