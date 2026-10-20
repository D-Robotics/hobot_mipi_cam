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

#ifndef HOBOT_MIPI_DIAG_HPP_
#define HOBOT_MIPI_DIAG_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>

namespace mipi_cam {
namespace diag {

// 采集链路诊断用单调时钟（毫秒），只用于计算间隔，不做时间戳发布。
inline int64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// 单个环节的成功/失败计数与最后一次成功时间。
struct StageStat {
  std::atomic<uint64_t> ok{0};
  std::atomic<uint64_t> fail{0};
  std::atomic<int64_t> last_ok_ms{0};

  void mark_ok() {
    ok.fetch_add(1, std::memory_order_relaxed);
    last_ok_ms.store(now_ms(), std::memory_order_relaxed);
  }
  void mark_fail() { fail.fetch_add(1, std::memory_order_relaxed); }
};

// 每条采集链（pipe）的运行时统计。写侧是采集线程，读侧是诊断线程，
// 全部使用 relaxed 原子；诊断线程自己的上周期快照放在 PipeSnap 中。
struct PipeDiag {
  std::atomic<int64_t> heartbeat_ms{0};      // 采集线程每轮循环刷新
  std::atomic<int64_t> last_fd_ready_ms{0};  // 该 pipe 的 vnode 输出 fd 最近一次可读
  std::atomic<uint64_t> select_wake{0};      // 该 pipe 的 FD_ISSET 次数
  StageStat getframe;                        // hbn_vnode_getframe 成功/失败
  std::atomic<uint64_t> empty_null{0};       // get_empty_buff 返回空
  std::atomic<uint64_t> push{0};             // 成功入 q_buff_
  std::atomic<uint32_t> last_frame_id{0};    // 最近一次 vnode frame_id
  std::atomic<bool> fid_valid{false};
  std::atomic<uint64_t> fid_advanced{0};     // frame_id 累计推进量（含跳变）
  std::atomic<uint64_t> fid_missing{0};      // frame_id 累计缺失量（超过预期步长的部分）
  // 由 start() 在采集线程启动前写入，之后只读。
  uint32_t expected_fid_step = 1;  // frame_id 预期步长：ceil(sensor_fps / app_fps)
  double sensor_fps = 0.0;         // 驱动侧 sensor 帧率，用于区分 SOURCE_SLOW 与 APP_DROP
};

// 诊断线程私有：上一周期快照，用于算增量。
struct PipeSnap {
  uint64_t wake = 0;
  uint64_t gf_ok = 0;
  uint64_t gf_fail = 0;
  uint64_t empty_null = 0;
  uint64_t push = 0;
  uint64_t fid_advanced = 0;
  uint64_t fid_missing = 0;
  int64_t last_report_ms = 0;
};

// 采集链状态。判定顺序见 diagTask()：先静默的环节即根因。
enum class DiagState {
  kOk = 0,
  kThreadDead,     // 采集线程心跳停止（select 退出/卡死）
  kSourceSilent,   // 线程活着，但该 pipe 的 vnode 输出 fd 长时间不可读
  kSourceSlow,     // vnode 取帧与 frame_id 推进都低于配置帧率
  kAppDrop,        // frame_id 正常推进，但应用侧未取到全部帧
  kHwDrop,         // frame_id 出现跳变（驱动层丢帧）
  kBacklog         // 队列长期满，下游消费跟不上
};

inline const char* to_string(DiagState state) {
  switch (state) {
    case DiagState::kOk:
      return "OK";
    case DiagState::kThreadDead:
      return "THREAD_DEAD";
    case DiagState::kSourceSilent:
      return "SOURCE_SILENT";
    case DiagState::kSourceSlow:
      return "SOURCE_SLOW";
    case DiagState::kAppDrop:
      return "APP_DROP";
    case DiagState::kHwDrop:
      return "HW_DROP";
    case DiagState::kBacklog:
      return "BACKLOG";
    default:
      return "UNKNOWN";
  }
}

// 帧率对齐容差：max(2fps, 配置值 10%)，吸收 VSE 分频取整误差。
inline double fps_tolerance(double cfg_fps) {
  double tol = cfg_fps * 0.10;
  return tol < 2.0 ? 2.0 : tol;
}

}  // namespace diag
}  // namespace mipi_cam

#endif  // HOBOT_MIPI_DIAG_HPP_
