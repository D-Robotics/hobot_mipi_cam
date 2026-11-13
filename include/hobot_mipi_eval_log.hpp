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

// hobot_mipi_cam 评测埋点统一接口（跨平台，X5/S100/S600 后端共用）
//
// 输出契约（分析工具只认此格式，详见 docs/评测架构定稿.md）：
//   [EVAL] ts_mono=<int64 ns> event=<name> <key=value ...>
// 公共 key（emit 自动添加）: ts_mono（打点时刻 CLOCK_MONOTONIC 纳秒）、
// event（事件名）。payload 里的公共 key（pipe/frame_id/topic/ret）跨平台
// 同名同义，不得重定义。所有时间字段一律 int64 纳秒。
//
// 零侵入约束：默认关闭（不产生任何输出）；开启时只旁路打点，不改数据
// 通路逻辑。开关由 node 参数 eval_log_enable / eval_log_events 运行时配置。

#ifndef HOBOT_MIPI_EVAL_LOG_HPP_
#define HOBOT_MIPI_EVAL_LOG_HPP_

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

namespace mipi_cam {
namespace eval {

// 埋点事件类别（bitmask，可组合）
enum EvalEvent : uint32_t {
  EVAL_EV_NONE = 0u,
  EVAL_EV_STARTUP = 1u << 0,    // 启动阶段耗时（stage/dur_ms）
  EVAL_EV_FRAME_IN = 1u << 1,   // 帧到达/拷贝入队（pipe/stream/frame_id/ts_frame/exp_us/exp_valid）
  EVAL_EV_PUB_OUT = 1u << 2,    // 发布完成（topic/ts_frame）
  EVAL_EV_QUEUE_STAT = 1u << 3, // 队列水位周期汇总（main/sub/combine depth）
  EVAL_EV_ERROR = 1u << 4,      // 采集通路错误（err/pipe/ret）
  EVAL_EV_IMU_ERROR = 1u << 5,  // IMU 通路错误（err/detail）
  EVAL_EV_FRAME_DROP = 1u << 6, // 采集/队列直接丢弃（reason/pipe/stream/ret）
  // 曝光/触发原始量（trig_valid/ts_trig/ts_hw/exp/stamp_src）。
  // 目前仅 X5 实现；trig_valid 沿用 X5 产品代码的 sec&&usec 判据
  // （S100 的 exp_valid 是 sec||usec，跨平台口径待统一）。
  EVAL_EV_EXPOSURE = 1u << 7,
  // 双目/主辅配对点（tag/ok/pipe/frame_id/ts/diff/tol）；失败行限流 1 条/秒
  EVAL_EV_SYNC_PAIR = 1u << 8,
  // 拼接产出（tag/n/order/frame_id/frame_id_left/frame_id_right/
  //           ts_combine/ts_left/ts_right/width/height）
  EVAL_EV_COMBINE_OUT = 1u << 9,
  EVAL_EV_ALL = 0xFFFFFFFFu,
};

// 事件名（[EVAL] 行 event= 的取值，跨平台固定）
inline const char* eval_event_name(uint32_t ev) {
  switch (ev) {
    case EVAL_EV_STARTUP:    return "startup";
    case EVAL_EV_FRAME_IN:   return "frame_in";
    case EVAL_EV_PUB_OUT:    return "pub_out";
    case EVAL_EV_QUEUE_STAT: return "queue_stat";
    case EVAL_EV_ERROR:      return "error";
    case EVAL_EV_IMU_ERROR:  return "imu_error";
    case EVAL_EV_FRAME_DROP: return "frame_drop";
    case EVAL_EV_EXPOSURE:   return "exposure";
    case EVAL_EV_SYNC_PAIR:  return "sync_pair";
    case EVAL_EV_COMBINE_OUT: return "combine_out";
    default:                 return "unknown";
  }
}

inline bool eval_event_from_name(const std::string& name, uint32_t* ev) {
  if (name == "startup")          { *ev = EVAL_EV_STARTUP; return true; }
  if (name == "frame_in")         { *ev = EVAL_EV_FRAME_IN; return true; }
  if (name == "pub_out")          { *ev = EVAL_EV_PUB_OUT; return true; }
  if (name == "queue_stat")       { *ev = EVAL_EV_QUEUE_STAT; return true; }
  if (name == "error")            { *ev = EVAL_EV_ERROR; return true; }
  if (name == "imu_error")        { *ev = EVAL_EV_IMU_ERROR; return true; }
  if (name == "frame_drop")       { *ev = EVAL_EV_FRAME_DROP; return true; }
  if (name == "exposure")         { *ev = EVAL_EV_EXPOSURE; return true; }
  if (name == "sync_pair")        { *ev = EVAL_EV_SYNC_PAIR; return true; }
  if (name == "combine_out")      { *ev = EVAL_EV_COMBINE_OUT; return true; }
  return false;
}

// 类别名列表 -> bitmask，用于解析 eval_log_events 参数。
// "all" 或空串 -> 全部；逗号分隔逐项解析，未知项跳过；
// 全部未知（疑似拼写错误）-> 告警并回退全部，避免静默零输出。
inline uint32_t eval_event_mask_from_names(const std::string& names) {
  if (names.empty() || names == "all") {
    return EVAL_EV_ALL;
  }
  uint32_t mask = EVAL_EV_NONE;
  size_t pos = 0;
  while (pos <= names.size()) {
    size_t comma = names.find(',', pos);
    if (comma == std::string::npos) comma = names.size();
    // 去首尾空白
    size_t b = pos, e = comma;
    while (b < e && (names[b] == ' ' || names[b] == '\t')) ++b;
    while (e > b && (names[e - 1] == ' ' || names[e - 1] == '\t')) --e;
    if (e > b) {
      uint32_t ev = EVAL_EV_NONE;
      if (eval_event_from_name(names.substr(b, e - b), &ev)) {
        mask |= ev;
      }
    }
    if (comma == names.size()) break;
    pos = comma + 1;
  }
  if (mask == EVAL_EV_NONE) {
    fprintf(stderr,
            "[EVAL] warning: no known event in eval_log_events='%s', fallback to all\n",
            names.c_str());
    return EVAL_EV_ALL;
  }
  return mask;
}

inline uint64_t eval_now_mono_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

// 纯格式化：构造一行 [EVAL] 输出（不含换行的写入由调用方/emit 决定）。
// 返回写入字符数（vsnprintf 语义）；payload 为空时不产生尾随空格。
inline int eval_format_line(char* buf, size_t buf_size, uint64_t ts_mono,
                            uint32_t ev, const char* fmt, va_list ap) {
  char payload[384];
  if (fmt == nullptr || fmt[0] == '\0') {
    payload[0] = '\0';
  } else {
    vsnprintf(payload, sizeof(payload), fmt, ap);
  }
  if (payload[0] == '\0') {
    return snprintf(buf, buf_size, "[EVAL] ts_mono=%llu event=%s",
                    static_cast<unsigned long long>(ts_mono), eval_event_name(ev));
  }
  return snprintf(buf, buf_size, "[EVAL] ts_mono=%llu event=%s %s",
                  static_cast<unsigned long long>(ts_mono), eval_event_name(ev),
                  payload);
}

// 进程级埋点配置单例：node 初始化时 configure()，各线程只读。
// 输出 sink 默认 stdout（启用时切行缓冲），测试可重定向。
class EvalLog {
 public:
  static EvalLog& instance() {
    static EvalLog s_instance;
    return s_instance;
  }

  void configure(bool enable, uint32_t mask, FILE* sink = nullptr) {
    enabled_.store(enable, std::memory_order_relaxed);
    mask_.store(mask ? mask : EVAL_EV_NONE, std::memory_order_relaxed);
    if (sink != nullptr) sink_ = sink;
    if (enable && sink_ != nullptr) {
      setvbuf(sink_, nullptr, _IOLBF, 0);  // 行缓冲，保证及时可 grep
    }
  }

  bool enabled() const { return enabled_.load(std::memory_order_relaxed); }
  uint32_t mask() const { return mask_.load(std::memory_order_relaxed); }
  bool event_enabled(uint32_t ev) const {
    return enabled_.load(std::memory_order_relaxed) &&
           (ev & mask_.load(std::memory_order_relaxed)) != 0;
  }

  void emit(uint32_t ev, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
  void emit_va(uint32_t ev, const char* fmt, va_list ap) {
    if (!event_enabled(ev) || sink_ == nullptr) return;
    char line[512];
    eval_format_line(line, sizeof(line), eval_now_mono_ns(), ev, fmt, ap);
    fprintf(sink_, "%s\n", line);
  }

 private:
  EvalLog() = default;
  ~EvalLog() = default;
  std::atomic<bool> enabled_{false};
  std::atomic<uint32_t> mask_{EVAL_EV_ALL};
  FILE* sink_ = stdout;
};

inline void EvalLog::emit(uint32_t ev, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  emit_va(ev, fmt, ap);
  va_end(ap);
}

}  // namespace eval
}  // namespace mipi_cam

// 平台后端/node 层统一调用入口；默认关闭时仅一次原子读开销
#define EVAL_LOG(ev, fmt, ...)                                               \
  do {                                                                       \
    if (mipi_cam::eval::EvalLog::instance().event_enabled(ev)) {             \
      mipi_cam::eval::EvalLog::instance().emit((ev), (fmt), ##__VA_ARGS__);  \
    }                                                                        \
  } while (0)

#endif  // HOBOT_MIPI_EVAL_LOG_HPP_
