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

// 缝3：埋点输出契约测试（纯函数，无 ROS 依赖）。
// 本测试固化的行格式是三平台共享的对外接口承诺，字段名不得变更。

#include <gtest/gtest.h>

#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <string>

#include "hobot_mipi_eval_log.hpp"

namespace {

std::string fmt_line(uint64_t ts, uint32_t ev, const char *fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  mipi_cam::eval::eval_format_line(buf, sizeof(buf), ts, ev, fmt, ap);
  va_end(ap);
  return std::string(buf);
}

std::string fmt_line_buf(char *buf, size_t size, uint64_t ts, uint32_t ev,
                         const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  mipi_cam::eval::eval_format_line(buf, size, ts, ev, fmt, ap);
  va_end(ap);
  return std::string(buf);
}

}  // namespace

TEST(EvalEventName, AllEvents) {
  using namespace mipi_cam::eval;
  EXPECT_STREQ("startup", eval_event_name(EVAL_EV_STARTUP));
  EXPECT_STREQ("frame_in", eval_event_name(EVAL_EV_FRAME_IN));
  EXPECT_STREQ("pub_out", eval_event_name(EVAL_EV_PUB_OUT));
  EXPECT_STREQ("queue_stat", eval_event_name(EVAL_EV_QUEUE_STAT));
  EXPECT_STREQ("error", eval_event_name(EVAL_EV_ERROR));
  EXPECT_STREQ("imu_error", eval_event_name(EVAL_EV_IMU_ERROR));
  EXPECT_STREQ("frame_drop", eval_event_name(EVAL_EV_FRAME_DROP));
  EXPECT_STREQ("exposure", eval_event_name(EVAL_EV_EXPOSURE));
  EXPECT_STREQ("sync_pair", eval_event_name(EVAL_EV_SYNC_PAIR));
  EXPECT_STREQ("combine_out", eval_event_name(EVAL_EV_COMBINE_OUT));
}

TEST(EvalEventName, ParseNewEvents) {
  using namespace mipi_cam::eval;
  uint32_t ev = 0;
  ASSERT_TRUE(eval_event_from_name("exposure", &ev));
  EXPECT_EQ(ev, EVAL_EV_EXPOSURE);
  ASSERT_TRUE(eval_event_from_name("sync_pair", &ev));
  EXPECT_EQ(ev, EVAL_EV_SYNC_PAIR);
  ASSERT_TRUE(eval_event_from_name("frame_drop", &ev));
  EXPECT_EQ(ev, EVAL_EV_FRAME_DROP);
  ASSERT_TRUE(eval_event_from_name("combine_out", &ev));
  EXPECT_EQ(ev, EVAL_EV_COMBINE_OUT);
}

TEST(EvalFormatLine, PayloadPassthrough) {
  const auto s = fmt_line(1234567890ULL, mipi_cam::eval::EVAL_EV_FRAME_IN,
                          "pipe=%d stream=%s frame_id=%u ts_frame=%lld "
                          "exp_us=%ld exp_valid=%d",
                          1, "main", 42, 1234567890123LL, 5000L, 1);
  EXPECT_EQ(s, "[EVAL] ts_mono=1234567890 event=frame_in "
               "pipe=1 stream=main frame_id=42 ts_frame=1234567890123 "
               "exp_us=5000 exp_valid=1");
}

TEST(EvalFormatLine, EmptyPayloadHasNoTrailingSpace) {
  const auto s = fmt_line(7, mipi_cam::eval::EVAL_EV_QUEUE_STAT, "");
  EXPECT_EQ(s, "[EVAL] ts_mono=7 event=queue_stat");
}

TEST(EvalFormatLine, NegativeAndLargeValues) {
  const auto s = fmt_line(9, mipi_cam::eval::EVAL_EV_ERROR,
                          "err=%s pipe=%d ret=%d", "getframe_fail", 0, -22);
  EXPECT_EQ(s, "[EVAL] ts_mono=9 event=error err=getframe_fail pipe=0 ret=-22");
  // int64 上限（曝光/时间戳字段必须容纳）
  const auto big = fmt_line(9, mipi_cam::eval::EVAL_EV_FRAME_IN, "ts_frame=%lld",
                            9223372036854775807LL);
  EXPECT_NE(std::string::npos, big.find("ts_frame=9223372036854775807"));
}

TEST(EvalFormatLine, TruncationSafe) {
  // 小缓冲 + 长 payload：不得越界，必须以 NUL 结尾
  char small[24];
  char payload_fmt[256];
  const std::string long_payload(200, 'x');
  snprintf(payload_fmt, sizeof(payload_fmt), "data=%s", long_payload.c_str());
  const auto s = fmt_line_buf(small, sizeof(small), 1,
                              mipi_cam::eval::EVAL_EV_FRAME_IN, "%s",
                              payload_fmt);
  (void)s;
  EXPECT_EQ(strlen(small), sizeof(small) - 1);
}

TEST(EvalMaskParse, AllAndEmpty) {
  using namespace mipi_cam::eval;
  EXPECT_EQ(EVAL_EV_ALL, eval_event_mask_from_names("all"));
  EXPECT_EQ(EVAL_EV_ALL, eval_event_mask_from_names(""));
}

TEST(EvalMaskParse, ListAndSpaces) {
  using namespace mipi_cam::eval;
  const uint32_t m = eval_event_mask_from_names(" frame_in , pub_out ");
  EXPECT_EQ(EVAL_EV_FRAME_IN | EVAL_EV_PUB_OUT, m);
}

TEST(EvalMaskParse, UnknownSkipped) {
  using namespace mipi_cam::eval;
  EXPECT_EQ(EVAL_EV_FRAME_IN, eval_event_mask_from_names("foo,frame_in"));
  // 全未知（拼写错误场景）回退全部并告警，避免静默零输出
  EXPECT_EQ(EVAL_EV_ALL, eval_event_mask_from_names("foo"));
  EXPECT_EQ(EVAL_EV_ALL, eval_event_mask_from_names("ALL"));  // 大小写敏感
}

TEST(EvalLogGate, DisabledAndFilter) {
  auto &log = mipi_cam::eval::EvalLog::instance();
  log.configure(false, mipi_cam::eval::EVAL_EV_ALL);
  EXPECT_FALSE(log.enabled());
  EXPECT_FALSE(log.event_enabled(mipi_cam::eval::EVAL_EV_FRAME_IN));

  log.configure(true, mipi_cam::eval::EVAL_EV_FRAME_IN | mipi_cam::eval::EVAL_EV_ERROR);
  EXPECT_TRUE(log.event_enabled(mipi_cam::eval::EVAL_EV_FRAME_IN));
  EXPECT_FALSE(log.event_enabled(mipi_cam::eval::EVAL_EV_PUB_OUT));

  log.configure(false, mipi_cam::eval::EVAL_EV_ALL);  // 恢复默认关闭
}

TEST(EvalLogEmit, SinkOutputContract) {
  auto &log = mipi_cam::eval::EvalLog::instance();
  FILE *tmp = tmpfile();
  ASSERT_NE(nullptr, tmp);
  log.configure(true, mipi_cam::eval::EVAL_EV_ALL, tmp);

  EVAL_LOG(mipi_cam::eval::EVAL_EV_ERROR, "err=%s ret=%d pipe=%d",
           "getframe_fail", -1, 2);
  EVAL_LOG(mipi_cam::eval::EVAL_EV_FRAME_IN, "pipe=%d frame_id=%u", 0, 7);
  fflush(tmp);

  char line[512];
  uint64_t ts1 = 0, ts2 = 0;
  int n_lines = 0;
  rewind(tmp);
  while (fgets(line, sizeof(line), tmp) != nullptr) {
    ++n_lines;
    ASSERT_EQ(0, strncmp(line, "[EVAL] ts_mono=", 15));
    uint64_t ts = 0;
    sscanf(line + 15, "%llu", reinterpret_cast<unsigned long long *>(&ts));
    if (n_lines == 1) {
      ts1 = ts;
      EXPECT_NE(nullptr, strstr(line, "event=error err=getframe_fail ret=-1 pipe=2"));
    } else {
      ts2 = ts;
      EXPECT_NE(nullptr, strstr(line, "event=frame_in pipe=0 frame_id=7"));
    }
  }
  EXPECT_EQ(2, n_lines);
  EXPECT_LE(ts1, ts2);  // ts_mono 不回退

  fclose(tmp);
  log.configure(false, mipi_cam::eval::EVAL_EV_ALL);  // 恢复默认关闭
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
