#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析链（缝1）测试：合成 fixture 断言指标数值，无 ROS/板端依赖。

运行：python3 -m unittest discover -s src/box/hobot_sensor/hobot_mipi_cam/eval/test/analyze
"""
import os
import sys
import unittest

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools", "analyze")
sys.path.insert(0, TOOLS)

from parsers import parse_clock_sidecar, parse_eval_log, parse_recorder_csv  # noqa: E402
from metrics import (analyze, frame_in_metrics, imu_sync_metrics,  # noqa: E402
                     latency_metrics, pairing_metrics, percentile, topic_metrics)
from report import render_baseline_report  # noqa: E402

MONO_BASE = 500_000_000_000_000          # 5e14 ns（monotonic 域基准）
RT_BASE = 1_700_000_000_000_000_000      # 1.7e18 ns（realtime 域基准）
CLOCK_OFFSET = RT_BASE - MONO_BASE
MS = 1_000_000


def _csv_text():
    """5 帧 image(33ms 间隔) + 5 帧 sub_image(同源同 stamp) + 40 条 IMU(5ms 间隔，realtime 域，覆盖图像全跨度)。"""
    lines = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding"]
    for i in range(5):
        stamp = MONO_BASE + i * 33 * MS
        recv = stamp + 3 * MS                      # 恒定 3ms 接收延迟
        lines.append(f"/t/image_raw,,{stamp},{recv},1920,1080,nv12")
        lines.append(f"/t/sub_image_raw,,{stamp},{recv + 100000},960,540,nv12")
    for i in range(40):
        rt_stamp = RT_BASE + i * 5 * MS            # IMU stamp 在 realtime 域
        recv = MONO_BASE + i * 5 * MS + MS // 2
        lines.append(f"/t/imu,,{rt_stamp},{recv},0,0,imu")
    return "\n".join(lines)


def _eval_text():
    """frame_in(pipe0 main): id 1,2,4,5（丢 id=3）；前3帧曝光有效 10ms；1 帧 exp_valid=0。"""
    lines = []
    stamps = [MONO_BASE + i * 33 * MS for i in range(5)]
    ids = [1, 2, 4, 5]
    for idx, fid in enumerate(ids):
        if idx < 3:
            lines.append(f"[EVAL] ts_mono={stamps[idx] + 500000} event=frame_in "
                         f"pipe=0 stream=main frame_id={fid} ts_frame={stamps[idx]} "
                         f"exp_us=10000 exp_valid=1")
        else:
            lines.append(f"[EVAL] ts_mono={stamps[idx] + 500000} event=frame_in "
                         f"pipe=0 stream=main frame_id={fid} ts_frame={stamps[idx]} "
                         f"exp_us=0 exp_valid=0")
    lines.append(f"[EVAL] ts_mono={MONO_BASE} event=pub_out topic=/t/image_raw ts_frame={stamps[0]}")
    lines.append(f"[EVAL] ts_mono={MONO_BASE} event=queue_stat pipe=0 main_depth=0 sub_depth=0 combine_depth=0")
    lines.append("this is not an eval line")
    lines.append("[EVAL] broken line without ts")
    return "\n".join(lines)


class TestParsers(unittest.TestCase):
    def test_eval_log_parse_and_skip(self):
        evs = parse_eval_log(_eval_text())
        self.assertEqual(len(evs), 4 + 1 + 1)  # 4 frame_in + pub_out + queue_stat
        fi = [e for e in evs if e["event"] == "frame_in"]
        self.assertEqual(fi[0]["kv"]["exp_us"], 10000)
        self.assertEqual(fi[0]["kv"]["exp_valid"], 1)

    def test_csv_parse_and_skip(self):
        rows, skipped = parse_recorder_csv("topic,seq,stamp_ns,recv_mono_ns,width,height,encoding\n"
                                           "/a,,1,2,3,4,nv12\nbad,line\n")
        self.assertEqual(len(rows), 1)
        self.assertEqual(skipped, 1)
        self.assertEqual(rows[0].stamp_ns, 1)

    def test_clock_sidecar(self):
        self.assertEqual(parse_clock_sidecar(
            "realtime_ns,mono_ns,offset_ns\n123,100,23\n"), 23)
        self.assertIsNone(parse_clock_sidecar("nothing valid"))


class TestMetrics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows, _ = parse_recorder_csv(_csv_text())
        cls.events = parse_eval_log(_eval_text())

    def test_percentile(self):
        self.assertEqual(percentile([1.0, 2.0, 3.0, 4.0], 50), 2.5)
        self.assertEqual(percentile([5.0], 99), 5.0)
        self.assertIsNone(percentile([], 50))

    def test_topic_freq_and_monotonic(self):
        m = topic_metrics(self.rows)
        img = m["/t/image_raw"]
        self.assertEqual(img["count"], 5)
        self.assertAlmostEqual(img["fps_mean"], 5 / 0.132, delta=0.01)
        self.assertEqual(img["stamp_monotonic_violations"], 0)
        self.assertEqual(img["stamp_duplicates"], 0)
        itv = img["recv_interval_ns"]
        self.assertEqual(itv["min"], 33 * MS)

    def test_frame_in_drop_reconstruction(self):
        m = frame_in_metrics(self.events)
        p0 = m["pipe0/main"]
        self.assertEqual(p0["count"], 4)
        self.assertEqual(p0["dropped_by_frame_id"], 1)   # id 3 缺失
        self.assertAlmostEqual(p0["drop_rate"], 0.2, delta=1e-6)
        self.assertAlmostEqual(p0["exp_valid_ratio"], 0.75, delta=1e-6)
        self.assertEqual(p0["exp_us_mean"], 10000)

    def test_latency_exposure_midpoint(self):
        # recv = stamp + 3ms；t0 = stamp − 5ms(曝光中点) → 曝光组延迟 8ms
        # frame_ts 组 = 无有效曝光帧(exp_valid=0) + 无 frame_in 埋点的帧(丢帧后) = 2
        m = latency_metrics(self.rows, self.events, "mono", CLOCK_OFFSET)
        lat = m["/t/image_raw"]
        self.assertEqual(lat["exp_midpoint"]["count"], 3)
        self.assertEqual(lat["exp_midpoint"]["p50_ms"], 8.0)
        self.assertEqual(lat["frame_ts"]["count"], 2)
        self.assertEqual(lat["frame_ts"]["p50_ms"], 3.0)

    def test_pairing_same_source(self):
        p = pairing_metrics(self.rows, "/t/image_raw", "/t/sub_image_raw")
        self.assertEqual(p["paired"], 5)
        self.assertEqual(p["pair_rate"], 1.0)
        self.assertEqual(p["diff_max_ns"], 0)

    def test_imu_nearest_after_normalization(self):
        # IMU realtime→mono 归一后与图像 33ms 网格对齐：最近邻 ≤2.5ms（半个 IMU 周期）
        m = imu_sync_metrics(self.rows, "/t/image_raw", "mono", CLOCK_OFFSET)
        self.assertEqual(m["imu_count"], 40)
        self.assertLessEqual(m["nearest_max_ns"], 2.5 * MS)

    def test_analyze_full_and_report(self):
        result = analyze(self.rows, _eval_text(), "mono", CLOCK_OFFSET)
        self.assertEqual(result["meta"]["clock_offset_ns"], CLOCK_OFFSET)
        self.assertIn("frame_in", result)
        md = render_baseline_report(result)
        self.assertIn("基线模式", md)
        self.assertIn("/t/image_raw", md)
        self.assertIn("曝光中点", md)
        self.assertIn("8.00", md)


if __name__ == "__main__":
    unittest.main()
