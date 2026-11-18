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
from metrics import (analyze, drop_metrics, expected_from_launch_args,  # noqa: E402
                     frame_in_metrics, frame_timing_metrics, imu_sync_metrics,
                     latency_metrics, pairing_metrics, percentile, resolution_metrics,
                     stability_warnings, startup_metrics, topic_metrics)
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
        # 无偏估计：(N-1)/时长 = 4/0.132
        self.assertAlmostEqual(img["fps_mean"], 4 / 0.132, delta=0.01)
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

    def test_frame_timing_metrics(self):
        lines = []
        # pipe0：5 帧 33ms，正常
        for i in range(5):
            ts = MONO_BASE + i * 33 * MS
            lines.append(f"[EVAL] ts_mono={ts + MS} event=frame_in "
                         f"pipe=0 stream=main ts_frame={ts}")
        # pipe1：最后一帧 stamp 回退且重复（ts_mono 各帧不同，模拟真实打点）
        for i, ts in enumerate((MONO_BASE, MONO_BASE + 33 * MS,
                                MONO_BASE + 66 * MS, MONO_BASE + 33 * MS)):
            lines.append(f"[EVAL] ts_mono={ts + (2 + i) * MS} event=frame_in "
                         f"pipe=1 stream=main ts_frame={ts}")
        evs = parse_eval_log("\n".join(lines))
        m = frame_timing_metrics(evs)
        p0, p1 = m["pipe0/main"], m["pipe1/main"]
        self.assertEqual(p0["count"], 5)
        self.assertAlmostEqual(p0["fps_mean"], 30.303, places=2)
        self.assertEqual(p0["stamp_monotonic_violations"], 0)
        self.assertEqual(p0["stamp_duplicates"], 0)
        self.assertEqual(p1["count"], 4)
        self.assertEqual(p1["stamp_monotonic_violations"], 1)
        self.assertEqual(p1["stamp_duplicates"], 1)
        # X5 frame_in 无 frame_id/exp 时，frame_in_metrics 不应误报
        fi = frame_in_metrics(evs)
        self.assertIsNone(fi["pipe0/main"]["dropped_by_frame_id"])
        self.assertIsNone(fi["pipe0/main"]["exp_valid_ratio"])

    def test_drop_metrics(self):
        lines = []
        for fid in (1, 2, 4, 5):  # 缺 id=3
            ts = MONO_BASE + fid * 33 * MS
            lines.append(f"[EVAL] ts_mono={ts} event=frame_in "
                         f"pipe=0 stream=main frame_id={fid} ts_frame={ts}")
        lines.append(
            "[EVAL] ts_mono=2000 event=queue_stat pipe=0 main_depth=3 sub_depth=0 "
            "combine_depth=0 main_recycled=2 sub_recycled=0 combine_recycled=0 "
            "main_empty_null=1 sub_empty_null=0 combine_empty_null=0 "
            "main_fq_full=1 main_fq_expired=0 sub_fq_full=0 sub_fq_expired=0")
        lines.append("[EVAL] ts_mono=2100 event=frame_drop pipe=0 stream=main "
                     "reason=getframe_fail ret=-1")
        lines.append("[EVAL] ts_mono=2200 event=frame_drop pipe=- stream=main "
                     "reason=select_timeout timeout_s=2")
        d = drop_metrics(parse_eval_log("\n".join(lines)))
        s = d["streams"]["pipe0/main"]
        self.assertEqual(s["frames_in"], 4)
        self.assertEqual(s["drops_by_reason"]["queue_recycle"], 2)
        self.assertEqual(s["drops_by_reason"]["empty_null"], 1)
        self.assertEqual(s["drops_by_reason"]["fq_full"], 1)
        self.assertEqual(s["drops_by_reason"]["getframe_fail"], 1)
        self.assertEqual(s["drops_total"], 5)
        self.assertAlmostEqual(s["drop_rate"], 5 / 9, places=6)
        self.assertEqual(s["frame_id_gaps"], 1)
        self.assertEqual(d["task_drops"]["main"]["select_timeout"], 1)
        self.assertTrue(d["queue_counters_available"])

    def test_expected_from_launch_args(self):
        meta = ("case=C04\n"
                "launch_args=device_mode:=dual dual_combine:=1 image_width:=1088 "
                "image_height:=1280 sub_stream_enable:=False out_format:=nv12\n")
        e = expected_from_launch_args(meta)
        self.assertEqual(e["main"]["width"], 1088)
        self.assertEqual(e["main"]["height"], 1280)
        self.assertEqual(e["combine"]["width"], 1088)
        self.assertEqual(e["combine"]["height"], 2560)
        self.assertEqual(e["info"]["encoding"], "camera_info")

    def test_resolution_metrics(self):
        csv = "\n".join([
            "topic,seq,stamp_ns,recv_mono_ns,width,height,encoding",
            f"/image_left_raw,,1,1,1088,1280,nv12",
            f"/image_right_raw,,1,1,1080,1280,nv12",
            f"/image_combine_raw,,1,1,1088,2560,nv12",
            f"/image_left_raw/camera_info,,1,1,1088,1280,camera_info",
        ])
        eval_text = "\n".join([
            "[EVAL] ts_mono=1 event=frame_in pipe=0 stream=main ts_frame=1 width=1088 height=1280",
            "[EVAL] ts_mono=2 event=frame_in pipe=1 stream=main ts_frame=1 width=1080 height=1280",
        ])
        expected = expected_from_launch_args(
            "launch_args=device_mode:=dual dual_combine:=1 image_width:=1088 "
            "image_height:=1280 sub_stream_enable:=False out_format:=nv12")
        rows, _ = parse_recorder_csv(csv)
        r = resolution_metrics(rows, parse_eval_log(eval_text), expected)
        self.assertEqual(r["device"]["pipe0/main"]["mismatch"], 0)
        self.assertEqual(r["device"]["pipe1/main"]["mismatch"], 1)
        self.assertEqual(r["published"]["/image_right_raw"]["mismatch"], 1)
        self.assertEqual(r["published"]["/image_combine_raw"]["mismatch"], 0)
        self.assertFalse(r["all_match"])

    def test_resolution_missing_and_device_uncovered(self):
        csv = "\n".join([
            "topic,seq,stamp_ns,recv_mono_ns,width,height,encoding",
            f"/image_left_raw,,1,1,1088,1280,nv12",
        ])
        expected = expected_from_launch_args(
            "launch_args=device_mode:=dual dual_combine:=1 image_width:=1088 "
            "image_height:=1280 sub_stream_enable:=False out_format:=nv12")
        rows, _ = parse_recorder_csv(csv)
        r = resolution_metrics(rows, [], expected,
                               ["/image_left_raw", "/image_right_raw"])
        self.assertFalse(r["device_covered"])
        self.assertIn("/image_right_raw", r["missing_topics"])
        self.assertFalse(r["all_match"])

    def test_sub_combine_expected(self):
        e = expected_from_launch_args(
            "launch_args=device_mode:=dual dual_combine:=1 image_width:=1088 "
            "image_height:=1280 sub_stream_enable:=True sub_image_width:=544 "
            "sub_image_height:=640 out_format:=nv12")
        self.assertEqual(e["sub_combine"]["width"], 544)
        self.assertEqual(e["sub_combine"]["height"], 1280)
        self.assertEqual(e["info_sub"]["width"], 544)

    def test_stability_warnings(self):
        ft = {"pipe0/main": {"interval_max_ns": 23_200_000_000}}
        w = stability_warnings(ft, {"framerate": 10.0})
        self.assertEqual(len(w), 1)
        self.assertEqual(w[0]["threshold_ms"], 300.0)
        self.assertEqual(stability_warnings(ft, {"framerate": 0.0}), [])

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

    def test_realtime_domain_exp_match(self):
        # realtime 域：stamp = ts_frame + offset + 12ms 管线延迟 → 最近邻匹配仍命中曝光
        stamps = [MONO_BASE + i * 33 * MS for i in range(3)]
        csv = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding"]
        eval_lines = []
        for i, s in enumerate(stamps):
            rt_stamp = s + CLOCK_OFFSET + 12 * MS
            csv.append(f"/t/img_rt,,{rt_stamp},{s + 20 * MS},1920,1080,nv12")
            eval_lines.append(f"[EVAL] ts_mono={s} event=frame_in pipe=0 stream=main "
                              f"frame_id={i + 1} ts_frame={s} exp_us=10000 exp_valid=1")
        rows, _ = parse_recorder_csv("\n".join(csv))
        lat = latency_metrics(rows, parse_eval_log("\n".join(eval_lines)),
                              "realtime", CLOCK_OFFSET)
        # realtime 口径（方案 6.1）：t0 = 帧ts + exp/2
        # 归一后 stamp_mono = s+12ms，recv = s+20ms → lat=8ms；−exp/2 → 3ms
        self.assertEqual(lat["/t/img_rt"]["exp_midpoint"]["count"], 3)
        self.assertEqual(lat["/t/img_rt"]["exp_midpoint"]["p50_ms"], 3.0)

    def test_domain_autodetect(self):
        self.assertEqual(analyze(self.rows, "")["meta"]["image_domain"], "mono")
        rt_rows, _ = parse_recorder_csv(
            "topic,seq,stamp_ns,recv_mono_ns,width,height,encoding\n"
            f"/t/x,,{RT_BASE},{MONO_BASE},1,1,nv12\n")
        self.assertEqual(analyze(rt_rows, "")["meta"]["image_domain"], "realtime")

    def test_startup_metrics(self):
        text = "\n".join([
            "[EVAL] ts_mono=1000000000 event=startup stage=node_init pipe=- t0_ns=900000000",
            "[EVAL] ts_mono=1100000000 event=startup stage=vflow_start_begin pipe=0",
            "[EVAL] ts_mono=1200000000 event=startup stage=vflow_start_end pipe=0 ret=0",
            "[EVAL] ts_mono=1300000000 event=startup stage=backend_init_end pipe=- ret=0",
            "[EVAL] ts_mono=1400000000 event=startup stage=backend_start_end pipe=- ret=0",
            "[EVAL] ts_mono=1500000000 event=startup stage=first_frame pipe=0 stream=main",
            "[EVAL] ts_mono=1650000000 event=startup stage=first_frame pipe=0 stream=main",
            "[EVAL] ts_mono=1700000000 event=startup stage=first_frame pipe=1 stream=main",
            "[EVAL] ts_mono=1600000000 event=startup stage=first_pub pipe=- topic=/image_left_raw channel=left",
            "[EVAL] ts_mono=1900000000 event=startup stage=first_pub pipe=- topic=/image_left_raw channel=left",
            "[EVAL] ts_mono=1800000000 event=startup stage=first_pub pipe=- topic=/image_right_raw channel=right",
        ])
        st = startup_metrics(parse_eval_log(text))
        self.assertIsNotNone(st)
        self.assertEqual(st["time_to_first_frame_ms"], 600.0)
        self.assertEqual(st["time_to_first_pub_ms"], 700.0)
        self.assertEqual(len(st["first_frame"]), 2)
        self.assertEqual(st["first_frame"]["pipe0/main"]["since_t0_ms"], 600.0)
        self.assertEqual(st["first_pub"]["/image_left_raw"]["since_t0_ms"], 700.0)
        self.assertIn("/image_right_raw", st["first_pub"])
        self.assertTrue(st["complete"])
        self.assertEqual(st["stage_durations"][0]["duration_ms"], 100.0)

    def test_startup_metrics_missing_t0_and_incomplete(self):
        text = "\n".join([
            "[EVAL] ts_mono=1000000000 event=startup stage=node_init pipe=-",
            "[EVAL] ts_mono=1100000000 event=startup stage=backend_init_end pipe=- ret=-1",
        ])
        st = startup_metrics(parse_eval_log(text))
        self.assertEqual(st["t0_ns"], 1000000000)  # 回退到 node_init 事件时刻
        self.assertIsNone(st["time_to_first_frame_ms"])
        self.assertFalse(st["complete"])

    def test_startup_missing_first_pub(self):
        csv = "\n".join([
            "topic,seq,stamp_ns,recv_mono_ns,width,height,encoding",
            f"/t/a,,{MONO_BASE},{MONO_BASE + MS},1,1,nv12",
            f"/t/b,,{MONO_BASE},{MONO_BASE + MS},1,1,nv12",
        ])
        eval_text = "\n".join([
            "[EVAL] ts_mono=1000000000 event=startup stage=node_init pipe=- t0_ns=900000000",
            "[EVAL] ts_mono=1500000000 event=startup stage=first_frame pipe=0 stream=main",
            "[EVAL] ts_mono=1600000000 event=startup stage=first_pub pipe=- topic=/t/a channel=main",
            "[EVAL] ts_mono=1700000000 event=startup stage=backend_init_end pipe=- ret=0",
            "[EVAL] ts_mono=1800000000 event=startup stage=backend_start_end pipe=- ret=0",
        ])
        rows, _ = parse_recorder_csv(csv)
        res = analyze(rows, eval_text, "mono")
        self.assertEqual(res["startup"]["missing_first_pub"], ["/t/b"])

    def _exposure_events(self, lines):
        return parse_eval_log("\n".join(lines))

    def test_exposure_no_trig_sensor(self):
        from metrics import exposure_metrics
        text = [
            f"[EVAL] ts_mono=1 event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=0 ts_trig_ns=0 ts_hw_ns=0 exp_us=0 "
            f"ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE}",
            f"[EVAL] ts_mono=2 event=exposure pipe=0 stream=main frame_id=2 "
            f"stamp_src=sensor trig_valid=0 ts_trig_ns=0 ts_hw_ns=0 exp_us=0 "
            f"ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE + 33 * MS}",
        ]
        ex = exposure_metrics(self._exposure_events(text))
        self.assertEqual(ex["total"], 2)
        p = ex["pipes"]["0/main"]
        self.assertEqual(p["trig_valid_ratio"], 0.0)
        self.assertIsNone(p["exp_us"])
        self.assertEqual(p["stamp_src"], {"sensor": 2})
        self.assertEqual(p["frame_domain"], "mono")
        self.assertEqual(p["sys_raw_domain"], "realtime")
        # 无曝光 -> t0 = 帧 ts，间隔 33ms
        self.assertAlmostEqual(p["t0_interval_ms"]["p50"], 33.0, places=3)
        self.assertEqual(p["t0_backwards"], 0)

    def test_exposure_with_trig_t0(self):
        from metrics import exposure_metrics
        text = [
            f"[EVAL] ts_mono=1 event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=1000000000 ts_hw_ns=1010000000 "
            f"exp_us=10000 ts_sys_raw_ns={RT_BASE} ts_frame=2000000000",
            f"[EVAL] ts_mono=2 event=exposure pipe=0 stream=main frame_id=2 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=1033333000 ts_hw_ns=1043333000 "
            f"exp_us=10000 ts_sys_raw_ns={RT_BASE} ts_frame=2033333000",
        ]
        p = exposure_metrics(self._exposure_events(text))["pipes"]["0/main"]
        self.assertEqual(p["trig_valid_ratio"], 1.0)
        self.assertEqual(p["exp_us"]["p50"], 10000.0)
        # t0 = trig + exp/2
        self.assertAlmostEqual(p["t0_interval_ms"]["p50"], 33.333, places=3)
        self.assertEqual(p["t0_backwards"], 0)
        # hw 与 frame 同域：hw-frame = +10ms（1.01e9-2e9 为负，故只断言存在）
        self.assertIsNotNone(p["hw_minus_frame_ms"])

    def test_exposure_realtime_adjust(self):
        from metrics import exposure_metrics
        ok = RT_BASE - 10 * MS          # sys_raw 回退 exp(=10ms)
        text = [
            f"[EVAL] ts_mono=1 event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=realtime trig_valid=1 ts_trig_ns=1 ts_hw_ns=2 exp_us=10000 "
            f"ts_sys_raw_ns={RT_BASE} ts_frame={ok}",
            f"[EVAL] ts_mono=2 event=exposure pipe=0 stream=main frame_id=2 "
            f"stamp_src=realtime trig_valid=1 ts_trig_ns=1 ts_hw_ns=2 exp_us=10000 "
            f"ts_sys_raw_ns={RT_BASE} ts_frame={RT_BASE}",  # 未回退
        ]
        p = exposure_metrics(self._exposure_events(text))["pipes"]["0/main"]
        self.assertEqual(p["stamp_src"], {"realtime": 2})
        self.assertEqual(p["frame_domain"], "realtime")
        self.assertEqual(p["realtime_adjust"]["checked"], 2)
        self.assertEqual(p["realtime_adjust"]["mismatch"], 1)

    def test_exposure_negative_and_report_warning(self):
        from metrics import exposure_metrics
        text = [
            f"[EVAL] ts_mono=1 event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=5000000000 ts_hw_ns=4000000000 "
            f"exp_us=-1000 ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE}",
        ]
        ex = exposure_metrics(self._exposure_events(text))
        self.assertEqual(ex["pipes"]["0/main"]["exp_negative"], 1)
        md = render_baseline_report({"exposure": ex})
        self.assertIn("负曝光值", md)

    def test_exposure_stale_trig_not_usable(self):
        """lpwm 关闭时 trig_tv 残留陈旧固定值：trig_valid=1 但曝光跨域不可用。"""
        from metrics import exposure_metrics
        text = [
            f"[EVAL] ts_mono=1 event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=73999485154000 "
            f"ts_hw_ns=243658362766000 exp_us=169658877612 "
            f"ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE}",
            f"[EVAL] ts_mono=2 event=exposure pipe=0 stream=main frame_id=3 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=73999485154000 "
            f"ts_hw_ns=243658396099000 exp_us=169658910945 "
            f"ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE + 66 * MS}",
        ]
        p = exposure_metrics(self._exposure_events(text))["pipes"]["0/main"]
        self.assertEqual(p["trig_valid_ratio"], 1.0)
        self.assertEqual(p["trig_usable_ratio"], 0.0)
        self.assertEqual(p["exp_over_1s"], 2)
        # 曝光不可用 -> 不用 exp 修正，退化为帧 ts 口径
        self.assertEqual(p["t0_basis"], {"frame_ts_sensor": 2})
        self.assertAlmostEqual(p["t0_interval_ms"]["p50"], 66.0, places=3)
        md = render_baseline_report(
            {"exposure": exposure_metrics(self._exposure_events(text))})
        self.assertIn("trig_tv 恒定", md)

    def test_exposure_stale_trig_small_exp_rejected(self):
        """trig 恒定即判陈旧，即使 exp 落在 1s 内也不可用。"""
        from metrics import exposure_metrics
        text = [
            f"[EVAL] ts_mono=1 event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=5000000000 ts_hw_ns=5004000000 "
            f"exp_us=4000 ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE}",
            f"[EVAL] ts_mono=2 event=exposure pipe=0 stream=main frame_id=2 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=5000000000 ts_hw_ns=5037000000 "
            f"exp_us=37000 ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE + 33 * MS}",
        ]
        p = exposure_metrics(self._exposure_events(text))["pipes"]["0/main"]
        self.assertTrue(p["trig_stale"])
        self.assertEqual(p["trig_usable_ratio"], 0.0)
        self.assertEqual(p["t0_basis"], {"frame_ts_sensor": 2})

    def test_exposure_long_exposure_advancing_trig_usable(self):
        """1fps 下 1.2s 长曝光：触发在推进 + 上限随帧间隔放宽，应可用。"""
        from metrics import exposure_metrics
        text = [
            f"[EVAL] ts_mono=1 event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=1000000000 ts_hw_ns=2200000000 "
            f"exp_us=1200000 ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE}",
            f"[EVAL] ts_mono=2 event=exposure pipe=0 stream=main frame_id=2 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=2000000000 ts_hw_ns=3200000000 "
            f"exp_us=1200000 ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE + 1000 * MS}",
        ]
        p = exposure_metrics(self._exposure_events(text))["pipes"]["0/main"]
        self.assertFalse(p["trig_stale"])
        self.assertEqual(p["trig_usable_ratio"], 1.0)
        self.assertGreaterEqual(p["exp_cap_us"], 1200000)
        self.assertEqual(p["t0_basis"], {"trig": 2})

    def test_exposure_mixed_basis_flagged(self):
        from metrics import exposure_metrics
        text = [
            f"[EVAL] ts_mono=1 event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=1000000000 ts_hw_ns=1005000000 "
            f"exp_us=5000 ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE}",
            f"[EVAL] ts_mono=2 event=exposure pipe=0 stream=main frame_id=2 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns=1033000000 ts_hw_ns=4033000000 "
            f"exp_us=3000000 ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE + 33 * MS}",
        ]
        p = exposure_metrics(self._exposure_events(text))["pipes"]["0/main"]
        self.assertTrue(p["t0_mixed"])
        self.assertEqual(p["t0_basis"], {"frame_ts_sensor": 1, "trig": 1})

    def test_exposure_event_drives_latency_midpoint(self):
        """仅有 exposure 事件时，延迟也能拿到曝光中点口径。"""
        from metrics import latency_metrics
        csv = "\n".join([
            "topic,seq,stamp_ns,recv_mono_ns,width,height,encoding",
            f"/t/img,,{MONO_BASE},{MONO_BASE + 50 * MS},1,1,nv12",
        ])
        ev = (f"[EVAL] ts_mono={MONO_BASE} event=exposure pipe=0 stream=main frame_id=1 "
              f"stamp_src=sensor trig_valid=1 ts_trig_ns=1000 ts_hw_ns=2000 exp_us=10000 "
              f"ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE}")
        rows, _ = parse_recorder_csv(csv)
        lat = latency_metrics(rows, parse_eval_log(ev), "mono")
        self.assertEqual(lat["/t/img"]["exp_midpoint"]["count"], 1)
        self.assertEqual(lat["/t/img"]["exp_midpoint"]["p50_ms"], 55.0)

    def _sync_lines(self):
        lines = []
        for i in range(3):
            for p in (0, 1):
                lines.append(f"[EVAL] ts_mono={1000 + i} event=frame_in pipe={p} "
                             f"stream=main frame_id={i + 1} ts_frame={MONO_BASE + i * 33 * MS}")
        for i in range(3):
            lines.append(
                f"[EVAL] ts_mono={2000 + i} event=sync_pair tag=main ok=1 "
                f"pipe_a=0 pipe_b=1 frame_id_a={i + 1} frame_id_b={i + 1} "
                f"ts_a={MONO_BASE + i * 33 * MS} ts_b={MONO_BASE + i * 33 * MS + 100000} "
                f"diff_ns=100000 tol_ns=16666666")
        lines.append("[EVAL] ts_mono=3000 event=sync_pair tag=main ok=0 "
                     "reason=diff_over_tol missing=0 pipe_a=0 pipe_b=1 "
                     "frame_id_a=4 frame_id_b=5 ts_a=1 ts_b=20000001 "
                     "diff_ns=20000000 tol_ns=16666666")
        lines.append("[EVAL] ts_mono=4000 event=sync_pair tag=main ok=0 "
                     "reason=missing_frame missing=2 tol_ns=16666666")
        lines.append("[EVAL] ts_mono=5000 event=sync_pair tag=main ok=0 "
                     "reason=missing_frame missing=3 tol_ns=16666666")
        return lines

    def test_sync_metrics_pairs_and_fails(self):
        from metrics import sync_metrics
        m = sync_metrics(parse_eval_log("\n".join(self._sync_lines())))["main"]
        self.assertEqual(m["pairs"], 3)
        self.assertEqual(m["expected"], 3)
        self.assertEqual(m["pair_rate"], 1.0)
        self.assertEqual(m["diff_p50_ns"], 100000)
        self.assertEqual(m["diff_max_ns"], 100000)
        self.assertEqual(m["frame_id_mismatch"], 0)
        self.assertEqual(m["fails"], {"diff_over_tol": 1, "missing_frame": 2})
        self.assertEqual(m["fail_missing"], {"2": 1, "3": 1})
        self.assertEqual(m["missing_asym"], 1)   # 仅 pipe1 缺帧
        self.assertEqual(m["missing_idle"], 1)   # 两路都空（帧间常态）
        self.assertEqual(m["pair_interval_p50_ns"], 1)

    def test_sync_metrics_mismatch_and_report_warning(self):
        from metrics import sync_metrics
        lines = self._sync_lines()
        lines[6] = lines[6].replace("frame_id_b=1 ", "frame_id_b=2 ")
        m = {"main": sync_metrics(parse_eval_log("\n".join(lines)))["main"]}
        self.assertEqual(m["main"]["frame_id_mismatch"], 1)
        self.assertEqual(m["main"]["frame_id_delta"], {"-1": 1, "0": 2})
        md = render_baseline_report({"sync_pair": m})
        self.assertIn("frame_id 不一致", md)
        self.assertIn("固定帧号偏移", md)

    def _combine_lines(self):
        return [
            "[EVAL] ts_mono=20000000 event=pub_out topic=/image_left_raw ts_frame=1000000",
            "[EVAL] ts_mono=21000000 event=pub_out topic=/image_combine_raw ts_frame=1000000",
            "[EVAL] ts_mono=20000000 event=pub_out topic=/image_left_raw ts_frame=34000000",
            "[EVAL] ts_mono=21000000 event=pub_out topic=/image_combine_raw ts_frame=34000000",
            "[EVAL] ts_mono=22000000 event=combine_out tag=main n=2 order=0,1 "
            "frame_id=5 frame_id_left=5 frame_id_right=5 ts_combine=1000000 "
            "ts_left=1000000 ts_right=1283000 width=1088 height=2560",
            "[EVAL] ts_mono=23000000 event=combine_out tag=main n=2 order=0,1 "
            "frame_id=6 frame_id_left=6 frame_id_right=6 ts_combine=34000000 "
            "ts_left=34000000 ts_right=34283000 width=1088 height=2560",
        ]

    def test_combine_metrics_normal_and_extra_latency(self):
        from metrics import combine_metrics
        csv = "\n".join([
            "topic,seq,stamp_ns,recv_mono_ns,width,height,encoding",
            "/image_left_raw,,1000000,5000000,1088,1280,nv12",
            "/image_combine_raw,,1000000,5500000,1088,2560,nv12",
        ])
        rows, _ = parse_recorder_csv(csv)
        exp = {"combine": {"width": 1088, "height": 2560}}
        m = combine_metrics(parse_eval_log("\n".join(self._combine_lines())),
                            rows, exp)["main"]
        self.assertEqual(m["count"], 2)
        self.assertEqual(m["ts_source"], {"left": 2, "right": 0, "other": 0})
        self.assertEqual(m["ts_right_minus_left_ns"]["p50"], 283000.0)
        self.assertEqual(m["frame_id_match_left"], 2)
        self.assertEqual((m["width_mismatch"], m["height_mismatch"]), (0, 0))
        self.assertEqual(m["order"], {"0,1": 2})
        self.assertEqual(m["extra_latency_ns"]["p50"], 1e6)
        self.assertEqual(m["extra_recv_ns"]["p50"], 500000.0)

    def test_combine_metrics_bad_source_geometry_warns(self):
        from metrics import combine_metrics
        line = ("[EVAL] ts_mono=1 event=combine_out tag=main n=2 order=0,1 "
                "frame_id=7 frame_id_left=7 frame_id_right=8 ts_combine=1283000 "
                "ts_left=1000000 ts_right=1283000 width=1088 height=1280")
        m = {"main": combine_metrics(parse_eval_log(line), None,
                                     {"combine": {"width": 1088, "height": 2560}})["main"]}
        self.assertEqual(m["main"]["ts_source"], {"left": 0, "right": 1, "other": 0})
        self.assertEqual(m["main"]["height_mismatch"], 1)
        md = render_baseline_report({"combine": m})
        self.assertIn("来源不是左目", md)
        self.assertIn("输出几何与期望不符", md)

    def test_combine_metrics_no_left_pub(self):
        """C05（combine=2）：只有拼接 topic，B-11 设备侧应为空。"""
        from metrics import combine_metrics
        line = ("[EVAL] ts_mono=1 event=combine_out tag=main n=2 order=0,1 "
                "frame_id=1 frame_id_left=1 frame_id_right=1 ts_combine=1000000 "
                "ts_left=1000000 ts_right=1283000 width=1088 height=2560")
        m = combine_metrics(parse_eval_log(line))["main"]
        self.assertIsNone(m["extra_latency_ns"])
        self.assertIsNone(m["extra_recv_ns"])

    def test_device_latency_decomposition(self):
        from metrics import device_latency_metrics
        b = MONO_BASE
        hw = b + 2_000_000_000          # 硬件域比单调域快 ~2s（实测现象）
        lines = [
            f"[EVAL] ts_mono={b + 100} event=frame_in pipe=0 stream=main "
            f"ts_frame={b} width=1 height=1",
            f"[EVAL] ts_mono={b + 200} event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns={b + 1_990_000_000} ts_hw_ns={hw} "
            f"exp_us=10000 ts_sys_raw_ns={RT_BASE} ts_frame={b}",
            f"[EVAL] ts_mono={b + 300} event=combine_out tag=main n=2 order=0,1 "
            f"frame_id=1 frame_id_left=1 frame_id_right=1 ts_combine={b} ts_left={b} "
            f"ts_right={b + 1000} width=1 height=2",
            f"[EVAL] ts_mono={b + 500} event=pub_out topic=/image_left_raw "
            f"ts_frame={b} deq_mono={b + 400}",
            f"[EVAL] ts_mono={b + 700} event=pub_out topic=/image_combine_raw "
            f"ts_frame={b} deq_mono={b + 600}",
        ]
        m = device_latency_metrics(parse_eval_log("\n".join(lines)))
        left = m["/image_left_raw"]
        self.assertEqual(left["count"], 1)
        self.assertEqual(left["deq_checked"], 1)
        self.assertEqual(left["ready_to_get_ns"]["p50"], 300.0)
        self.assertEqual(left["publish_cost_ns"]["p50"], 100.0)
        self.assertEqual(left["total_ns"]["p50"], 400.0)
        self.assertEqual(left["t0_source"], "trig")
        self.assertEqual(left["t0_checked"], 1)
        # t0 = trig + exp/2 − offset；offset = hw − 打点 = 2e9 − 200
        self.assertAlmostEqual(left["acq_from_t0_ns"]["p50"], 4_999_900.0, delta=1)
        self.assertAlmostEqual(left["pub_from_t0_ns"]["p50"], 5_000_300.0, delta=1)
        comb = m["/image_combine_raw"]
        self.assertEqual(comb["ready_to_get_ns"]["p50"], 300.0)
        self.assertEqual(comb["t0_source"], "trig")

    def test_device_latency_negative_and_unmatched_ready(self):
        from metrics import device_latency_metrics
        b = MONO_BASE
        lines = [
            f"[EVAL] ts_mono={b + 100} event=frame_in pipe=0 stream=main ts_frame={b}",
            # deq 早于 ready -> 负值
            f"[EVAL] ts_mono={b + 300} event=pub_out topic=/image_left_raw "
            f"ts_frame={b} deq_mono={b + 50}",
            # combine 无 combine_out -> 就绪匹配为 0
            f"[EVAL] ts_mono={b + 10} event=pub_out topic=/image_combine_raw "
            f"ts_frame={b + 5} deq_mono={b + 20}",
            # 未登记 topic
            f"[EVAL] ts_mono={b + 11} event=pub_out topic=/unknown_raw "
            f"ts_frame={b + 6} deq_mono={b + 21}",
        ]
        m = device_latency_metrics(parse_eval_log("\n".join(lines)))
        self.assertLess(m["/image_left_raw"]["ready_to_get_ns"]["min"], 0)
        self.assertEqual(m["/image_combine_raw"]["ready_checked"], 0)
        self.assertTrue(m["/unknown_raw"]["unmapped"])
        md = render_baseline_report({"device_latency": m})
        self.assertIn("负的", md)
        self.assertIn("没有匹配到就绪时刻", md)
        self.assertIn("未登记", md)

    def test_device_latency_single_and_hbmem_mapping(self):
        from metrics import device_latency_metrics
        b = MONO_BASE
        lines = [
            f"[EVAL] ts_mono={b + 100} event=frame_in pipe=0 stream=main ts_frame={b}",
            f"[EVAL] ts_mono={b + 160} event=pub_out topic=/hbmem_img "
            f"ts_frame={b} deq_mono={b + 140}",
            f"[EVAL] ts_mono={b + 170} event=pub_out topic=/image_raw "
            f"ts_frame={b} deq_mono={b + 140}",
        ]
        m = device_latency_metrics(parse_eval_log("\n".join(lines)))
        for topic in ("/hbmem_img", "/image_raw"):
            self.assertFalse(m[topic]["unmapped"])
            self.assertEqual(m[topic]["ready_checked"], 1)
            self.assertEqual(m[topic]["ready_to_get_ns"]["p50"], 40.0)

    def test_device_latency_duplicate_ready_keys(self):
        from metrics import device_latency_metrics
        b = MONO_BASE
        lines = [
            f"[EVAL] ts_mono={b + 100} event=frame_in pipe=0 stream=main ts_frame={b}",
            f"[EVAL] ts_mono={b + 110} event=frame_in pipe=0 stream=main ts_frame={b}",
            f"[EVAL] ts_mono={b + 200} event=pub_out topic=/image_left_raw "
            f"ts_frame={b} deq_mono={b + 150}",
        ]
        m = device_latency_metrics(parse_eval_log("\n".join(lines)))["/image_left_raw"]
        self.assertEqual(m["ready_duplicates"], 1)
        md = render_baseline_report({"device_latency": {"/image_left_raw": m}})
        self.assertIn("关联键有重复", md)

    def test_device_latency_missing_deq_and_t0(self):
        from metrics import device_latency_metrics
        line = (f"[EVAL] ts_mono={MONO_BASE + 50} event=pub_out topic=/image_left_raw "
                f"ts_frame={MONO_BASE}")
        m = device_latency_metrics(parse_eval_log(line))["/image_left_raw"]
        self.assertEqual(m["deq_checked"], 0)
        self.assertIsNone(m["ready_to_get_ns"])
        self.assertEqual(m["t0_source"], "none")
        md = render_baseline_report({"device_latency": {"/image_left_raw": m}})
        self.assertIn("缺 deq_mono", md)

    def test_sync_metrics_absent(self):
        from metrics import sync_metrics
        self.assertIsNone(sync_metrics(parse_eval_log("")))

    def test_sync_metrics_missing_without_frame_in(self):
        """某一路彻底无 frame_in 时，用事件里的 n 判 full mask，不能把单侧失效当 idle。"""
        from metrics import sync_metrics
        text = "\n".join([
            "[EVAL] ts_mono=1 event=sync_pair tag=main ok=0 reason=missing_frame "
            "missing=3 n=2 tol_ns=16666666",
            "[EVAL] ts_mono=2 event=sync_pair tag=main ok=0 reason=missing_frame "
            "missing=2 n=2 tol_ns=16666666",
            "[EVAL] ts_mono=3 event=sync_pair tag=main ok=0 reason=missing_frame "
            "missing=1 n=2 tol_ns=16666666",
        ])
        m = sync_metrics(parse_eval_log(text))["main"]
        self.assertEqual(m["missing_idle"], 1)   # missing=3 两路都空
        self.assertEqual(m["missing_asym"], 2)   # missing=2/1 单侧缺
        self.assertIsNone(m["pair_rate"])

    def test_sync_metrics_zero_rate_warns(self):
        from metrics import sync_metrics
        lines = [f"[EVAL] ts_mono={i} event=frame_in pipe={p} stream=main "
                 f"frame_id={i} ts_frame={MONO_BASE}" for i in range(3) for p in (0, 1)]
        lines.append("[EVAL] ts_mono=9 event=sync_pair tag=main ok=0 "
                     "reason=diff_over_tol missing=0 n=2 diff_ns=20000000 tol_ns=16666666")
        m = {"main": sync_metrics(parse_eval_log("\n".join(lines)))["main"]}
        self.assertEqual(m["main"]["pair_rate"], 0.0)
        md = render_baseline_report({"sync_pair": m})
        self.assertIn("配对率 0.00%", md)

    def test_eval_log_dedupe_nearby_duplicates(self):
        line = (f"[EVAL] ts_mono=100 event=frame_in pipe=0 stream=main "
                f"ts_frame={MONO_BASE}")
        stats = {}
        evs = parse_eval_log("\n".join([line, line, line]), stats)
        self.assertEqual(len(evs), 1)
        self.assertEqual(stats["duplicates"], 2)

    def test_exposure_no_trig_report_warning(self):
        from metrics import exposure_metrics
        text = [
            f"[EVAL] ts_mono=1 event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=0 ts_trig_ns=0 ts_hw_ns=0 exp_us=0 "
            f"ts_sys_raw_ns={RT_BASE} ts_frame={MONO_BASE}",
        ]
        md = render_baseline_report({"exposure": exposure_metrics(self._exposure_events(text))})
        self.assertIn("逐帧曝光不可得", md)

    def test_startup_report_incomplete(self):
        text = "\n".join([
            "[EVAL] ts_mono=1000000000 event=startup stage=node_init pipe=- t0_ns=900000000",
            "[EVAL] ts_mono=1100000000 event=startup stage=backend_init_end pipe=- ret=-1",
        ])
        st = startup_metrics(parse_eval_log(text))
        md = render_baseline_report({"startup": st})
        self.assertIn("不完整", md)


class TestThresholds(unittest.TestCase):
    def setUp(self):
        from metrics import reset_thresholds
        reset_thresholds()

    def tearDown(self):
        from metrics import reset_thresholds
        reset_thresholds()

    def test_default_and_merge(self):
        from thresholds import DEFAULT_THRESHOLDS, KNOWN_KEYS, merge_thresholds
        th = merge_thresholds()
        self.assertEqual(th["pair_rate_min"], 0.999)
        # 覆盖生效
        th2 = merge_thresholds({"pair_rate_min": 0.99})
        self.assertEqual(th2["pair_rate_min"], 0.99)
        # 未知键忽略
        th3 = merge_thresholds({"no_such_key": 1})
        self.assertNotIn("no_such_key", th3)
        self.assertEqual(set(DEFAULT_THRESHOLDS), KNOWN_KEYS)

    def test_pair_rate_threshold_used_in_report(self):
        from metrics import sync_metrics
        from report import render_baseline_report
        lines = [f"[EVAL] ts_mono={i} event=frame_in pipe={p} stream=main "
                 f"frame_id={i} ts_frame={MONO_BASE}" for i in range(3) for p in (0, 1)]
        lines.append("[EVAL] ts_mono=9 event=sync_pair tag=main ok=1 "
                     "frame_id_a=1 frame_id_b=1 ts_a=1 ts_b=2 diff_ns=100000 tol_ns=16666666")
        m = sync_metrics(parse_eval_log("\n".join(lines)))["main"]
        self.assertAlmostEqual(m["pair_rate"], 1 / 3, places=4)
        # 默认 0.999 -> 告警
        md = render_baseline_report({"meta": {}, "sync_pair": {"main": m}})
        self.assertIn("配对率", md)
        # 覆盖到 0.3 -> 不告警
        md2 = render_baseline_report({"meta": {"thresholds": {"pair_rate_min": 0.3}},
                                      "sync_pair": {"main": m}})
        self.assertNotIn("配对率 33.33% 低于", md2)

    def test_stability_ratio_threshold(self):
        from metrics import stability_warnings
        ft = {"pipe0/main": {"interval_max_ns": 4 * 33 * MS}}
        exp = {"framerate": 30.0}
        # 默认 3×：4×周期 -> 告警
        self.assertEqual(len(stability_warnings(ft, exp)), 1)
        # 覆盖到 5×：不告警
        from metrics import set_thresholds
        set_thresholds({"stability_interval_ratio": 5.0})
        self.assertEqual(stability_warnings(ft, exp), [])


class TestImuTiming(unittest.TestCase):
    def _rows(self, n=200, period_us=5000, drop_at=()):
        """n 个 5ms 间隔的 IMU 样本（realtime 域）；drop_at 挖掉若干下标。"""
        lines = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding"]
        for i in range(n):
            if i in drop_at:
                continue
            lines.append(f"/imu_data,,{RT_BASE + i * period_us * 1000},"
                         f"{MONO_BASE + i * period_us * 1000},0,0,imu")
        rows, _ = parse_recorder_csv("\n".join(lines))
        return rows

    def _probe(self, n=199, fail=1, src="event"):
        return (f"[EVAL] ts_mono=1 event=imu_probe src={src} n={n} rd_fail={fail} "
                f"ts_first={RT_BASE} ts_last={RT_BASE + 199 * 5 * MS} "
                f"iv_max_ns={5 * MS} iv_backwards=0")

    def test_imu_timing_basic(self):
        from metrics import imu_timing_metrics
        m = imu_timing_metrics(self._rows(), parse_eval_log(self._probe(n=199, fail=0)))
        self.assertEqual(m["count"], 200)
        self.assertEqual(m["backward"], 0)
        self.assertEqual(m["duplicate"], 0)
        self.assertAlmostEqual(m["rate_hz"], 200.0, places=1)
        self.assertEqual(m["interval_us"]["p50"], 5000)
        self.assertEqual(m["missing_est"], 0)
        self.assertEqual(m["probe_n_sum"], 199)
        self.assertEqual(m["pub_loss"], -1)   # probe 窗口计数与 CSV 差 1 属对账近似
        self.assertEqual(m["domain"], "realtime")

    def test_imu_drop_and_backwards(self):
        from metrics import imu_timing_metrics
        rows = self._rows(n=200, drop_at=(50, 51, 52, 100))
        m = imu_timing_metrics(rows, parse_eval_log(""))
        self.assertEqual(m["count"], 196)
        self.assertEqual(m["missing_est"], 4)
        self.assertAlmostEqual(m["missing_rate"], 0.02, places=3)
        self.assertGreater(m["interval_us"]["max"], 15000)   # 15ms 大缺口
        # 实测周期反推：3+1=4 样本缺口
        self.assertEqual(m["missing_gap"], 4)
        self.assertEqual(len(m["gap_events"]), 2)

    def test_imu_odr_offset_gap_detection(self):
        """ODR 偏 +1% 时标称反推漏报，实测周期反推应抓到大缺口。"""
        from metrics import imu_timing_metrics
        rows = self._rows(n=1000, period_us=4950, drop_at=(500, 501, 502, 503, 504, 505, 506, 507))
        m = imu_timing_metrics(rows, parse_eval_log(""))
        # 标称口径：期望=跨度×200 ≈ 985 < 实收 992 → missing=0（漏报）
        self.assertEqual(m["missing_est"], 0)
        # 实测口径：40ms 缺口 ≈ 8 样本
        self.assertEqual(m["missing_gap"], 8)

    def test_imu_report_warnings(self):
        from metrics import imu_timing_metrics
        from report import render_baseline_report
        rows = self._rows(n=100, period_us=6000)   # 166Hz < 198
        m = imu_timing_metrics(rows, parse_eval_log(self._probe(n=99, fail=2)))
        md = render_baseline_report({"meta": {}, "imu": m})
        self.assertIn("频率比率", md)
        self.assertIn("读取失败 2", md)

    def test_imu_absent(self):
        from metrics import imu_timing_metrics
        csv = "topic,seq,stamp_ns,recv_mono_ns,width,height,encoding\n" \
              f"/t/img,,{MONO_BASE},{MONO_BASE},1,1,nv12\n"
        rows, _ = parse_recorder_csv(csv)
        self.assertIsNone(imu_timing_metrics(rows, parse_eval_log("")))


class TestImuValidity(unittest.TestCase):
    def _stat(self, n=200, ax=0.1, ay=-0.2, az=9.8, gx=0.001, gy=-0.002,
              gz=0.0005, sax=0.04, sgx=0.001, nan=0, inf=0, cal=1):
        return (f"[EVAL] ts_mono=1 event=imu_stat cal={cal} n={n} nan={nan} inf={inf} "
                f"ax={ax} ay={ay} az={az} gx={gx} gy={gy} gz={gz} "
                f"sax={sax} say={sax} saz={sax} sgx={sgx} sgy={sgx} sgz={sgx}")

    def test_validity_normal(self):
        from metrics import imu_validity_metrics
        m = imu_validity_metrics(parse_eval_log("\n".join([
            self._stat(n=200), self._stat(n=100, ax=0.2)])))
        self.assertEqual(m["windows"], 2)
        self.assertEqual(m["samples"], 300)
        self.assertEqual(m["nan"], 0)
        # 加权均值：ax = (0.1*200 + 0.2*100)/300
        self.assertAlmostEqual(m["axes"]["ax"], 0.13333, places=4)
        self.assertAlmostEqual(m["accel_norm"], 9.8, places=2)
        self.assertLess(m["gyro_bias_max"], 0.003)
        self.assertEqual(m["cal"], [1])

    def test_validity_bias_and_norm_warnings(self):
        from metrics import imu_validity_metrics
        from report import render_baseline_report
        m = imu_validity_metrics(parse_eval_log(self._stat(gx=0.05, az=8.0)))
        self.assertGreater(m["gyro_bias_max"], 0.03)
        self.assertLess(m["accel_norm"], 9.3)
        md = render_baseline_report({"meta": {}, "imu_validity": m})
        self.assertIn("零偏 0.05", md)
        self.assertIn("accel 模长", md)

    def test_validity_nan_inf_cal_flip(self):
        from metrics import imu_validity_metrics
        from report import render_baseline_report
        m = imu_validity_metrics(parse_eval_log("\n".join([
            self._stat(nan=3), self._stat(cal=-1, n=100)])))
        self.assertEqual(m["nan"], 3)
        self.assertEqual(m["cal"], [-1, 1])
        md = render_baseline_report({"meta": {}, "imu_validity": m})
        self.assertIn("NaN/Inf 共 3", md)
        self.assertIn("翻转", md)

    def test_validity_absent(self):
        from metrics import imu_validity_metrics
        self.assertIsNone(imu_validity_metrics(parse_eval_log("")))


class TestCamImu(unittest.TestCase):
    def _data(self, n_frames=3, imu_per_frame=6, t0=MONO_BASE):
        """每帧 33.33ms、帧间 6 个 IMU 样本（5.556ms 间隔，200Hz）。"""
        lines = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding"]
        frame_t0 = []
        hw_off = 2_000_000_000   # 硬件域比 mono 快 2s
        ev = []
        for f in range(n_frames):
            t = t0 + f * 33_333_333
            frame_t0.append(t)
            for s in range(imu_per_frame):
                rt = (t + s * 5_555_556) + CLOCK_OFFSET
                lines.append(f"/imu_data,,{rt},{t},0,0,imu")
            ev.append(f"[EVAL] ts_mono={t + 100} event=exposure pipe=0 stream=main "
                      f"frame_id={f} stamp_src=sensor trig_valid=1 "
                      f"ts_trig_ns={t + hw_off} ts_hw_ns={t + hw_off + 5000000} "
                      f"exp_us=10000 ts_sys_raw_ns={RT_BASE} ts_frame={t}")
        rows, _ = parse_recorder_csv("\n".join(lines))
        return rows, parse_eval_log("\n".join(ev))

    def test_pair_and_cover(self):
        from metrics import cam_imu_metrics
        rows, ev = self._data()
        m = cam_imu_metrics(rows, ev, CLOCK_OFFSET)
        self.assertEqual(m["frames"], 3)
        self.assertEqual(m["imu_samples"], 18)
        # t0 = trig + 5ms − off(2s+~偏移中位) → 样本以 5.556ms 网格覆盖，
        # 最近邻 ≤ 半周期 2.78ms
        self.assertLessEqual(m["nearest_us"]["max"], 2800)
        # 覆盖率：窗口内 6/6.67
        self.assertGreater(m["cover_ratio"], 0.85)

    def test_absent(self):
        from metrics import cam_imu_metrics
        csv = "topic,seq,stamp_ns,recv_mono_ns,width,height,encoding\n" \
              f"/t/img,,{MONO_BASE},{MONO_BASE},1,1,nv12\n"
        rows, _ = parse_recorder_csv(csv)
        self.assertIsNone(cam_imu_metrics(rows, parse_eval_log(""), 0))


class TestRecvMetrics(unittest.TestCase):
    def _rows(self, n_pub=10, n_recv=9, n_info=9):
        """n_pub 帧 pub，CSV 收到后 n_recv 帧（丢第 5 帧）；info 独立写 n_info 帧。"""
        lines = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding"]
        for i in range(n_pub):
            if i == 4 and n_recv < n_pub:
                continue
            s = MONO_BASE + i * 33 * MS
            lines.append(f"/image_left_raw,,{s},{s + MS},10,10,nv12")
        for i in range(n_info):
            s = MONO_BASE + i * 33 * MS
            lines.append(f"/image_left_raw/camera_info,,{s},{s + MS},10,10,camera_info")
        rows, _ = parse_recorder_csv("\n".join(lines))
        ev = [f"[EVAL] ts_mono={MONO_BASE + i} event=pub_out topic=/image_left_raw "
              f"ts_frame={MONO_BASE + i * 33 * MS}" for i in range(n_pub)]
        return rows, parse_eval_log("\n".join(ev))

    def test_recv_drop_and_e10(self):
        from metrics import recv_metrics
        rows, ev = self._rows()
        m = recv_metrics(rows, ev)["topics"]["/image_left_raw"]
        self.assertEqual(m["pub_count"], 10)
        self.assertEqual(m["csv_rows"], 9)
        self.assertEqual(m["recv_drop"], 1)
        self.assertAlmostEqual(m["recv_drop_rate"], 0.1, places=3)
        self.assertEqual(m["backward"], 0)
        ip = recv_metrics(rows, ev)["info_pair"]["/image_left_raw/camera_info"]
        # info 覆盖 i=0..8 共 9 帧，其中 i=4 的图像帧被丢 → matched=8、多余 1
        self.assertEqual(ip["matched"], 8)
        self.assertEqual(ip["extra_info"], 1)
        self.assertAlmostEqual(ip["pair_rate"], 8 / 9, places=3)

    def test_pre_subscription_window(self):
        from metrics import recv_metrics
        rows, ev = self._rows(n_pub=12, n_recv=12, n_info=0)
        # CSV 首帧是第 3 帧（模拟订阅晚建立）：把前 2 帧 pub 的 stamp 前移
        lines = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding"]
        for i in range(2, 12):
            s = MONO_BASE + i * 33 * MS
            lines.append(f"/image_left_raw,,{s},{s + MS},10,10,nv12")
        rows, _ = parse_recorder_csv("\n".join(lines))
        m = recv_metrics(rows, ev)["topics"]["/image_left_raw"]
        self.assertEqual(m["pre_subscription"], 2)
        self.assertEqual(m["recv_drop"], 0)

    def test_backward_and_warning(self):
        from metrics import recv_metrics
        from report import render_baseline_report
        rows, ev = self._rows(n_pub=5, n_recv=5, n_info=0)
        csv = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding"]
        stamps = [MONO_BASE, MONO_BASE + MS, MONO_BASE, MONO_BASE + 2 * MS,
                  MONO_BASE + 3 * MS]   # 第 3 行倒退+重复
        for s in stamps:
            csv.append(f"/image_left_raw,,{s},{s},10,10,nv12")
        rows, _ = parse_recorder_csv("\n".join(csv))
        m = {"recv": recv_metrics(rows, ev)}
        self.assertEqual(m["recv"]["topics"]["/image_left_raw"]["backward"], 1)
        self.assertEqual(m["recv"]["topics"]["/image_left_raw"]["duplicate"], 1)
        md = render_baseline_report({"meta": {}, **m})
        self.assertIn("倒退 1 次", md)

    def test_imu_recv(self):
        from metrics import recv_metrics
        csv = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding",
               f"/image_left_raw,,{MONO_BASE},{MONO_BASE},1,1,nv12"]
        for i in range(200):
            csv.append(f"/imu_data,,{RT_BASE + i * 5 * MS},{MONO_BASE + i},0,0,imu")
        rows, _ = parse_recorder_csv("\n".join(csv))
        ev = (f"[EVAL] ts_mono=1 event=pub_out topic=/image_left_raw "
              f"ts_frame={MONO_BASE}")
        m = recv_metrics(rows, parse_eval_log(ev))
        self.assertEqual(m["imu"]["csv_rows"], 200)
        self.assertEqual(m["imu"]["backward"], 0)


class TestRecvLatency(unittest.TestCase):
    def test_dds_and_e2e(self):
        from metrics import recv_latency_metrics
        t = MONO_BASE
        hw = t + 2_000_000_000
        ev = [
            f"[EVAL] ts_mono={t + 100} event=exposure pipe=0 stream=main frame_id=1 "
            f"stamp_src=sensor trig_valid=1 ts_trig_ns={t + 1_990_000_000} ts_hw_ns={hw} "
            f"exp_us=10000 ts_sys_raw_ns={RT_BASE} ts_frame={t}",
            f"[EVAL] ts_mono={t + 1000} event=pub_out topic=/image_left_raw "
            f"ts_frame={t} deq_mono={t + 900}",
        ]
        csv = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding",
               f"/image_left_raw,,{t},{t + 2000},10,10,nv12"]
        rows, _ = parse_recorder_csv("\n".join(csv))
        m = recv_latency_metrics(rows, parse_eval_log("\n".join(ev)))
        rec = m["/image_left_raw"]
        # DDS = recv − pub = 2000 − 1000 = 1000µs
        self.assertEqual(rec["dds_ns"]["p50"], 1000.0)
        # t0 = trig + 5ms − (hw − 打点=2e9−100) = t − 4_999_900
        # e2e = recv − t0 = 2000 + 4_999_900
        self.assertAlmostEqual(rec["e2e_from_t0_ns"]["p50"], 5_001_900.0, delta=1)
        self.assertEqual(rec["t0_source"], "trig")

    def test_no_pub_no_output(self):
        from metrics import recv_latency_metrics
        csv = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding",
               f"/t/img,,{MONO_BASE},{MONO_BASE},1,1,nv12"]
        rows, _ = parse_recorder_csv("\n".join(csv))
        self.assertIsNone(recv_latency_metrics(rows, parse_eval_log("")))


class TestBandwidth(unittest.TestCase):
    def test_three_sources(self):
        from metrics import bandwidth_metrics
        csv = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding"]
        for i in range(30):   # 1s 内 30 帧 2.09MB → 62.7MB/s
            s = MONO_BASE + i * 33 * MS
            csv.append(f"/image_left_raw,,{s},{s + MS},1088,1280,nv12")
        rows, _ = parse_recorder_csv("\n".join(csv))
        bw_txt = ("Subscribed to [/image_left_raw]\n"
                  "29.60 MB/s from 13 messages\n\tMessage size mean: 2.09 MB\n"
                  "31.85 MB/s from 100 messages\n\tMessage size mean: 2.09 MB\n")
        m = bandwidth_metrics(rows, {"framerate": 30.0},
                              {"/image_left_raw": bw_txt})["/image_left_raw"]
        self.assertEqual(m["bytes_per_frame"], 2088960)
        self.assertEqual(m["pub_bw_mb_s"], 62.67)
        self.assertAlmostEqual(m["csv_bw_mb_s"], 62.67, delta=1.0)
        self.assertEqual(m["bw_tool_mb_s"], 31.85)   # 取最后一条收敛值

    def test_bgr8(self):
        from metrics import bandwidth_metrics
        csv = ["topic,seq,stamp_ns,recv_mono_ns,width,height,encoding",
               f"/t/img,,{MONO_BASE},{MONO_BASE},100,100,bgr8"]
        rows, _ = parse_recorder_csv("\n".join(csv))
        m = bandwidth_metrics(rows)["/t/img"]
        self.assertEqual(m["bytes_per_frame"], 30000)   # 100×100×3


class TestSuiteReport(unittest.TestCase):
    def _run(self, result="OK", with_pair=False, with_res=True, unstable=False):
        metrics = {}
        if with_res:
            metrics["resolution"] = {"all_match": True}
        if unstable:
            metrics["stability_warnings"] = [{"stream": "pipe0/main"}]
        if with_pair:
            metrics["meta"] = {"thresholds": {"pair_rate_min": 0.999,
                                              "pair_diff_p99_ms": 1.0}}
            metrics["frame_timing"] = {"pipe0/main": {"fps_mean": 30.0}}
            metrics["sync_pair"] = {"main": {"pair_rate": 0.5, "diff_p50_ns": 3000000,
                                             "diff_p99_ns": 3100000,
                                             "frame_id_mismatch": 0}}
        return {"case": "C04", "outdir": "/tmp/x", "result": result,
                "elapsed_s": 180, "metrics": metrics}

    def test_verdicts(self):
        from suite_report import summarize_run
        self.assertEqual(summarize_run(self._run())["verdict"], "PASS")
        self.assertEqual(summarize_run(self._run(unstable=True))["verdict"], "WARN")
        self.assertEqual(summarize_run(self._run(result="FAIL"))["verdict"], "FAIL")
        self.assertEqual(summarize_run(self._run(with_pair=True))["verdict"], "WARN")

    def test_summary_fields(self):
        from suite_report import summarize_run
        s = summarize_run(self._run(with_pair=True))
        self.assertEqual(s["fps"], {"pipe0/main": 30.0})
        self.assertEqual(s["pair"]["rate"], 0.5)
        self.assertEqual(s["pair"]["diff_p50_ms"], 3.0)
        self.assertIn("pair_rate_low", s["warnings"])
        self.assertIn("pair_diff_p99", s["warnings"])

    def test_render(self):
        from suite_report import render_suite_report, summarize_run
        suite = {"suite": "smoke",
                 "runs": [summarize_run(self._run()),
                          summarize_run(self._run(with_pair=True))]}
        md = render_suite_report(suite)
        self.assertIn("评测 suite：smoke", md)
        self.assertIn("**WARN**", md)
        self.assertIn("pair_rate_low", md)


if __name__ == "__main__":
    unittest.main()
