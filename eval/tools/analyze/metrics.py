#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""指标统计：域归一、频率/间隔分位、采集丢帧反推、配对、延迟（曝光中点 t0）、IMU。

口径术语见工作区根 CONTEXT.md：
- 域归一：IMU(REALTIME) 与图像(monotonic) 归一到 monotonic 轴（ADR-0001，评测侧做）
- 曝光中点：延迟 t0 = 帧ts − 曝光/2；exp_valid=0 的帧单独分组（帧 ts 口径）
- 采集丢帧：frame_in 的 frame_id 连续性反推；接收丢帧：订阅计数 vs 采集计数
"""
from collections import Counter, defaultdict
from typing import Dict, List, Optional

from parsers import RecRow, parse_eval_log  # 同目录导入（脚本/测试均从此目录起）

# 曝光匹配容差：realtime 域下图像 stamp 与内核 ts_frame 相差帧管线延迟+时钟漂移，
# 用最近邻匹配（mono 域下精确命中，diff=0）
EXP_MATCH_TOL_NS = 50 * 1000 * 1000
# 域自动识别阈值：realtime 域 stamp ~1.7e18，monotonic 域 <1e17
REALTIME_DETECT_NS = 10 ** 17


def percentile(sorted_vals: List[float], p: float) -> Optional[float]:
    """线性插值分位数；输入须已升序。"""
    if not sorted_vals:
        return None
    if len(sorted_vals) == 1:
        return float(sorted_vals[0])
    idx = p / 100.0 * (len(sorted_vals) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = idx - lo
    return float(sorted_vals[lo]) + (float(sorted_vals[hi]) - float(sorted_vals[lo])) * frac


def _modal_step(ids: List[int]) -> int:
    """frame_id 的常见步长（X5 VSE 分频时可能为 2，S100 通常为 1）。"""
    deltas = [b - a for a, b in zip(ids, ids[1:]) if b >= a]
    if not deltas:
        return 1
    step = Counter(deltas).most_common(1)[0][0]
    return step if step > 0 else 1


def _frame_id_missing(ids: List[int]) -> Optional[int]:
    """超过常见步长的 frame_id 缺口（仅归因；不是丢帧主口径）。"""
    if len(ids) < 2:
        return None
    step = _modal_step(ids)
    deltas = [b - a for a, b in zip(ids, ids[1:]) if b >= a]
    return sum(max(0, d - step) for d in deltas)


def expected_from_launch_args(meta_text: str) -> Optional[Dict]:
    """从运行目录 meta.txt 的 launch_args 解析分辨率期望值。"""
    args = None
    for line in meta_text.splitlines():
        if line.startswith("launch_args="):
            args = line.split("=", 1)[1]
            break
    if not args:
        return None
    kv = {}
    for tok in args.split():
        if ":=" in tok:
            k, v = tok.split(":=", 1)
            kv[k] = v
    try:
        main_w = int(kv["image_width"])
        main_h = int(kv["image_height"])
    except (KeyError, ValueError):
        return None
    enc = kv.get("out_format", "nv12")
    try:
        fps = float(kv.get("framerate", "0") or 0)
    except ValueError:
        fps = 0.0
    expected = {
        "main": {"width": main_w, "height": main_h, "encoding": enc},
        "info": {"width": main_w, "height": main_h, "encoding": "camera_info"},
        "framerate": fps,
    }
    sub_on = kv.get("sub_stream_enable", "False") not in ("False", "false", "0")
    if sub_on and "sub_image_width" in kv and "sub_image_height" in kv:
        sub_w = int(kv["sub_image_width"])
        sub_h = int(kv["sub_image_height"])
        expected["sub"] = {"width": sub_w, "height": sub_h, "encoding": enc}
        expected["info_sub"] = {"width": sub_w, "height": sub_h, "encoding": "camera_info"}
    if kv.get("device_mode") == "dual" and kv.get("dual_combine") in ("1", "2"):
        # X5 combine 为纵向拼接：宽不变，高 ×2
        expected["combine"] = {"width": main_w, "height": main_h * 2, "encoding": enc}
        if "sub" in expected:
            expected["sub_combine"] = {
                "width": expected["sub"]["width"],
                "height": expected["sub"]["height"] * 2,
                "encoding": enc,
            }
    return expected


def _expected_for_topic(topic: str, expected: Optional[Dict]) -> Optional[Dict]:
    if not expected:
        return None
    if "sub" in topic:
        if "camera_info" in topic:
            return expected.get("info_sub") or expected.get("info")
        if "combine" in topic:
            return expected.get("sub_combine") or expected.get("sub")
        return expected.get("sub") or expected.get("main")
    if "combine" in topic:
        if "camera_info" in topic:
            return expected.get("info")
        return expected.get("combine")
    if "camera_info" in topic:
        return expected.get("info")
    return expected.get("main")


def resolution_metrics(rows: List[RecRow], eval_events: List[Dict],
                       expected: Optional[Dict],
                       expected_topics: Optional[List[str]] = None) -> Optional[Dict]:
    """分辨率准确性：设备侧 frame_in 宽高 + 接收侧 recorder 元数据 vs 配置。"""
    device_groups = defaultdict(list)
    for e in eval_events:
        if e["event"] != "frame_in":
            continue
        kv = e["kv"]
        w, h = kv.get("width"), kv.get("height")
        if isinstance(w, int) and isinstance(h, int):
            device_groups[(kv.get("pipe"), kv.get("stream", "?"))].append((w, h))
    if not device_groups and not rows:
        return None

    device = {}
    for key, sizes in device_groups.items():
        stream = key[1]
        exp = None
        if expected:
            exp = expected.get(stream) or expected.get("main")
        cnt = Counter(sizes)
        mismatch = None
        if exp:
            mismatch = sum(n for (w, h), n in cnt.items()
                           if (w, h) != (exp["width"], exp["height"]))
        device[f"pipe{key[0]}/{stream}"] = {
            "expected": "%dx%d" % (exp["width"], exp["height"]) if exp else None,
            "observed": {"%dx%d" % (w, h): n for (w, h), n in sorted(cnt.items())},
            "mismatch": mismatch,
        }

    by_topic = defaultdict(list)
    for r in rows:
        if r.encoding == "imu":
            continue
        by_topic[r.topic].append((r.width, r.height, r.encoding))
    published = {}
    for topic, vals in by_topic.items():
        exp = _expected_for_topic(topic, expected)
        cnt = Counter(vals)
        mismatch = None
        if exp:
            mismatch = sum(
                n for (w, h, enc), n in cnt.items()
                if (w, h, enc) != (exp["width"], exp["height"], exp["encoding"]))
        published[topic] = {
            "expected": "%dx%d %s" % (exp["width"], exp["height"], exp["encoding"])
            if exp else None,
            "observed": {"%dx%d %s" % (w, h, enc): n
                         for (w, h, enc), n in sorted(cnt.items())},
            "mismatch": mismatch,
        }

    dev_mis = sum(v["mismatch"] or 0 for v in device.values())
    pub_mis = sum(v["mismatch"] or 0 for v in published.values())
    missing_topics = [t for t in (expected_topics or []) if t not in by_topic]
    device_covered = bool(device_groups)
    return {
        "expected": expected,
        "device": device,
        "published": published,
        "device_mismatch": dev_mis,
        "published_mismatch": pub_mis,
        "missing_topics": missing_topics,
        "device_covered": device_covered,
        "all_match": (expected is not None and device_covered and
                      dev_mis == 0 and pub_mis == 0 and not missing_topics),
        "expected_available": expected is not None,
    }


def _interval_stats(values_ns: List[int]):
    """相邻差 -> {count, mean, p50, p95, p99, max, min}（ns）"""
    out = {"count": len(values_ns)}
    if len(values_ns) < 2:
        return out
    diffs = sorted(b - a for a, b in zip(values_ns, values_ns[1:]))
    out.update({
        "mean": sum(diffs) / len(diffs),
        "p50": percentile(diffs, 50),
        "p95": percentile(diffs, 95),
        "p99": percentile(diffs, 99),
        "max": diffs[-1],
        "min": diffs[0],
    })
    return out


def topic_metrics(rows: List[RecRow]) -> Dict[str, Dict]:
    """按 topic：接收频率/间隔分位/时间戳单调性/重复 stamp。"""
    by_topic = defaultdict(list)
    for r in rows:
        by_topic[r.topic].append(r)
    out = {}
    for topic, rs in by_topic.items():
        rs.sort(key=lambda r: r.recv_mono_ns)
        recvs = [r.recv_mono_ns for r in rs]
        stamps = [r.stamp_ns for r in rs]
        mono_violations = sum(1 for a, b in zip(stamps, stamps[1:]) if b < a)
        dup_stamps = len(stamps) - len(set(stamps))
        duration_s = (recvs[-1] - recvs[0]) / 1e9 if len(recvs) > 1 else 0.0
        # 无偏帧率估计：(N-1)/时长（时长只覆盖 N-1 个间隔）
        m = {
            "count": len(rs),
            "duration_s": round(duration_s, 3),
            "fps_mean": round((len(rs) - 1) / duration_s, 3) if len(rs) > 1 else None,
            "recv_interval_ns": _interval_stats(recvs),
            "stamp_monotonic_violations": mono_violations,
            "stamp_duplicates": dup_stamps,
        }
        # hbmem 序号连续性（接收丢帧直接判定）
        seqs = [int(r.seq) for r in rs if r.seq.isdigit()]
        if seqs and len(seqs) == len(rs):
            gaps = (seqs[-1] - seqs[0] + 1) - len(seqs)
            m["seq_gaps"] = max(gaps, 0)
        out[topic] = m
    return out


def frame_in_metrics(eval_events: List[Dict]) -> Dict:
    """按 pipe/stream：采集帧数、frame_id 反推丢帧、曝光有效率。"""
    groups = defaultdict(list)
    for e in eval_events:
        if e["event"] != "frame_in":
            continue
        key = (e["kv"].get("pipe"), e["kv"].get("stream", "?"))
        groups[key].append(e)
    out = {}
    for key, evs in groups.items():
        ids = [e["kv"]["frame_id"] for e in evs if "frame_id" in e["kv"]]
        step = _modal_step(ids) if len(ids) > 1 else 1
        dropped = _frame_id_missing(ids)
        has_exp = any("exp_valid" in e["kv"] for e in evs)
        valid = [e["kv"] for e in evs if e["kv"].get("exp_valid") == 1]
        exp_us = [k["exp_us"] for k in valid if "exp_us" in k]
        out[f"pipe{key[0]}/{key[1]}"] = {
            "count": len(evs),
            "frame_id_step": step,
            "dropped_by_frame_id": dropped,
            "drop_rate": round(dropped / (dropped + len(set(ids))), 6)
            if dropped is not None and step == 1 and (dropped + len(set(ids))) > 0 else None,
            "exp_valid_ratio": round(len(valid) / len(evs), 4) if has_exp and evs else None,
            "exp_us_mean": round(sum(exp_us) / len(exp_us)) if exp_us else None,
        }
    return out


def frame_timing_metrics(eval_events: List[Dict]) -> Dict:
    """设备侧帧时序：按 pipe/stream 统计实际 fps、间隔分位、单调性、重复率。

    只消费 frame_in 的 pipe/stream/ts_frame（02-frame-timing 最小字段集），
    不依赖 frame_id/exp 等后续轮次字段。
    """
    groups = defaultdict(list)  # key -> [(event_index, ts_frame)]
    for idx, e in enumerate(eval_events):
        if e["event"] != "frame_in":
            continue
        kv = e["kv"]
        ts = kv.get("ts_frame")
        if not isinstance(ts, int):
            continue
        key = (kv.get("pipe"), kv.get("stream", "?"))
        groups[key].append((idx, ts))
    out = {}
    for key, pairs in groups.items():
        # 按日志顺序判单调/重复；按时间戳排序算间隔分位
        seq = [ts for _, ts in pairs]
        mono_violations = sum(1 for a, b in zip(seq, seq[1:]) if b < a)
        dup = len(seq) - len(set(seq))
        sorted_ts = sorted(seq)
        intervals = sorted(b - a for a, b in zip(sorted_ts, sorted_ts[1:]))
        duration_s = (sorted_ts[-1] - sorted_ts[0]) / 1e9 if len(sorted_ts) > 1 else 0.0
        m = {
            "count": len(seq),
            "duration_s": round(duration_s, 3),
            "fps_mean": round((len(seq) - 1) / duration_s, 3) if duration_s > 0 else None,
            "interval_p50_ns": percentile(intervals, 50),
            "interval_p95_ns": percentile(intervals, 95),
            "interval_p99_ns": percentile(intervals, 99),
            "interval_max_ns": intervals[-1] if intervals else None,
            "interval_min_ns": intervals[0] if intervals else None,
            "stamp_monotonic_violations": mono_violations,
            "stamp_duplicates": dup,
        }
        out[f"pipe{key[0]}/{key[1]}"] = m
    return out


def pub_out_metrics(eval_events: List[Dict]) -> Dict[str, int]:
    out = defaultdict(int)
    for e in eval_events:
        if e["event"] == "pub_out" and "topic" in e["kv"]:
            out[str(e["kv"]["topic"])] += 1
    return dict(out)


def _exp_pairs_sorted(eval_events: List[Dict]):
    """[(ts_frame, exp_us)]（升序），供延迟 t0=曝光中点使用。

    优先 `exposure` 事件（走与 exposure_metrics 相同的可信闸门），
    缺失时回退 `frame_in` 的 `exp_valid/exp_us`（S100 既有产物）。
    """
    groups = defaultdict(list)
    for e in eval_events:
        if e["event"] != "exposure":
            continue
        kv = e["kv"]
        groups[(kv.get("pipe"), str(kv.get("stream")))].append(kv)
    pairs = {}
    for rows in groups.values():
        pairs.update(dict(_exposure_gate(rows)["usable"]))
    for e in eval_events:
        if e["event"] != "frame_in":
            continue
        kv = e["kv"]
        ts, exp = kv.get("ts_frame"), kv.get("exp_us")
        if kv.get("exp_valid") == 1 and isinstance(ts, int) and isinstance(exp, int) and exp > 0:
            pairs.setdefault(ts, exp)   # exposure 优先
    return sorted(pairs.items())


def _lookup_exp(stamp_ns: int, pairs, tol_ns: int) -> Optional[int]:
    """最近邻曝光匹配（容差 tol_ns）"""
    import bisect
    ts_list = [p[0] for p in pairs]
    i = bisect.bisect_left(ts_list, stamp_ns)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(pairs):
            d = abs(pairs[j][0] - stamp_ns)
            if d <= tol_ns and (best is None or d < best[0]):
                best = (d, pairs[j][1])
    return best[1] if best else None


def latency_metrics(rows: List[RecRow], eval_events: List[Dict],
                    image_domain: str = "mono", clock_offset_ns: int = 0) -> Dict:
    """端到端延迟（接收端视角）：t0=曝光中点（帧ts−exp/2），无曝光帧单独分组。

    image_domain: 图像 stamp 的时钟域（frame_ts_type=sensor -> mono；realtime -> realtime）
    域归一：imu 行 stamp 恒为 realtime，减 offset 到 mono 轴；图像行按 image_domain
    归一到 mono 后与 frame_in 的 ts_frame（内核 monotonic ts）做最近邻曝光匹配。
    """
    exp_pairs = _exp_pairs_sorted(eval_events)
    # 匹配容差：不超过半帧间隔（避免丢帧/无曝光帧"借"到邻帧曝光），
    # 且不超过帧管线延迟上界（覆盖 realtime 域 stamp 与 ts_frame 的偏差）
    tol_ns = EXP_MATCH_TOL_NS
    if len(exp_pairs) >= 2:
        gaps = [b[0] - a[0] for a, b in zip(exp_pairs, exp_pairs[1:])]
        tol_ns = min(EXP_MATCH_TOL_NS, max(gaps) // 2)
    by_topic = defaultdict(list)
    for r in rows:
        if r.encoding in ("imu", "camera_info"):
            continue
        stamp = r.stamp_ns if image_domain == "mono" else r.stamp_ns - clock_offset_ns
        by_topic[r.topic].append((stamp, r.recv_mono_ns))
    out = {}
    for topic, pairs in by_topic.items():
        lat_mid, lat_ts = [], []
        for stamp, recv in pairs:
            lat = recv - stamp
            exp_us = _lookup_exp(stamp, exp_pairs, tol_ns)
            if exp_us is not None:
                # sensor: t0 = 帧ts − exp/2；realtime: t0 = 帧ts + exp/2（方案 6.1）
                half = exp_us * 1000 // 2
                lat_mid.append(lat + half if image_domain != "realtime" else lat - half)
            else:
                lat_ts.append(lat)
        def _pack(vals):
            if not vals:
                return None
            s = sorted(vals)
            return {"count": len(s),
                    "mean_ms": round(sum(s) / len(s) / 1e6, 3),
                    "p50_ms": round(percentile(s, 50) / 1e6, 3),
                    "p95_ms": round(percentile(s, 95) / 1e6, 3),
                    "p99_ms": round(percentile(s, 99) / 1e6, 3),
                    "max_ms": round(s[-1] / 1e6, 3)}
        out[topic] = {"exp_midpoint": _pack(lat_mid), "frame_ts": _pack(lat_ts)}
    return out


def pairing_metrics(rows: List[RecRow], topic_a: str, topic_b: str,
                    image_domain: str = "mono", clock_offset_ns: int = 0,
                    tol_ns: int = 1000000) -> Optional[Dict]:
    """两图像流同源配对：stamp 相等（容差 tol）视为同帧。"""
    def stamps_of(topic):
        return [r.stamp_ns if image_domain == "mono" else r.stamp_ns - clock_offset_ns
                for r in rows if r.topic == topic]
    a, b = stamps_of(topic_a), stamps_of(topic_b)
    if not a or not b:
        return None
    b_sorted = sorted(b)
    paired, diffs = 0, []
    import bisect
    for ts in a:
        i = bisect.bisect_left(b_sorted, ts)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(b_sorted):
                d = abs(b_sorted[j] - ts)
                if best is None or d < best:
                    best = d
        if best is not None and best <= tol_ns:
            paired += 1
            diffs.append(best)
    diffs.sort()
    return {
        "topic_a": topic_a, "topic_b": topic_b,
        "count_a": len(a), "count_b": len(b),
        "paired": paired,
        "pair_rate": round(paired / max(len(a), len(b)), 4),
        "diff_p99_ns": percentile(diffs, 99) if diffs else None,
        "diff_max_ns": diffs[-1] if diffs else None,
    }


def imu_sync_metrics(rows: List[RecRow], image_topic: str,
                     image_domain: str = "mono", clock_offset_ns: int = 0) -> Optional[Dict]:
    """T-06 最近邻：每帧图像与最近 IMU 样本（双域归一后）的时间差分布。"""
    img = [r.stamp_ns if image_domain == "mono" else r.stamp_ns - clock_offset_ns
           for r in rows if r.topic == image_topic]
    imu = [r.stamp_ns - clock_offset_ns for r in rows if r.encoding == "imu"]
    if not img or not imu:
        return None
    imu.sort()
    import bisect
    diffs = []
    for ts in img:
        i = bisect.bisect_left(imu, ts)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(imu):
                d = abs(imu[j] - ts)
                if best is None or d < best:
                    best = d
        if best is not None:
            diffs.append(best)
    diffs.sort()
    return {
        "image_topic": image_topic, "imu_count": len(imu), "image_count": len(img),
        "nearest_p50_ns": percentile(diffs, 50),
        "nearest_p99_ns": percentile(diffs, 99),
        "nearest_max_ns": diffs[-1] if diffs else None,
    }


def detect_image_domain(rows: List[RecRow]) -> str:
    """域自动识别：图像 stamp 中位数超过 realtime 阈值则判为 realtime。"""
    stamps = sorted(r.stamp_ns for r in rows if r.encoding not in ("imu", "camera_info"))
    if not stamps:
        return "mono"
    return "realtime" if stamps[len(stamps) // 2] > REALTIME_DETECT_NS else "mono"


def drop_metrics(eval_events: List[Dict]) -> Optional[Dict]:
    """设备侧丢帧：直接丢弃点计数 + frame_id 序号缺口（仅归因）。

    - frame_in：每 pipe/stream 收到帧数与 frame_id 缺口（归因，不计入主丢帧率）
    - frame_drop：getframe_fail / select_timeout / select_error 逐次事件
    - queue_stat：recycle / empty_null / fq_full / fq_expired 的累计计数
    主丢帧率 = 直接计数之和 / (收到帧数 + 直接计数之和)；
    select_timeout/select_error 是任务级事件，单列在 task_drops，不摊到 pipe。
    """
    frames = defaultdict(int)
    ids = defaultdict(list)
    frame_drop = defaultdict(Counter)
    task_drops = defaultdict(Counter)
    queue_last = {}
    queue_events = 0

    for e in eval_events:
        ev = e["event"]
        kv = e["kv"]
        if ev == "frame_in":
            key = (kv.get("pipe"), kv.get("stream", "?"))
            frames[key] += 1
            fid = kv.get("frame_id")
            if isinstance(fid, int):
                ids[key].append(fid)
        elif ev == "frame_drop":
            reason = str(kv.get("reason", "?"))
            stream = kv.get("stream", "?")
            pipe = kv.get("pipe")
            frame_drop[(pipe, stream)][reason] += 1
            if reason in ("select_timeout", "select_error"):
                task_drops[stream][reason] += 1
        elif ev == "queue_stat":
            queue_events += 1
            queue_last[kv.get("pipe")] = kv

    if not frames and queue_events == 0:
        return None

    keys = set(frames)
    for pipe in queue_last:
        keys.add((pipe, "main"))
        keys.add((pipe, "sub"))

    streams = {}
    for key in sorted(keys, key=lambda x: (str(x[0]), str(x[1]))):
        pipe, stream = key
        q = queue_last.get(pipe, {})
        prefix = "sub" if stream == "sub" else "main"
        reasons = {
            "queue_recycle": int(q.get(f"{prefix}_recycled", 0) or 0),
            "empty_null": int(q.get(f"{prefix}_empty_null", 0) or 0),
            "fq_full": int(q.get(f"{prefix}_fq_full", 0) or 0),
            "fq_expired": int(q.get(f"{prefix}_fq_expired", 0) or 0),
            "getframe_fail": int(frame_drop[key].get("getframe_fail", 0)),
        }
        total = sum(reasons.values())
        n = frames[key]
        if n == 0 and total == 0:
            continue
        id_list = ids.get(key, [])
        gaps = _frame_id_missing(id_list)
        step = _modal_step(id_list) if len(id_list) > 1 else 1
        streams[f"pipe{pipe}/{stream}"] = {
            "frames_in": n,
            "drops_total": total,
            "drop_rate": round(total / (n + total), 6) if (n + total) > 0 else None,
            "drops_by_reason": reasons,
            "frame_id_gaps": gaps,
            "frame_id_step": step,
            "queue_meta": {
                "depth": int(q.get(f"{prefix}_depth", 0) or 0),
                "queue_stat_seen": bool(q),
            },
        }

    last_q = next(iter(queue_last.values())) if queue_last else {}
    combine = {
        "queue_recycle": int(last_q.get("combine_recycled", 0) or 0),
        "empty_null": int(last_q.get("combine_empty_null", 0) or 0),
        "depth": int(last_q.get("combine_depth", 0) or 0),
    }

    return {
        "streams": streams,
        "task_drops": {k: dict(v) for k, v in task_drops.items()},
        "combine": combine,
        "queue_stat_events": queue_events,
        "queue_counters_available": queue_events > 0,
    }


def startup_metrics(eval_events: List[Dict]) -> Optional[Dict]:
    """启动阶段：node_init(t0) -> vflow_start -> first_frame -> first_pub。

    直接证据：每个阶段的 [EVAL] 事件 ts_mono；node_init 事件额外带 t0_ns
    （构造函数起始时刻），用于计算相对耗时。不反推。
    """
    evs = [e for e in eval_events if e["event"] == "startup"]
    if not evs:
        return None
    evs.sort(key=lambda e: e["ts_mono"])
    node_init = next((e for e in evs if e["kv"].get("stage") == "node_init"), None)
    t0 = None
    if node_init is not None:
        t0 = node_init["kv"].get("t0_ns")
        if not isinstance(t0, int):
            t0 = node_init["ts_mono"]

    stages = []
    first_frame = {}
    first_pub = {}
    for e in evs:
        kv = e["kv"]
        stage = kv.get("stage")
        item = {"stage": stage, "ts_mono": e["ts_mono"]}
        for k in ("pipe", "stream", "topic", "channel", "ret"):
            if k in kv:
                item[k] = kv[k]
        if t0 is not None:
            item["since_t0_ms"] = round((e["ts_mono"] - t0) / 1e6, 3)
        stages.append(item)
        if stage == "first_frame":
            key = "pipe%s/%s" % (kv.get("pipe", "?"), kv.get("stream", "?"))
            first_frame.setdefault(key, item)
        elif stage == "first_pub":
            key = str(kv.get("topic", "?"))
            first_pub.setdefault(key, item)

    ff = [v["since_t0_ms"] for v in first_frame.values() if "since_t0_ms" in v]
    fp = [v["since_t0_ms"] for v in first_pub.values() if "since_t0_ms" in v]

    # begin/end 配对，直接产出阶段耗时（不靠人工相减）
    durations = []
    pending = {}
    for s in stages:
        stage = s.get("stage") or ""
        if stage.endswith("_begin"):
            pending[(stage[:-6], s.get("pipe"))] = s
        elif stage.endswith("_end"):
            key = (stage[:-4], s.get("pipe"))
            begin = pending.pop(key, None)
            if begin is not None:
                durations.append({
                    "stage": key[0],
                    "pipe": s.get("pipe"),
                    "duration_ms": round((s["ts_mono"] - begin["ts_mono"]) / 1e6, 3),
                    "ret": s.get("ret", "-"),
                })

    def _ret(name):
        vals = [s.get("ret") for s in stages if s.get("stage") == name]
        return vals[-1] if vals else None

    complete = (_ret("backend_init_end") == 0 and
                _ret("backend_start_end") == 0 and bool(first_frame))

    return {
        "t0_ns": t0,
        "stages": stages,
        "stage_durations": durations,
        "first_frame": first_frame,
        "first_pub": first_pub,
        "time_to_first_frame_ms": min(ff) if ff else None,
        "time_to_first_pub_ms": min(fp) if fp else None,
        "complete": complete,
    }


def stability_warnings(frame_timing: Optional[Dict], expected: Optional[Dict]) -> List[Dict]:
    """帧时序稳定性告警：最大间隔超过 3 个目标周期。"""
    if not frame_timing or not expected:
        return []
    fps = expected.get("framerate") or 0
    if fps <= 0:
        return []
    threshold_ns = 3e9 / float(fps)
    out = []
    for key, m in sorted(frame_timing.items()):
        mx = m.get("interval_max_ns")
        if mx and mx > threshold_ns:
            out.append({
                "stream": key,
                "interval_max_ms": round(mx / 1e6, 3),
                "threshold_ms": round(threshold_ns / 1e6, 3),
            })
    return out


def _summary(vals: List[int], div: float = 1.0, ndigits: int = 3) -> Optional[Dict]:
    """升序分位摘要；div 为单位换算（ns->ms 用 1e6，us 用 1）。"""
    if not vals:
        return None
    s = sorted(vals)
    return {
        "count": len(s),
        "min": round(s[0] / div, ndigits),
        "mean": round(sum(s) / len(s) / div, ndigits),
        "p50": round(percentile(s, 50) / div, ndigits),
        "p95": round(percentile(s, 95) / div, ndigits),
        "p99": round(percentile(s, 99) / div, ndigits),
        "max": round(s[-1] / div, ndigits),
    }


def _domain_of(vals: List[int]) -> Optional[str]:
    """按量级判定时钟域：realtime ~1.7e18，monotonic 为开机纳秒（<1e17）。"""
    if not vals:
        return None
    return "realtime" if percentile(sorted(vals), 50) > REALTIME_DETECT_NS else "mono"


EXP_US_HARD_CAP = 5000000   # 单帧“触发→取帧”硬上限 5s，防跨域脏值
EXP_US_MIN_CAP = 1000000    # 闸门下限 1s，避免误杀低帧率长曝光


def _exposure_gate(rows: List[Dict]) -> Dict:
    """判定一组 exposure 记录里哪些帧的 (trig, exp) 可信。

    `trig_valid` 只判 `trig_tv` 非零，而 LPWM 关闭后驱动会残留一个固定旧值；
    因此再加两条闸门：
    - `trig` 必须在推进：同 pipe 内出现多个不同 trig 值，恒定即视为陈旧；
    - `0 < exp <= max(1s, 1.5×帧间隔)`，避免用跨域/垃圾曝光量修正 t0。

    返回 {'stale': bool, 'cap_us': int, 'usable': [(ts_frame, exp_us)]}
    """
    def _i(kv: Dict, key: str) -> Optional[int]:
        v = kv.get(key)
        return v if isinstance(v, int) else None

    trig_vals = [v for r in rows for v in [_i(r, "ts_trig_ns")] if v and v > 0]
    frames = [v for r in rows for v in [_i(r, "ts_frame")] if v is not None]
    itv = [b - a for a, b in zip(frames, frames[1:]) if b > a]
    itv_us = percentile(sorted(itv), 50) / 1000.0 if itv else 0.0
    cap_us = int(max(float(EXP_US_MIN_CAP), 1.5 * itv_us)) if itv_us else EXP_US_MIN_CAP
    cap_us = min(cap_us, EXP_US_HARD_CAP)
    stale = len(trig_vals) > 1 and len(set(trig_vals)) == 1
    usable = []
    for r in rows:
        if stale or _i(r, "trig_valid") != 1:
            continue
        tg, e, f = _i(r, "ts_trig_ns"), _i(r, "exp_us"), _i(r, "ts_frame")
        if tg is None or tg <= 0 or e is None or f is None:
            continue
        if 0 < e <= cap_us:
            usable.append((f, e))
    return {"stale": stale, "cap_us": cap_us, "usable": usable}


def exposure_metrics(eval_events: List[Dict]) -> Optional[Dict]:
    """曝光/触发直接量统计（05-exposure）。

    输入只取 exposure 事件；每个字段都是该帧在 getVnodeFrame 处的原始量：
    - trig_valid: 产品代码判据（trig_tv.sec 与 usec 同时非零）
    - ts_trig_ns / ts_hw_ns: trig_tv / tv 的原始硬件值
    - exp_us: int64 计算的 (tv - trig_tv)，trig 无效时按既有口径记 0
    - ts_sys_raw_ns: 被曝光回退修改之前的 sys_timestamps
    - ts_frame: 最终写入 buffer 的 stamp（sensor 分支=timestamps，realtime 分支=回退后 sys）
    - stamp_src: 实际走的分支

    t0 口径（方案 6.1）：trig 可信 -> ts_trig + exp/2；否则曝光不可用，
    t0 退化为帧 ts（不拿垃圾 exp 修正，realtime/sensor 只在 t0_basis 里区分）。
    输出 t0 序列的间隔自洽性与口径混用标记。
    """
    evs = [e for e in eval_events if e["event"] == "exposure"]
    if not evs:
        return None

    def _i(kv: Dict, key: str) -> Optional[int]:
        v = kv.get(key)
        return v if isinstance(v, int) else None

    groups = defaultdict(list)
    for e in evs:
        kv = e["kv"]
        groups[(kv.get("pipe"), str(kv.get("stream")))].append((e["ts_mono"], kv))

    pipes = {}
    for (pipe, stream), items in sorted(groups.items(), key=lambda x: (str(x[0][0]), x[0][1])):
        rows = [kv for _, kv in items]
        n = len(rows)
        trig_valid = sum(1 for r in rows if _i(r, "trig_valid") == 1)
        src = Counter(str(r.get("stamp_src")) for r in rows)

        exps = [v for r in rows if _i(r, "trig_valid") == 1
                for v in [_i(r, "exp_us")] if v is not None]
        frames = [v for r in rows for v in [_i(r, "ts_frame")] if v is not None]
        sys_raw = [v for r in rows for v in [_i(r, "ts_sys_raw_ns")] if v is not None]

        hw_delta, sys_delta = [], []
        for r in rows:
            f = _i(r, "ts_frame")
            if f is None:
                continue
            h = _i(r, "ts_hw_ns")
            if h is not None and h > 0:
                hw_delta.append(h - f)
            s = _i(r, "ts_sys_raw_ns")
            if s is not None and s > 0:
                sys_delta.append(s - f)

        # realtime 分支：ts_frame 应等于 sys_raw 回退 exp 后的值
        adj_checked = adj_mismatch = 0
        for r in rows:
            if str(r.get("stamp_src")) != "realtime" or _i(r, "trig_valid") != 1:
                continue
            f, s, e = _i(r, "ts_frame"), _i(r, "ts_sys_raw_ns"), _i(r, "exp_us")
            if f is None or s is None or e is None:
                continue
            adj_checked += 1
            if abs((s - f) - e * 1000) > 1000:
                adj_mismatch += 1

        gate = _exposure_gate(rows)
        usable_exp = dict(gate["usable"])
        trig_usable = len(gate["usable"])
        # 打点时刻（CLOCK_MONOTONIC）与硬件时戳 tv 的差：用于识别字段的时钟域
        hw_mono = []
        for mono, r in items:
            h = _i(r, "ts_hw_ns")
            if h is not None and h > 0:
                hw_mono.append(h - mono)

        t0s, t0_basis = [], Counter()
        for r in rows:
            f = _i(r, "ts_frame")
            if f is None:
                continue
            if f in usable_exp:
                t0s.append(_i(r, "ts_trig_ns") + usable_exp[f] * 1000 // 2)
                t0_basis["trig"] += 1
                continue
            # 曝光不可用（无触发或 trig 为陈旧值）：不拿 exp 做修正，
            # 直接退化为帧 ts 口径，避免用垃圾曝光量污染 t0。
            if str(r.get("stamp_src")) == "realtime":
                t0s.append(f)
                t0_basis["frame_ts_realtime"] += 1
            else:
                t0s.append(f)
                t0_basis["frame_ts_sensor"] += 1
        t0_deltas = [b - a for a, b in zip(t0s, t0s[1:])]

        pipes["%s/%s" % (pipe, stream)] = {
            "count": n,
            "trig_valid": trig_valid,
            "trig_valid_ratio": round(trig_valid / n, 4),
            "trig_usable": trig_usable,
            "trig_usable_ratio": round(trig_usable / n, 4),
            "trig_stale": gate["stale"],
            "exp_cap_us": gate["cap_us"],
            "stamp_src": dict(sorted(src.items())),
            "exp_us": _summary(exps, 1.0),
            "exp_negative": sum(1 for v in exps if v < 0),
            "exp_zero": sum(1 for v in exps if v == 0),
            "exp_over_1s": sum(1 for v in exps if v > 1000000),
            "frame_domain": _domain_of(frames),
            "sys_raw_domain": _domain_of(sys_raw),
            "hw_minus_frame_ms": _summary(hw_delta, 1e6),
            "sys_raw_minus_frame_ms": _summary(sys_delta, 1e6),
            # tv 与打点 CLOCK_MONOTONIC 的差：非 0 且稳定即为域偏移（不是管线延迟）
            "hw_minus_event_mono_ms": _summary(hw_mono, 1e6),
            "realtime_adjust": {"checked": adj_checked, "mismatch": adj_mismatch},
            "t0_basis": dict(sorted(t0_basis.items())),
            "t0_mixed": ("trig" in t0_basis and
                         any(k.startswith("frame_ts") for k in t0_basis)),
            "t0_interval_ms": _summary(t0_deltas, 1e6),
            "t0_backwards": sum(1 for d in t0_deltas if d < 0),
        }
    return {"total": len(evs), "pipes": pipes}


def sync_metrics(eval_events: List[Dict]) -> Optional[Dict]:
    """双目/主辅配对点统计（06-stereo-sync）。

    输入只取 `sync_pair` 事件：
    - ok=1：每配成一对一行，字段为配对点两侧的 frame_id 与 stamp；
    - ok=0：缺帧 / 超容差，已在驱动侧限流 1 条/秒，只能看“是否发生过”。
    配对率分母用同 stream 各 pipe 的 `frame_in` 帧数取 min（理论上限）。
    """
    evs = [(e["ts_mono"], e["kv"]) for e in eval_events if e["event"] == "sync_pair"]
    if not evs:
        return None

    def _i(kv: Dict, key: str) -> Optional[int]:
        v = kv.get(key)
        return v if isinstance(v, int) else None

    # 各 (stream, pipe) 的 frame_in 帧数，用于配对率分母
    frame_counts = defaultdict(Counter)
    for e in eval_events:
        if e["event"] != "frame_in":
            continue
        kv = e["kv"]
        frame_counts[str(kv.get("stream", "-"))][kv.get("pipe")] += 1

    groups = defaultdict(list)
    for mono, kv in evs:
        groups[str(kv.get("tag", "-"))].append((mono, kv))

    out = {}
    for tag, items in sorted(groups.items()):
        oks = [kv for _, kv in items if _i(kv, "ok") == 1]
        fails = [kv for _, kv in items if _i(kv, "ok") == 0]
        diffs = [v for kv in oks for v in [_i(kv, "diff_ns")] if v is not None]
        mismatch = 0
        delta = Counter()
        for kv in oks:
            a, b = _i(kv, "frame_id_a"), _i(kv, "frame_id_b")
            if a is None or b is None:
                continue
            delta[a - b] += 1
            if a != b:
                mismatch += 1
        tol = next((v for kv in items for v in [_i(kv[1], "tol_ns")] if v is not None), None)
        counts = frame_counts.get(tag) or {}
        expected = min(counts.values()) if counts else None
        pair_times = [mono for mono, kv in items if _i(kv, "ok") == 1]
        deltas = [b - a for a, b in zip(pair_times, pair_times[1:])]
        reasons = Counter(str(kv.get("reason")) for kv in fails)
        missing = Counter(_i(kv, "missing") for kv in fails
                          if str(kv.get("reason", "")).startswith("missing"))
        # 两路队列都空是帧间常态（轮询 100Hz、帧率 30fps），不算配对失败；
        # 只有“一路有帧、另一路没有”才是单侧停摆/到达不对称的直接证据。
        # 队列数优先取事件里的 n（某一路彻底无 frame_in 时 counts 会少一路）。
        n_ev = next((v for _, kv in items for v in [_i(kv, "n")] if v and v > 0), None)
        n_pipes = n_ev if n_ev else (len(counts) if counts else 2)
        full_mask = (1 << n_pipes) - 1
        missing_idle = sum(v for k, v in missing.items() if k == full_mask)
        missing_asym = sum(v for k, v in missing.items()
                           if k is not None and k != 0 and k != full_mask)
        ds = sorted(diffs)
        out[tag] = {
            "pairs": len(oks),
            "expected": expected,
            "pair_rate": round(len(oks) / expected, 6) if expected else None,
            "diff_p50_ns": percentile(ds, 50) if ds else None,
            "diff_p95_ns": percentile(ds, 95) if ds else None,
            "diff_p99_ns": percentile(ds, 99) if ds else None,
            "diff_max_ns": ds[-1] if ds else None,
            "diff_mean_ns": round(sum(ds) / len(ds), 3) if ds else None,
            "frame_id_mismatch": mismatch,
            # a-b 的分布：单一非零值=固定帧号偏移（启动相位），多样=可能掩盖丢帧
            "frame_id_delta": {str(k): v for k, v in sorted(delta.items())},
            "tol_ns": tol,
            "fails": dict(sorted(reasons.items())),
            "fail_missing": {str(k): v for k, v in sorted(missing.items(),
                                                          key=lambda x: str(x[0]))},
            "missing_idle": missing_idle,   # 两路都空：帧间常态
            "missing_asym": missing_asym,   # 单侧有帧：到达不对称
            "pair_interval_p50_ns": percentile(sorted(deltas), 50) if deltas else None,
            "pair_interval_max_ns": max(deltas) if deltas else None,
        }
    return out


COMBINE_TOPIC_PAIRS = {
    "main": ("/image_combine_raw", "/image_left_raw"),
    "sub": ("/sub_image_combine_raw", "/sub_image_left_raw"),
}


def combine_metrics(eval_events: List[Dict], rows: Optional[List[RecRow]] = None,
                    expected: Optional[Dict] = None) -> Optional[Dict]:
    """拼接产出统计（07-combine）。

    直接回答：
    - `ts_combine` 的拷贝来源（应等于 `ts_left`，不是右目/最大值）；
    - `frame_id` 来源与左右一致性；
    - 输出几何（combine 应为 w × 2h）；
    - `order`：参与拼接的 pipe 顺序；
    - B-11 额外延迟：`combine pub_out − 同帧 left pub_out`（设备侧），
      并用 recorder 接收侧同 stamp 差值交叉校验。
    """
    evs = [e for e in eval_events if e["event"] == "combine_out"]
    if not evs:
        return None

    def _i(kv: Dict, key: str) -> Optional[int]:
        v = kv.get(key)
        return v if isinstance(v, int) else None

    # pub_out: topic -> {ts_frame: ts_mono}；重复 key 单独计数（覆盖写会静默丢样本）
    pub = defaultdict(dict)
    pub_dup = Counter()
    for e in eval_events:
        if e["event"] != "pub_out":
            continue
        kv = e["kv"]
        topic, ts = kv.get("topic"), _i(kv, "ts_frame")
        if isinstance(topic, str) and ts is not None:
            if ts in pub[topic]:
                pub_dup[topic] += 1
            pub[topic][ts] = e["ts_mono"]

    # recorder 接收侧: topic -> {stamp: recv_mono}
    recv = defaultdict(dict)
    recv_dup = Counter()
    for r in rows or []:
        if r.encoding in ("imu", "camera_info"):
            continue
        if r.stamp_ns in recv[r.topic]:
            recv_dup[r.topic] += 1
        recv[r.topic][r.stamp_ns] = r.recv_mono_ns

    groups = defaultdict(list)
    for e in evs:
        groups[str(e["kv"].get("tag", "-"))].append(e["kv"])

    out = {}
    for tag, items in sorted(groups.items()):
        exp = (expected or {}).get(
            "sub_combine" if tag == "sub" else "combine") or {}
        left_ok = right_ok = other = 0
        fid_left_ok = fid_right_ok = 0
        w_bad = h_bad = 0
        rl, extra_dev, extra_recv = [], [], []
        orders = Counter()
        for kv in items:
            tc, tl, tr = _i(kv, "ts_combine"), _i(kv, "ts_left"), _i(kv, "ts_right")
            if tc is not None and tl is not None and tc == tl:
                left_ok += 1
            elif tc is not None and tr is not None and tc == tr:
                right_ok += 1
            else:
                other += 1
            if tl is not None and tr is not None:
                rl.append(tr - tl)
            fid = _i(kv, "frame_id")
            fl, fr = _i(kv, "frame_id_left"), _i(kv, "frame_id_right")
            if fid is not None and fl is not None and fid == fl:
                fid_left_ok += 1
            if fid is not None and fr is not None and fid == fr:
                fid_right_ok += 1
            if exp:
                if _i(kv, "width") != exp.get("width"):
                    w_bad += 1
                if _i(kv, "height") != exp.get("height"):
                    h_bad += 1
            orders[str(kv.get("order"))] += 1
            pair = COMBINE_TOPIC_PAIRS.get(tag)
            if pair and tl is not None and tc is not None:
                ct, lt = pair
                a, b = pub[ct].get(tc), pub[lt].get(tl)
                if a is not None and b is not None:
                    extra_dev.append(a - b)
                a2, b2 = recv[ct].get(tc), recv[lt].get(tl)
                if a2 is not None and b2 is not None:
                    extra_recv.append(a2 - b2)
        pair_topics = COMBINE_TOPIC_PAIRS.get(tag, ())
        out[tag] = {
            "count": len(items),
            "ts_source": {"left": left_ok, "right": right_ok, "other": other},
            "ts_right_minus_left_ns": _summary(rl, 1.0),
            "frame_id_match_left": fid_left_ok,
            "frame_id_match_right": fid_right_ok,
            "width_mismatch": w_bad,
            "height_mismatch": h_bad,
            "expected_width": exp.get("width"),
            "expected_height": exp.get("height"),
            "order": dict(sorted(orders.items())),
            "extra_latency_ns": _summary(extra_dev, 1.0),
            "extra_latency_checked": len(extra_dev),
            "extra_latency_coverage": (round(len(extra_dev) / len(items), 4)
                                       if items else None),
            "extra_recv_ns": _summary(extra_recv, 1.0),
            "extra_recv_checked": len(extra_recv),
            "extra_recv_coverage": (round(len(extra_recv) / len(items), 4)
                                    if items else None),
            # 字典覆盖写会静默丢样本，这里显式暴露重复 key 数
            "pub_duplicates": sum(pub_dup[t] for t in pair_topics),
            "recv_duplicates": sum(recv_dup[t] for t in pair_topics),
        }
    return out


def analyze(rows: List[RecRow], eval_text: str, image_domain: Optional[str] = None,
            clock_offset_ns: int = 0, expected: Optional[Dict] = None,
            expected_topics: Optional[List[str]] = None) -> Dict:
    """总入口：产出指标字典（报告/JSON 用）。image_domain=None 时自动识别。"""
    ev_stats = {}
    events = parse_eval_log(eval_text, ev_stats)
    if image_domain is None:
        image_domain = detect_image_domain(rows)
    image_topics = sorted({r.topic for r in rows if r.encoding not in ("imu", "camera_info")})
    result = {
        "meta": {
            "image_domain": image_domain,
            "clock_offset_ns": clock_offset_ns,
            "eval_events": len(events),
            "eval_duplicate_lines": ev_stats.get("duplicates", 0),
            "eval_by_event": dict(sorted(
                (ev, sum(1 for e in events if e["event"] == ev))
                for ev in {e["event"] for e in events})),
        },
        "topics": topic_metrics(rows),
        "frame_in": frame_in_metrics(events),
        "pub_out": pub_out_metrics(events),
        "latency": latency_metrics(rows, events, image_domain, clock_offset_ns),
    }
    startup = startup_metrics(events)
    if startup:
        startup["missing_first_pub"] = sorted(
            t for t in image_topics if t not in startup.get("first_pub", {}))
        result["startup"] = startup
    frame_timing = frame_timing_metrics(events)
    if frame_timing:
        result["frame_timing"] = frame_timing
        warns = stability_warnings(frame_timing, expected)
        if warns:
            result["stability_warnings"] = warns
    drop = drop_metrics(events)
    if drop:
        result["drop"] = drop
    resolution = resolution_metrics(rows, events, expected, expected_topics)
    if resolution:
        result["resolution"] = resolution
    exposure = exposure_metrics(events)
    if exposure:
        result["exposure"] = exposure
    sync_pair = sync_metrics(events)
    if sync_pair:
        result["sync_pair"] = sync_pair
    combine = combine_metrics(events, rows, expected)
    if combine:
        result["combine"] = combine
    pairs = []
    for i in range(1, len(image_topics)):
        p = pairing_metrics(rows, image_topics[0], image_topics[i],
                            image_domain, clock_offset_ns)
        if p:
            pairs.append(p)
    if pairs:
        result["pairing"] = pairs
    if image_topics:
        sync = imu_sync_metrics(rows, image_topics[0], image_domain, clock_offset_ns)
        if sync:
            result["imu_nearest"] = sync
    return result
