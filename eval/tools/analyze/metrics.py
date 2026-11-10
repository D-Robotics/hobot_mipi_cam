#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""指标统计：域归一、频率/间隔分位、采集丢帧反推、配对、延迟（曝光中点 t0）、IMU。

口径术语见工作区根 CONTEXT.md：
- 域归一：IMU(REALTIME) 与图像(monotonic) 归一到 monotonic 轴（ADR-0001，评测侧做）
- 曝光中点：延迟 t0 = 帧ts − 曝光/2；exp_valid=0 的帧单独分组（帧 ts 口径）
- 采集丢帧：frame_in 的 frame_id 连续性反推；接收丢帧：订阅计数 vs 采集计数
"""
from collections import defaultdict
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
        dropped = None
        if ids and all(isinstance(i, int) for i in ids):
            dropped = max(0, (max(ids) - min(ids) + 1) - len(set(ids)))
        valid = [e["kv"] for e in evs if e["kv"].get("exp_valid") == 1]
        exp_us = [k["exp_us"] for k in valid if "exp_us" in k]
        out[f"pipe{key[0]}/{key[1]}"] = {
            "count": len(evs),
            "dropped_by_frame_id": dropped,
            "drop_rate": round(dropped / (dropped + len(set(ids))), 6)
            if dropped is not None and (dropped + len(set(ids))) > 0 else None,
            "exp_valid_ratio": round(len(valid) / len(evs), 4) if evs else None,
            "exp_us_mean": round(sum(exp_us) / len(exp_us)) if exp_us else None,
        }
    return out


def pub_out_metrics(eval_events: List[Dict]) -> Dict[str, int]:
    out = defaultdict(int)
    for e in eval_events:
        if e["event"] == "pub_out" and "topic" in e["kv"]:
            out[str(e["kv"]["topic"])] += 1
    return dict(out)


def _exp_pairs_sorted(eval_events: List[Dict]):
    """[(ts_frame, exp_us)]（exp_valid=1，升序），供延迟 t0=曝光中点使用。"""
    pairs = []
    for e in eval_events:
        if e["event"] != "frame_in":
            continue
        kv = e["kv"]
        if kv.get("exp_valid") == 1 and "ts_frame" in kv and "exp_us" in kv:
            pairs.append((kv["ts_frame"], kv["exp_us"]))
    pairs.sort()
    return pairs


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
                lat_mid.append(lat + exp_us * 1000 // 2)  # t0 = 帧ts − exp/2
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


def analyze(rows: List[RecRow], eval_text: str, image_domain: Optional[str] = None,
            clock_offset_ns: int = 0) -> Dict:
    """总入口：产出指标字典（报告/JSON 用）。image_domain=None 时自动识别。"""
    events = parse_eval_log(eval_text)
    if image_domain is None:
        image_domain = detect_image_domain(rows)
    image_topics = sorted({r.topic for r in rows if r.encoding not in ("imu", "camera_info")})
    result = {
        "meta": {
            "image_domain": image_domain,
            "clock_offset_ns": clock_offset_ns,
            "eval_events": len(events),
            "eval_by_event": dict(sorted(
                (ev, sum(1 for e in events if e["event"] == ev))
                for ev in {e["event"] for e in events})),
        },
        "topics": topic_metrics(rows),
        "frame_in": frame_in_metrics(events),
        "pub_out": pub_out_metrics(events),
        "latency": latency_metrics(rows, events, image_domain, clock_offset_ns),
    }
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
