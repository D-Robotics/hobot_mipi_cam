#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基线模式 Markdown 报告生成（只统计不评分；阈值判定属评分层，后续票据）。"""
import datetime
import json
from typing import Dict


def _fmt_ns(v, unit="ms"):
    if v is None:
        return "-"
    if unit == "ms":
        return f"{v / 1e6:.2f}"
    return f"{v / 1e9:.3f}"


def render_baseline_report(metrics: Dict, title: str = "mipi_cam 评测·基线模式报告") -> str:
    lines = []
    lines.append(f"# {title}")
    lines.append("")
    lines.append(f"- 生成时间：{datetime.datetime.now().isoformat(timespec='seconds')}")
    meta = metrics.get("meta", {})
    lines.append(f"- 图像时间戳域：`{meta.get('image_domain', '?')}`（sensor=monotonic）")
    lines.append(f"- 时钟偏移（realtime−monotonic）：{meta.get('clock_offset_ns')} ns")
    lines.append(f"- 埋点事件数：{meta.get('eval_events', 0)} {json.dumps(meta.get('eval_by_event', {}), ensure_ascii=False)}")
    lines.append("")
    lines.append("> 基线模式：仅统计，不判定。Camera-IMU 指标已域归一（ADR-0001）。")
    lines.append("")

    lines.append("## 接收端各 topic 指标")
    lines.append("")
    lines.append("| topic | 帧数 | 均值fps | 间隔p50(ms) | 间隔p99(ms) | 间隔max(ms) | stamp回退 | stamp重复 | seq缺口 |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for topic, m in sorted(metrics.get("topics", {}).items()):
        itv = m.get("recv_interval_ns", {})
        lines.append(
            f"| {topic} | {m['count']} | {m.get('fps_mean', '-')} "
            f"| {_fmt_ns(itv.get('p50'))} | {_fmt_ns(itv.get('p99'))} "
            f"| {_fmt_ns(itv.get('max'))} "
            f"| {m.get('stamp_monotonic_violations', '-')} "
            f"| {m.get('stamp_duplicates', '-')} | {m.get('seq_gaps', '-')} |")
    lines.append("")

    fi = metrics.get("frame_in", {})
    if fi:
        lines.append("## 设备侧采集（frame_in 埋点）")
        lines.append("")
        lines.append("| 流 | 采集帧数 | frame_id反推丢帧 | 丢帧率 | 曝光有效率 | 曝光均值(µs) |")
        lines.append("|---|---|---|---|---|---|")
        for key, m in sorted(fi.items()):
            lines.append(
                f"| {key} | {m['count']} | {m.get('dropped_by_frame_id', '-')} "
                f"| {m.get('drop_rate', '-')} | {m.get('exp_valid_ratio', '-')} "
                f"| {m.get('exp_us_mean', '-')} |")
        lines.append("")

    po = metrics.get("pub_out", {})
    if po:
        lines.append("## 设备侧发布计数（pub_out 埋点）")
        lines.append("")
        for topic, cnt in sorted(po.items()):
            lines.append(f"- `{topic}`：{cnt}")
        lines.append("")

    lat = metrics.get("latency", {})
    if lat:
        lines.append("## 端到端延迟（t0=曝光中点；frame_ts 组=无有效曝光的帧）")
        lines.append("")
        lines.append("| topic | 口径 | 帧数 | mean(ms) | p50(ms) | p95(ms) | p99(ms) | max(ms) |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for topic, m in sorted(lat.items()):
            for scope, label in (("exp_midpoint", "曝光中点"), ("frame_ts", "帧ts")):
                s = m.get(scope)
                if not s:
                    continue
                lines.append(
                    f"| {topic} | {label} | {s['count']} | {s['mean_ms']:.2f} "
                    f"| {s['p50_ms']:.2f} | {s['p95_ms']:.2f} | {s['p99_ms']:.2f} "
                    f"| {s['max_ms']:.2f} |")
        lines.append("")

    for p in metrics.get("pairing", []):
        lines.append("## 多流配对")
        lines.append("")
        lines.append(
            f"- `{p['topic_a']}` vs `{p['topic_b']}`：配对 {p['paired']}"
            f"/max({p['count_a']},{p['count_b']})（率 {p['pair_rate']}），"
            f"差 p99={_fmt_ns(p.get('diff_p99_ns'))}ms max={_fmt_ns(p.get('diff_max_ns'))}ms")
        lines.append("")

    ns = metrics.get("imu_nearest")
    if ns:
        lines.append("## Camera-IMU 最近邻间隔（已域归一，T-06）")
        lines.append("")
        lines.append(
            f"- `{ns['image_topic']}` vs IMU：p50={_fmt_ns(ns.get('nearest_p50_ns'))}ms "
            f"p99={_fmt_ns(ns.get('nearest_p99_ns'))}ms max={_fmt_ns(ns.get('nearest_max_ns'))}ms")
        lines.append("")

    return "\n".join(lines)
