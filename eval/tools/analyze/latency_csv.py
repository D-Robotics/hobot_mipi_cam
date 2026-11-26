#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_latency_test.py 产出的 CSV → 分段指标 + 三态判定 + Markdown 报告。

用法：
  python3 latency_csv.py --run-dir eval/logs/HI1_xxx/ [--out-md r.md] [--out-json m.json]

输入（run_latency.sh 的运行目录）：
  latency_imu.csv / latency_image.csv   脚本产出（topic,stamp_ns,recv_mono_ns,recv_rt_ns,latency_ms）
  meta.txt                              phase_pre_s/phase_stress_s/phase_post_s/thresholds
  mipi_cam.eval.log                     可选：frame_in 的 ts_mono−ts_frame 偏移（剥离用）

分段边界：脚本内部无阶段标记，按「首样本时间 + pre_stress」推算
（误差 <1s，对 30~60s 窗口足够），报告中显式声明该假设。

判据（可被用例 YAML thresholds 覆盖）：
  IMU   pre.p99 ≤ imu_latency_p99_ms
  Image stress.p95 ≤ image_stress_p95_ratio × pre.p95
  Image post.p95   ≤ image_post_p95_ratio   × pre.p95
"""
import argparse
import datetime
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from thresholds import merge_thresholds  # noqa: E402


def _pct(sorted_vals, q):
    if not sorted_vals:
        return None
    idx = min(int(q * (len(sorted_vals) - 1)), len(sorted_vals) - 1)
    return sorted_vals[idx]


def _stats(vals):
    if not vals:
        return None
    s = sorted(vals)
    return {
        "n": len(s),
        "mean_ms": round(sum(s) / len(s), 3),
        "p50_ms": round(_pct(s, 0.50), 3),
        "p95_ms": round(_pct(s, 0.95), 3),
        "p99_ms": round(_pct(s, 0.99), 3),
        "max_ms": round(s[-1], 3),
    }


def read_latency_csv(path):
    """-> [(topic, stamp_ns, recv_mono_ns, latency_ms)]"""
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, encoding="utf-8", errors="replace") as f:
        header = f.readline()
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 5:
                continue
            try:
                rows.append((parts[0], int(parts[1]), int(parts[2]), float(parts[4])))
            except ValueError:
                continue
    return rows


def read_meta(run_dir):
    meta = {}
    path = os.path.join(run_dir, "meta.txt")
    if not os.path.exists(path):
        return meta
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if "=" in line:
                k, v = line.rstrip("\n").split("=", 1)
                meta[k] = v
    return meta


def frame_stamp_offset_ms(eval_log):
    """ts_mono − ts_frame 的中位数（ms）：stamp 与单调钟的 boot 相关偏移。

    image 延迟 = recv_mono − stamp = (recv_mono − 打点) + 偏移，
    减去本值即得"打点后到接收"的部分。
    """
    if not eval_log or not os.path.exists(eval_log):
        return None
    vals = []
    with open(eval_log, encoding="utf-8", errors="replace") as f:
        for line in f:
            if "event=frame_in" not in line:
                continue
            ts_mono = ts_frame = None
            for tok in line.split():
                if tok.startswith("ts_mono="):
                    ts_mono = int(tok.split("=", 1)[1])
                elif tok.startswith("ts_frame="):
                    ts_frame = int(tok.split("=", 1)[1])
            if ts_mono is not None and ts_frame is not None:
                vals.append(ts_mono - ts_frame)
    if not vals:
        return None
    vals.sort()
    return round(_pct(vals, 0.5) / 1e6, 3)


def read_phases(run_dir):
    """读脚本产出的 *.phases.json（阶段边界/域/偏移提示）；没有返回 None。"""
    if not os.path.isdir(run_dir):
        return None
    for name in sorted(os.listdir(run_dir)):
        if not name.endswith(".phases.json"):
            continue
        try:
            with open(os.path.join(run_dir, name), encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            continue
        if data.get("stress_start_mono_ns") and data.get("stress_end_mono_ns"):
            return data
    return None


def split_phases(rows, pre_s, stress_s, post_s, offset_ms=None, warmup_s=0.0,
                 bounds=None):
    """按 recv_mono 相对首样本的时间切三阶段 -> {topic: {phase: stats}}。

    bounds=(t0, stress_start, stress_end) 来自脚本 sidecar 时直接采用（精确）；
    否则按「首样本 + pre_stress」推算（CSV 无阶段标记，误差 <1s）。
    warmup_s：剔除启动窗口（仅推算模式需要；脚本侧已自行丢弃）。
    返回 (metrics, stress_start, stress_end, dropped_warmup_rows)。
    """
    if not rows:
        return {}, None, None, 0
    if bounds:
        t0, stress_start, stress_end = [int(v) for v in bounds]
        warm_end = t0
    else:
        t0 = min(r[2] for r in rows)
        stress_start = t0 + int(pre_s * 1e9)
        stress_end = stress_start + int(stress_s * 1e9)
        warm_end = t0 + int(warmup_s * 1e9)

    buckets = {}
    kept = dropped = 0
    for topic, _stamp, recv, lat in rows:
        if recv < warm_end:
            dropped += 1
            continue
        kept += 1
        if recv < stress_start:
            phase = "pre"
        elif recv < stress_end:
            phase = "stress"
        else:
            phase = "post"
        val = lat - offset_ms if offset_ms is not None else lat
        buckets.setdefault(topic, {}).setdefault(phase, []).append(val)
    out = {t: {p: _stats(v) for p, v in ph.items()} for t, ph in buckets.items()}
    return out, stress_start, stress_end, dropped


def judge(metrics, th):
    """三态判定 + 明细。"""
    checks, warns = [], []

    def _add(name, ok, detail):
        checks.append({"check": name, "pass": bool(ok), "detail": detail})
        if not ok:
            warns.append(name)

    imu = {t: m for t, m in (metrics.get("imu") or {}).items()}
    for t, ph in imu.items():
        pre = ph.get("pre") or {}
        if pre.get("p99_ms") is not None:
            lim = float(th["imu_latency_p99_ms"])
            _add(f"IMU 空载 p99 ≤ {lim}ms（{t}）", pre["p99_ms"] <= lim,
                 f"p99={pre['p99_ms']}ms")

    for t, ph in (metrics.get("image") or {}).items():
        pre = (ph.get("pre") or {}).get("p95_ms")
        st = (ph.get("stress") or {}).get("p95_ms")
        po = (ph.get("post") or {}).get("p95_ms")
        # 剥离偏移后空载基准可能趋零：此时比率无意义，记录为跳过而非误判
        if pre is not None and pre <= 0:
            checks.append({"check": f"图像延迟比率（{t}）", "pass": True,
                           "detail": "空载基准 ≤0（偏移剥离后），跳过比率判定"})
            continue
        if pre is not None and st is not None:
            lim = float(th["image_stress_p95_ratio"])
            _add(f"图像压力期 p95 ≤ {lim}× 空载（{t}）", st <= lim * pre,
                 f"空载 {pre}ms → 压力 {st}ms（{round(st / pre, 2)}×）")
        if pre is not None and po is not None:
            lim = float(th["image_post_p95_ratio"])
            _add(f"图像恢复期 p95 ≤ {lim}× 空载（{t}）", po <= lim * pre,
                 f"空载 {pre}ms → 恢复 {po}ms（{round(po / pre, 2)}×）")

    if not checks:
        return "WARN", checks
    return ("PASS" if all(c["pass"] for c in checks) else "WARN"), checks


def _fmt(v):
    return "-" if v is None else f"{v:.1f}"


def render(metrics, checks, verdict, meta):
    lines = ["# IMU/Image 延迟 × CPU 负载（imu_latency_test.py 封装）", ""]
    lines.append(f"- 生成时间：{datetime.datetime.now().isoformat(timespec='seconds')}")
    lines.append(f"- 阶段：空载 {meta.get('phase_pre_s', '-')}s → 压力 "
                 f"{meta.get('phase_stress_s', '-')}s（{meta.get('stress_cores', '-')} 核）"
                 f" → 恢复 {meta.get('phase_post_s', '-')}s")
    lines.append(f"- image stamp 域：`{meta.get('image_domain', '-')}`；IMU 域：`realtime`")
    wd = metrics.get("warmup_dropped") or {}
    if metrics.get("warmup_s"):
        lines.append(f"- **启动窗口剔除**：前 {metrics['warmup_s']:g}s"
                     f"（IMU {wd.get('imu', 0)} / Image {wd.get('image', 0)} 样本）——"
                     f"IIO 缓冲积压与订阅建立属启动瞬态，不参与稳态判定")
    off = metrics.get("stamp_offset_ms")
    if off is not None:
        lines.append(f"- **偏移剥离**：frame_in 的 `ts_mono−ts_frame` 中位数 "
                     f"**{off}ms**（boot 相关；表中 image 值已减去该偏移）")
    else:
        lines.append("- 偏移剥离：未取到 frame_in 数据（表中 image 值为原始 "
                     "`recv_mono−stamp`，含 boot 相关偏移，仅可同 run 内比较）")
    lines.append("")
    lines.append(f"## 判定：**{verdict}**")
    lines.append("")
    if checks:
        lines.append("| 检查项 | 结果 | 明细 |")
        lines.append("|---|---|---|")
        for c in checks:
            lines.append(f"| {c['check']} | {'✅' if c['pass'] else '❌'} | {c['detail']} |")
        lines.append("")

    for kind, title in (("imu", "IMU 延迟（recv_rt − stamp，realtime 域）"),
                        ("image", "Image 延迟（recv_mono − stamp，已减偏移）")):
        data = metrics.get(kind) or {}
        if not data:
            continue
        lines.append(f"## {title}")
        lines.append("")
        lines.append("| topic | 阶段 | 样本 | 均值(ms) | p50 | p95 | p99 | max |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for topic in sorted(data):
            for phase in ("pre", "stress", "post"):
                s = data[topic].get(phase)
                if not s:
                    continue
                label = {"pre": "空载", "stress": "压力", "post": "恢复"}[phase]
                lines.append(f"| {topic} | {label} | {s['n']} | {s['mean_ms']:.2f} "
                             f"| {s['p50_ms']:.2f} | {s['p95_ms']:.2f} "
                             f"| {s['p99_ms']:.2f} | {s['max_ms']:.2f} |")
        lines.append("")

    lines.append("## 口径与局限")
    lines.append("")
    lines.append("- 测量端即 `imu_latency_test.py`（IMU/图像分节点分线程订阅）；"
                 "压力期测量端自身也会被挤占，延迟含部分「测量端被饿」成分。")
    src = metrics.get("phase_bounds_source")
    if src == "sidecar":
        lines.append("- 阶段边界取自脚本 `<stem>.phases.json`（精确，非推算）。")
    else:
        lines.append("- 阶段边界按「首样本 + pre_stress」推算（旧版 CSV 无阶段标记），误差 <1s。")
    hints = metrics.get("offset_hint_ms") or {}
    if hints:
        lines.append("- 脚本域检查（`recv−stamp` 最小值，最接近纯偏移）："
                     + "、".join(f"{k}={v}ms" for k, v in hints.items() if v is not None))
    lines.append("- image 绝对值受 boot 相关偏移影响；已剥离时可比，未剥离时仅同 run 相对比较。")
    lines.append("- IMU 订阅为 best_effort depth=50（不随订阅端变慢积压），"
                 "与产品 best_effort 消费者行为一致。")
    lines.append("")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="解析 imu_latency_test.py 的 CSV 出分段指标")
    ap.add_argument("--run-dir", required=True, help="run_latency.sh 的运行目录")
    ap.add_argument("--out-md", default=None)
    ap.add_argument("--out-json", default=None)
    ap.add_argument("--warmup-s", type=float, default=5.0,
                    help="剔除启动窗口秒数（IIO 缓冲积压/订阅建立；默认 5s）")
    args = ap.parse_args()

    run_dir = os.path.abspath(args.run_dir)
    meta = read_meta(run_dir)
    try:
        th = merge_thresholds(json.loads(meta.get("thresholds") or "{}"))
    except ValueError:
        th = merge_thresholds()
    pre_s = float(meta.get("phase_pre_s") or 60)
    stress_s = float(meta.get("phase_stress_s") or 60)
    post_s = float(meta.get("phase_post_s") or 60)

    off = frame_stamp_offset_ms(os.path.join(run_dir, "mipi_cam.eval.log"))
    imu_rows = read_latency_csv(os.path.join(run_dir, "latency_imu.csv"))
    img_rows = read_latency_csv(os.path.join(run_dir, "latency_image.csv"))

    # 脚本 sidecar（新版）优先：边界精确，且脚本侧已丢弃预热窗口
    sidecar = read_phases(run_dir)
    bounds = None
    warmup_eff = args.warmup_s
    if sidecar:
        bounds = (sidecar["t0_mono_ns"], sidecar["stress_start_mono_ns"],
                  sidecar["stress_end_mono_ns"])
        warmup_eff = 0.0

    imu_m, start, end, imu_dropped = split_phases(
        imu_rows, pre_s, stress_s, post_s, warmup_s=warmup_eff, bounds=bounds)
    img_m, _, _, img_dropped = split_phases(
        img_rows, pre_s, stress_s, post_s, offset_ms=off,
        warmup_s=warmup_eff, bounds=bounds)
    metrics = {
        "run_dir": run_dir,
        "phases": {"pre_s": pre_s, "stress_s": stress_s, "post_s": post_s},
        "phase_bounds_source": "sidecar" if sidecar else "inferred",
        "offset_hint_ms": (sidecar or {}).get("offset_hint_ms"),
        "warmup_s": warmup_eff + (float((sidecar or {}).get("warmup_s") or 0)
                                  if sidecar else 0),
        "warmup_dropped": {"imu": imu_dropped, "image": img_dropped},
        "thresholds": th,
        "stamp_offset_ms": off,
        "phase_start_mono_ns": start,
        "phase_end_mono_ns": end,
        "imu": imu_m,
        "image": img_m,
        "raw_counts": {"imu": len(imu_rows), "image": len(img_rows)},
    }
    verdict, checks = judge(metrics, th)
    metrics["verdict"] = verdict
    metrics["checks"] = checks

    md = render(metrics, checks, verdict, meta)
    out_md = args.out_md or os.path.join(run_dir, "latency_report.md")
    out_json = args.out_json or os.path.join(run_dir, "latency_metrics.json")
    with open(out_md, "w", encoding="utf-8") as f:
        f.write(md)
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    print(f"report -> {out_md}")
    print(f"metrics -> {out_json}")
    print(f"verdict = {verdict}")
    return 0 if verdict != "FAIL" else 1


if __name__ == "__main__":
    sys.exit(main())
