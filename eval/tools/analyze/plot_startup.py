#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 analyze.py 输出的 metrics.json 中的 startup 阶段画成时间线 PNG。

用法：
  python3 eval/tools/analyze/plot_startup.py \
      --metrics eval/reports/01-startup/metrics.json \
      --out eval/reports/01-startup/plots/startup_timeline.png
"""
import argparse
import json


def _label(s):
    parts = [str(s.get("stage", "?"))]
    if s.get("pipe") not in (None, "-", ""):
        parts.append("pipe%s" % s["pipe"])
    if s.get("stream") not in (None, "-", ""):
        parts.append(str(s["stream"]))
    if s.get("topic") not in (None, "-", ""):
        parts.append(str(s["topic"]))
    return " ".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--metrics", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="X5 mipi_cam startup timeline")
    args = ap.parse_args()

    with open(args.metrics, "r", encoding="utf-8") as f:
        data = json.load(f)
    st = data.get("startup")
    if not st or not st.get("stages"):
        raise SystemExit("metrics.json 中没有 startup.stages")

    stages = st["stages"]
    # t0 缺失时 since_t0_ms 不存在；跳过这些点，避免全部画在 0ms 掩盖问题
    stages = [s for s in stages if "since_t0_ms" in s]
    if not stages:
        raise SystemExit("startup 缺少 t0/since_t0_ms，无法画时间线")
    xs = [float(s["since_t0_ms"]) for s in stages]
    ys = list(range(len(stages)))
    labels = [_label(s) for s in stages]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    height = max(4.0, 0.32 * len(stages) + 1.5)
    fig, ax = plt.subplots(figsize=(11, height))
    ax.plot(xs, ys, "-o", color="#1f77b4", markersize=4, linewidth=1.2)
    for x, y, label in zip(xs, ys, labels):
        ax.text(x, y, "  %s (%.1f ms)" % (label, x), va="center", fontsize=8)
    ax.set_yticks(ys)
    ax.set_yticklabels([""] * len(ys))  # 文字已标在点旁，避免重复
    ax.set_xlabel("milliseconds relative to constructor t0")
    ax.set_title(args.title)
    ax.grid(axis="x", linestyle="--", alpha=0.4)
    ax.set_xlim(left=0)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print("plot -> %s" % args.out)


if __name__ == "__main__":
    main()
