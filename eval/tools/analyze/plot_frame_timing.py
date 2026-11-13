#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 mipi_cam.eval.log 的 frame_in 事件画每 pipe/stream 的帧间隔直方图。

用法：
  python3 eval/tools/analyze/plot_frame_timing.py \
      --log eval/logs/C04_.../mipi_cam.eval.log \
      --out eval/reports/02-frame-timing/plots/frame_interval_hist.png
"""
import argparse
import collections
import re

LINE = re.compile(
    r"\[EVAL\] ts_mono=(\d+) event=frame_in .*?pipe=(\S+) stream=(\S+) ts_frame=(\d+)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="X5 frame interval distribution")
    args = ap.parse_args()

    groups = collections.defaultdict(list)
    with open(args.log, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = LINE.search(line)
            if m:
                groups[(m.group(2), m.group(3))].append(int(m.group(4)))
    if not groups:
        raise SystemExit("日志中没有 frame_in 事件")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    keys = sorted(groups)
    fig, axes = plt.subplots(len(keys), 1, figsize=(10, 3.0 * len(keys)), squeeze=False)
    for ax, key in zip(axes[:, 0], keys):
        ts = groups[key]
        intervals = [(b - a) / 1e6 for a, b in zip(ts, ts[1:]) if b >= a]
        ax.hist(intervals, bins=60, color="#1f77b4", alpha=0.8)
        if intervals:
            ordered = sorted(intervals)
            p99 = ordered[min(int(0.99 * (len(ordered) - 1)), len(ordered) - 1)]
            ax.axvline(p99, color="#d62728", linestyle="--",
                       label="p99=%.3f ms" % p99)
            ax.legend(loc="upper right")
        ax.set_title("pipe%s/%s  intervals (n=%d)" % (key[0], key[1], len(intervals)))
        ax.set_xlabel("interval (ms)")
        ax.set_ylabel("frames")
        ax.grid(axis="y", linestyle="--", alpha=0.3)
    fig.suptitle(args.title)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print("plot -> %s" % args.out)


if __name__ == "__main__":
    main()
