#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析链 CLI：[EVAL] 日志 + recorder CSV + 时钟旁车 -> 基线模式报告 + 指标 JSON。

用法：
  python3 analyze.py --csv <recorder.csv> --log <driver.log> \
      [--image-domain mono|realtime] [--out-md report.md] [--out-json metrics.json]

时钟旁车文件自动取 <csv>.clock（recorder 生成）。
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from parsers import parse_clock_sidecar, parse_eval_log, parse_recorder_csv
from metrics import analyze as run_analyze
from report import render_baseline_report


def main():
    ap = argparse.ArgumentParser(description="mipi_cam 评测分析链（基线模式）")
    ap.add_argument("--csv", required=True, help="recorder CSV 路径")
    ap.add_argument("--log", default=None, help="驱动日志（含 [EVAL] 行）路径")
    ap.add_argument("--image-domain", choices=["auto", "mono", "realtime"], default="auto",
                    help="图像 stamp 时钟域（auto=按 stamp 量级自动识别；"
                         "frame_ts_type=sensor→mono，realtime→realtime）")
    ap.add_argument("--out-md", default=None, help="输出 Markdown 报告路径")
    ap.add_argument("--out-json", default=None, help="输出指标 JSON 路径")
    args = ap.parse_args()

    if args.log is not None and not os.path.exists(args.log):
        print(f"error: --log 文件不存在：{args.log}", file=sys.stderr)
        sys.exit(2)

    with open(args.csv, "r", encoding="utf-8") as f:
        rows, skipped = parse_recorder_csv(f.read())
    eval_text = ""
    if args.log:
        with open(args.log, "r", encoding="utf-8", errors="replace") as f:
            eval_text = f.read()

    offset = 0
    clock_path = args.csv + ".clock"
    if os.path.exists(clock_path):
        with open(clock_path, "r", encoding="utf-8") as f:
            off = parse_clock_sidecar(f.read())
            offset = off if off is not None else 0

    result = run_analyze(rows, eval_text,
                         image_domain=None if args.image_domain == "auto" else args.image_domain,
                         clock_offset_ns=offset)
    result["meta"]["csv_skipped_rows"] = skipped

    md = render_baseline_report(result)
    if args.out_md:
        with open(args.out_md, "w", encoding="utf-8") as f:
            f.write(md)
        print(f"report  -> {args.out_md}")
    if args.out_json:
        with open(args.out_json, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print(f"metrics -> {args.out_json}")
    if not args.out_md and not args.out_json:
        print(md)


if __name__ == "__main__":
    main()
