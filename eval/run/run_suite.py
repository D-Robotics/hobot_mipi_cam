#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""suite 编排器：顺序跑多个用例 → 逐个分析 → 汇总三态报告。

换 sensor / 改配置后的标准入口：
    ./eval/run/run_suite.py --suite smoke        # 快速自检
    ./eval/run/run_suite.py --suite core         # 核心基线
    ./eval/run/run_suite.py C04 C07 --duration 60
    ./eval/run/run_suite.py --analyze-only <outdir1> <outdir2>   # 只重算汇总

产出：eval/logs/suite_<时间戳>/suite_report.md + suite_metrics.json
退出码：任一用例 FAIL → 1；否则 0。
"""
import argparse
import datetime
import json
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ = os.environ.get("PROJ") or os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ANALYZE = os.path.join(PROJ, "eval", "tools", "analyze", "analyze.py")
RUN_CASE = os.path.join(PROJ, "eval", "run", "run_case.sh")

sys.path.insert(0, os.path.join(PROJ, "eval", "tools", "analyze"))
from suite_report import render_suite_report, summarize_run  # noqa: E402

try:
    import yaml
except ImportError:
    yaml = None


def load_suites():
    path = os.path.join(SCRIPT_DIR, "suites.yaml")
    if not yaml or not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        return (yaml.safe_load(f) or {}).get("suites") or {}


def parse_result_txt(outdir):
    """result.txt -> {outdir, RESULT, elapsed_s}（缺字段宽容）。"""
    info = {"outdir": outdir, "result": "?", "elapsed_s": None}
    path = os.path.join(outdir, "result.txt")
    if not os.path.exists(path):
        return info
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("RESULT="):
                info["result"] = line.strip().split("=", 1)[1].split()[0] or "?"
            elif line.startswith("elapsed_s="):
                try:
                    info["elapsed_s"] = int(line.strip().split("=", 1)[1])
                except ValueError:
                    pass
    return info


def analyze_outdir(outdir):
    """对单个运行目录执行分析，返回 metrics dict（失败返回 {}）。"""
    csv = os.path.join(outdir, "recorder.csv")
    log = os.path.join(outdir, "mipi_cam.eval.log")
    js = os.path.join(outdir, "metrics.json")
    md = os.path.join(outdir, "report.md")
    if not os.path.exists(csv):
        return {}
    cmd = [sys.executable, ANALYZE, "--csv", csv, "--out-json", js, "--out-md", md]
    if os.path.exists(log):
        cmd += ["--log", log]
    r = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if r.returncode != 0 or not os.path.exists(js):
        return {}
    with open(js, encoding="utf-8") as f:
        return json.load(f)


def run_case(case, duration, known_dirs):
    """跑一个用例，返回本次新增的 outdir（失败返回 None）。

    板端时钟不可靠（可能回到 2000 年），目录名字典序不能代表新旧：
    优先取“跑前快照之后新出现”的目录，多个时按 mtime 取最新；
    没有新增（复跑同名校验冲突等）回退到该用例 mtime 最新的目录。
    """
    cmd = [RUN_CASE, case]
    if duration:
        cmd.append(str(duration))
    r = subprocess.run(cmd)
    if r.returncode != 0:
        return None
    logs = os.path.join(PROJ, "eval", "logs")
    fresh, any_case = [], []
    for name in os.listdir(logs):
        if not name.startswith(case + "_"):
            continue
        path = os.path.join(logs, name)
        if not os.path.isdir(path):
            continue
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        any_case.append((mtime, path))
        if path not in known_dirs:
            fresh.append((mtime, path))
    pool = fresh or any_case
    return max(pool)[1] if pool else None


def main():
    ap = argparse.ArgumentParser(description="mipi_cam 评测 suite 编排器")
    ap.add_argument("cases", nargs="*", help="用例 ID（如 C04 C07）")
    ap.add_argument("--suite", help="suites.yaml 中的预设名")
    ap.add_argument("--duration", type=int, default=None,
                    help="覆盖所有用例时长（秒）；缺省用 YAML 值")
    ap.add_argument("--suite-name", default=None, help="报告中的 suite 名（默认自动）")
    ap.add_argument("--analyze-only", nargs="*", metavar="OUTDIR",
                    help="不跑用例，只对给定运行目录重算并汇总")
    args = ap.parse_args()

    if args.analyze_only is not None:
        # case 名从目录名取 <CASE>_ 前缀（如 C04_dual-... -> C04）
        runs_src = [(os.path.basename(os.path.normpath(d)).split("_")[0], d)
                    for d in args.analyze_only]
        suite_name = args.suite_name or "analyze-only"
    else:
        cases, dur = list(args.cases), args.duration
        if args.suite:
            suites = load_suites()
            if args.suite not in suites:
                print(f"error: suite '{args.suite}' 不在 suites.yaml", file=sys.stderr)
                return 2
            spec = suites[args.suite] or {}
            cases = spec.get("cases") or cases
            dur = args.duration or spec.get("duration_s")
            suite_name = args.suite_name or args.suite
        elif not cases:
            ap.print_help()
            return 2
        else:
            suite_name = args.suite_name or "adhoc"
        runs_src = [(c, dur) for c in cases]

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_root = os.path.join(PROJ, "eval", "logs", f"suite_{suite_name}_{stamp}")
    os.makedirs(out_root, exist_ok=True)

    runs = []
    known_dirs = set()
    logs_dir = os.path.join(PROJ, "eval", "logs")
    if os.path.isdir(logs_dir):
        known_dirs = {os.path.join(logs_dir, n) for n in os.listdir(logs_dir)}
    for case, dur in runs_src:
        if args.analyze_only is not None:
            outdir = os.path.abspath(dur)
            if not os.path.isdir(outdir):
                print(f"skip: {outdir} 不是目录")
                continue
        else:
            print(f"==> run {case} (duration={dur or 'yaml'})")
            outdir = run_case(case, dur, known_dirs)
            if outdir is None:
                runs.append({"case": case, "outdir": None, "result": "FAIL",
                             "elapsed_s": None, "metrics": {}})
                print(f"==> {case} FAILED (runner)")
                continue
            known_dirs.add(outdir)
        info = parse_result_txt(outdir)
        info["case"] = case
        print(f"==> analyze {case}: {outdir} RESULT={info['result']}")
        info["metrics"] = analyze_outdir(outdir)
        runs.append(info)

    suite = {"suite": suite_name,
             "runs": [summarize_run(r) for r in runs],
             "raw": runs}
    md = render_suite_report(suite)
    md_path = os.path.join(out_root, "suite_report.md")
    js_path = os.path.join(out_root, "suite_metrics.json")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(md)
    with open(js_path, "w", encoding="utf-8") as f:
        json.dump(suite, f, ensure_ascii=False, indent=2)
    print(f"\nsuite report -> {md_path}")
    print(md.splitlines()[2] if len(md.splitlines()) > 2 else "")

    failed = any(r.get("result") != "OK" and r.get("result") != "?" for r in runs)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
