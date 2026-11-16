#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""suite 汇总：从多个单 run 的 metrics.json 抽取关键指标 + 三态判定。

三态（评分层雏形）：
- FAIL：runner RESULT=FAIL（数据链不完整，指标不可信）
- WARN：RESULT=OK 但报告存在 WARNING（指标可用，有超阈值项）
- PASS：无告警

输入 run dict 结构（run_suite.py 组装）：
  {"case","outdir","result","elapsed_s","metrics": <analyze.py 的 metrics.json>}
"""
from typing import Dict, List, Optional


def _fmt(v, unit="", nd=2):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{nd}f}{unit}"
    return f"{v}{unit}"


def summarize_run(run: Dict) -> Dict:
    """从单 run 的 metrics 抽关键指标；缺哪块给 None（用例没开对应事件属正常）。"""
    m = run.get("metrics") or {}
    s = {"case": run.get("case"), "result": run.get("result", "?"),
         "elapsed_s": run.get("elapsed_s"), "outdir": run.get("outdir")}

    ft = m.get("frame_timing") or {}
    s["fps"] = {k: round(v.get("fps_mean", 0), 2) for k, v in sorted(ft.items())} or None

    dr = m.get("drop") or {}
    streams = dr.get("streams") or {}
    if streams:
        total_in = sum(v.get("frames_in", 0) for v in streams.values())
        total_drop = sum(v.get("drops_total", 0) for v in streams.values())
        s["drop_rate"] = round(total_drop / total_in, 6) if total_in else None
    else:
        s["drop_rate"] = None

    res = m.get("resolution") or {}
    s["resolution_match"] = res.get("all_match")

    sp = m.get("sync_pair") or {}
    if sp:
        main = sp.get("main") or {}
        s["pair"] = {"rate": main.get("pair_rate"),
                     "diff_p50_ms": _ns2ms(main.get("diff_p50_ns")),
                     "diff_p99_ms": _ns2ms(main.get("diff_p99_ns")),
                     "fid_mismatch": main.get("frame_id_mismatch")} if main else None
    else:
        s["pair"] = None

    ex = m.get("exposure") or {}
    if ex:
        ratios = {k: v.get("trig_usable_ratio") for k, v in (ex.get("pipes") or {}).items()}
        s["trig_usable"] = ratios or None
    else:
        s["trig_usable"] = None

    dl = m.get("device_latency") or {}
    if dl:
        s["latency_p50_ms"] = {t: _ns2ms((v.get("total_ns") or {}).get("p50"))
                               for t, v in sorted(dl.items())} or None
    else:
        s["latency_p50_ms"] = None

    # WARNING 计数：从 metrics 里可判定的告警源聚合（与报告口径一致）
    warns = []
    if m.get("stability_warnings"):
        warns.append(f"stability×{len(m['stability_warnings'])}")
    if s["resolution_match"] is False:
        warns.append("resolution_mismatch")
    if s["pair"]:
        th = (m.get("meta", {}).get("thresholds") or {})
        if s["pair"]["rate"] is not None and s["pair"]["rate"] < float(th.get("pair_rate_min", 0.999)):
            warns.append("pair_rate_low")
        p99 = s["pair"]["diff_p99_ms"]
        if p99 is not None and p99 > float(th.get("pair_diff_p99_ms", 1.0)):
            warns.append("pair_diff_p99")
    sw = m.get("startup") or {}
    if sw and not sw.get("complete", True):
        warns.append("startup_incomplete")
    s["warnings"] = warns

    if run.get("result") != "OK":
        s["verdict"] = "FAIL"
    elif warns:
        s["verdict"] = "WARN"
    else:
        s["verdict"] = "PASS"
    return s


def _ns2ms(v):
    return round(v / 1e6, 3) if isinstance(v, (int, float)) else None


def render_suite_report(suite: Dict) -> str:
    """suite dict -> Markdown。suite = {"suite":name,"runs":[summarize_run 输出]}"""
    lines = [f"# 评测 suite：{suite.get('suite', '-')}", ""]
    runs = suite.get("runs") or []
    verdict_order = {"FAIL": 0, "WARN": 1, "PASS": 2}
    overall = ("FAIL" if any(r["verdict"] == "FAIL" for r in runs) else
               "WARN" if any(r["verdict"] == "WARN" for r in runs) else
               "PASS" if runs else "-")
    lines.append(f"- 总判定：**{overall}**（{len(runs)} 个用例："
                 f"{sum(1 for r in runs if r['verdict']=='PASS')} PASS / "
                 f"{sum(1 for r in runs if r['verdict']=='WARN')} WARN / "
                 f"{sum(1 for r in runs if r['verdict']=='FAIL')} FAIL）")
    lines.append("")
    lines.append("| 用例 | RESULT | 判定 | 帧率 | 丢帧率 | 分辨率 | 配对率 | 配对差p50/p99(ms) | 延迟p50(ms) | 告警 |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    for r in sorted(runs, key=lambda x: verdict_order.get(x.get("verdict"), 3)):
        fps = "<br>".join(f"{k}:{v}" for k, v in (r.get("fps") or {}).items()) or "-"
        lat = "<br>".join(f"{k.split('/')[-1] or k}:{_fmt(v)}"
                          for k, v in (r.get("latency_p50_ms") or {}).items()) or "-"
        pair = r.get("pair") or {}
        lines.append(
            f"| {r.get('case')} | {r.get('result')} | **{r.get('verdict')}** "
            f"| {fps} | {_fmt(r.get('drop_rate'), '', 4)} "
            f"| {_fmt(r.get('resolution_match'))} "
            f"| {_fmt(pair.get('rate'))} "
            f"| {_fmt(pair.get('diff_p50_ms'))}/{_fmt(pair.get('diff_p99_ms'))} "
            f"| {lat} "
            f"| {', '.join(r.get('warnings') or []) or '-'} |")
    lines.append("")
    for r in runs:
        if r.get("warnings"):
            lines.append(f"- **{r.get('case')}**：{', '.join(r['warnings'])}")
    lines.append("")
    lines.append("## 运行目录")
    lines.append("")
    for r in runs:
        lines.append(f"- {r.get('case')}：`{r.get('outdir')}`")
    lines.append("")
    return "\n".join(lines)
