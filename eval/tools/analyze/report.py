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


def _dash(v):
    return "-" if v is None else v


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

    st = metrics.get("startup")
    if st:
        lines.append("## 启动阶段（startup 埋点）")
        lines.append("")
        lines.append(f"- 起点 t0（MipiCamNode 构造函数体起点，monotonic）：{st.get('t0_ns', '-')} ns")
        lines.append(f"- 首帧耗时（相对 t0，最早 pipe/stream）：{st.get('time_to_first_frame_ms', '-')} ms")
        lines.append(f"- 首次发布耗时（相对 t0，最早 topic）：{st.get('time_to_first_pub_ms', '-')} ms")
        if not st.get("complete", True):
            lines.append("- **WARNING：启动时间线不完整**（backend_init/start 未成功或无 first_frame）")
        missing_fp = st.get("missing_first_pub") or []
        if missing_fp:
            lines.append(f"- **WARNING：以下图像 topic 没有 first_pub**：{', '.join(missing_fp)}")
        lines.append("")
        lines.append("| 阶段 | pipe | stream | topic | ret | 相对 t0 (ms) |")
        lines.append("|---|---|---|---|---|---|")
        for s in st.get("stages", []):
            lines.append(
                f"| {s.get('stage', '-')} | {s.get('pipe', '-')} "
                f"| {s.get('stream', '-')} | {s.get('topic', '-')} "
                f"| {s.get('ret', '-')} | {s.get('since_t0_ms', '-')} |")
        lines.append("")

        durs = st.get("stage_durations") or []
        if durs:
            lines.append("### 阶段耗时（begin/end 配对）")
            lines.append("")
            lines.append("| 阶段 | pipe | 耗时(ms) | ret |")
            lines.append("|---|---|---|---|")
            for d in durs:
                lines.append(
                    f"| {d.get('stage', '-')} | {d.get('pipe', '-')} "
                    f"| {d.get('duration_ms', '-')} | {d.get('ret', '-')} |")
            lines.append("")

    ft = metrics.get("frame_timing")
    if ft:
        lines.append("## 设备侧帧时序（frame_in 埋点）")
        lines.append("")
        lines.append("| 流 | 帧数 | 均值fps | 间隔p50(ms) | p95(ms) | p99(ms) | max(ms) | 回退 | 重复 |")
        lines.append("|---|---|---|---|---|---|---|---|---|")
        for key, m in sorted(ft.items()):
            lines.append(
                f"| {key} | {m['count']} | {m.get('fps_mean', '-')} "
                f"| {_fmt_ns(m.get('interval_p50_ns'))} | {_fmt_ns(m.get('interval_p95_ns'))} "
                f"| {_fmt_ns(m.get('interval_p99_ns'))} | {_fmt_ns(m.get('interval_max_ns'))} "
                f"| {m.get('stamp_monotonic_violations', '-')} | {m.get('stamp_duplicates', '-')} |")
        lines.append("")

    sw = metrics.get("stability_warnings")
    if sw:
        lines.append("## 稳定性告警")
        lines.append("")
        lines.append("| 流 | 最大间隔(ms) | 阈值(ms) |")
        lines.append("|---|---|---|")
        for w in sw:
            lines.append(f"| {w.get('stream', '-')} | {w.get('interval_max_ms', '-')} "
                         f"| {w.get('threshold_ms', '-')} |")
        lines.append("")

    ex = metrics.get("exposure")
    if ex:
        lines.append("## 曝光/触发（exposure 埋点）")
        lines.append("")
        lines.append(f"- exposure 事件总数：{ex.get('total', 0)}")
        lines.append("")
        lines.append("| 流 | 帧数 | trig_valid 覆盖率 | trig 可用率 | stamp_src | exp p50(us) | exp min~max(us) | 负值 | 0值 | >1s |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for key, m in sorted(ex.get("pipes", {}).items()):
            e = m.get("exp_us") or {}
            src = ", ".join(f"{k}:{v}" for k, v in (m.get("stamp_src") or {}).items())
            lines.append(
                f"| {key} | {m.get('count', '-')} | {m.get('trig_valid_ratio', '-')} "
                f"| {m.get('trig_usable_ratio', '-')} | {src or '-'} | {_dash(e.get('p50'))} "
                f"| {_dash(e.get('min'))}~{_dash(e.get('max'))} "
                f"| {m.get('exp_negative', '-')} | {m.get('exp_zero', '-')} "
                f"| {m.get('exp_over_1s', '-')} |")
        lines.append("")
        lines.append("| 流 | 帧域 | sys_raw 域 | tv−打点(ms) | hw−frame p50(ms) | sys_raw−frame p50(ms) | realtime 回退不符 | t0 口径 | t0 间隔 p50/p95(ms) | t0 回退 |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for key, m in sorted(ex.get("pipes", {}).items()):
            hw = m.get("hw_minus_frame_ms") or {}
            sd = m.get("sys_raw_minus_frame_ms") or {}
            hm = m.get("hw_minus_event_mono_ms") or {}
            t0 = m.get("t0_interval_ms") or {}
            ra = m.get("realtime_adjust") or {}
            basis = ", ".join(f"{k}:{v}" for k, v in (m.get("t0_basis") or {}).items())
            # hw−frame 在 realtime 帧域下是跨域差分，量级无意义，不展示
            hw_cell = "-" if m.get("frame_domain") == "realtime" else _dash(hw.get("p50"))
            lines.append(
                f"| {key} | {m.get('frame_domain', '-')} | {m.get('sys_raw_domain', '-')} "
                f"| {_dash(hm.get('p50'))} | {hw_cell} | {_dash(sd.get('p50'))} "
                f"| {ra.get('mismatch', 0)}/{ra.get('checked', 0)} | {basis or '-'} "
                f"| {_dash(t0.get('p50'))}/{_dash(t0.get('p95'))} "
                f"| {m.get('t0_backwards', '-')} |")
        lines.append("")

        pipes = ex.get("pipes", {})
        if pipes and all((m.get("trig_valid_ratio") or 0) == 0 for m in pipes.values()):
            lines.append("- **WARNING：所有流 trig_valid=0，逐帧曝光不可得，t0 只能退化为帧 ts（无曝光中点修正）**")
            lines.append("")
        for key, m in sorted(pipes.items()):
            ratio = m.get("trig_usable_ratio")
            if m.get("trig_stale"):
                lines.append(f"- **WARNING：{key} trig_tv 恒定（陈旧值，LPWM 关闭后的残留），"
                             f"曝光不可用**")
            elif (m.get("trig_valid_ratio") or 0) > 0 and (ratio or 0) == 0:
                lines.append(f"- **WARNING：{key} trig_valid 为真但曝光不可用"
                             f"（触发跨域/超上限，上限 {m.get('exp_cap_us', '-')}us），"
                             f"t0 未按触发计算**")
            elif ratio is not None and 0 < ratio < 1:
                lines.append(f"- **WARNING：{key} 仅 {ratio:.2%} 帧曝光可用，"
                             f"t0 口径混用（trig / 帧 ts）**")
            if m.get("t0_mixed"):
                lines.append(f"- **WARNING：{key} t0 混用触发域与帧域，"
                             f"间隔统计会跨域跳变**")
            if (m.get("exp_negative") or 0) > 0:
                lines.append(f"- **WARNING：{key} 出现负曝光值 {m['exp_negative']} 帧**")
            ra = m.get("realtime_adjust") or {}
            if (ra.get("mismatch") or 0) > 0:
                lines.append(f"- **WARNING：{key} realtime 分支回退校验不符 "
                             f"{ra['mismatch']}/{ra['checked']} 帧**")
        lines.append("")

    sp = metrics.get("sync_pair")
    if sp:
        lines.append("## 双目/主辅配对（sync_pair 埋点）")
        lines.append("")
        lines.append("| tag | 配对数 | 上限(帧数) | 配对率 | diff p50/p95/p99/max(ms) | 容差(ms) | frame_id 错配 |")
        lines.append("|---|---|---|---|---|---|---|")
        for tag, m in sorted(sp.items()):
            rate = m.get("pair_rate")
            rate_s = f"{rate:.2%}" if rate is not None else "-"
            lines.append(
                f"| {tag} | {m.get('pairs', '-')} | {_dash(m.get('expected'))} "
                f"| {rate_s} "
                f"| {_fmt_ns(m.get('diff_p50_ns'))}/{_fmt_ns(m.get('diff_p95_ns'))}"
                f"/{_fmt_ns(m.get('diff_p99_ns'))}/{_fmt_ns(m.get('diff_max_ns'))} "
                f"| {_fmt_ns(m.get('tol_ns'))} | {m.get('frame_id_mismatch', '-')} |")
        lines.append("")
        for tag, m in sorted(sp.items()):
            fails = m.get("fails") or {}
            if fails:
                detail = ", ".join(f"{k}:{v}" for k, v in fails.items())
                extra = ""
                if m.get("missing_asym"):
                    extra = f"（单侧有帧 {m['missing_asym']} 次为到达不对称）"
                elif m.get("missing_idle"):
                    extra = f"（两路都空 {m['missing_idle']} 次为帧间常态）"
                lines.append(f"- {tag} 配对失败（1 条/秒限流）：{detail}{extra}")
            if m.get("pair_rate") is not None and m["pair_rate"] < 0.999:
                lines.append(f"- **WARNING：{tag} 配对率 {m['pair_rate']:.2%} 低于 99.9% 基线**")
            if (m.get("diff_p99_ns") or 0) > 1e6:
                lines.append(f"- **WARNING：{tag} 配对差 p99="
                             f"{_fmt_ns(m.get('diff_p99_ns'))}ms 超过 1ms 口径**")
            if (m.get("frame_id_mismatch") or 0) > 0:
                nz = {k: v for k, v in (m.get("frame_id_delta") or {}).items() if k != "0"}
                desc = ", ".join(f"Δ={k}:{v}" for k, v in nz.items())
                if len(nz) == 1:
                    kind = "固定帧号偏移（启动相位/计数器起点），非随机错配"
                else:
                    kind = "帧号偏移不固定，可能掩盖丢帧"
                lines.append(f"- **WARNING：{tag} 有 {m['frame_id_mismatch']} 对 "
                             f"frame_id 不一致（{desc}；{kind}）**")
        lines.append("")

    cb = metrics.get("combine")
    if cb:
        lines.append("## 拼接产出（combine_out 埋点）")
        lines.append("")
        lines.append("| tag | 拼接帧数 | ts 来源(左/右/其他) | 右−左 p50(ms) | frame_id 与左一致 | 几何不符(宽/高) | order | B-11 设备侧 p50/p95(ms) | B-11 覆盖(设备/接收) | 接收侧 p50(ms) |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for tag, m in sorted(cb.items()):
            src = m.get("ts_source") or {}
            rl = m.get("ts_right_minus_left_ns") or {}
            dev = m.get("extra_latency_ns") or {}
            rcv = m.get("extra_recv_ns") or {}
            order = ", ".join(f"{k}:{v}" for k, v in (m.get("order") or {}).items())
            n = m.get("count") or 0
            cov = (f"{m.get('extra_latency_checked', 0)}/{n}、"
                   f"{m.get('extra_recv_checked', 0)}/{n}")
            lines.append(
                f"| {tag} | {m.get('count', '-')} "
                f"| {src.get('left', 0)}/{src.get('right', 0)}/{src.get('other', 0)} "
                f"| {_fmt_ns(rl.get('p50'))} "
                f"| {m.get('frame_id_match_left', '-')}/{m.get('count', '-')} "
                f"| {m.get('width_mismatch', '-')}/{m.get('height_mismatch', '-')} "
                f"| {order or '-'} "
                f"| {_fmt_ns(dev.get('p50'))}/{_fmt_ns(dev.get('p95'))} "
                f"| {cov} "
                f"| {_fmt_ns(rcv.get('p50'))} |")
        lines.append("")
        for tag, m in sorted(cb.items()):
            src = m.get("ts_source") or {}
            if (src.get("right") or 0) > 0 or (src.get("other") or 0) > 0:
                lines.append(f"- **WARNING：{tag} ts_combine 来源不是左目"
                             f"（right {src.get('right', 0)}，other {src.get('other', 0)}）**")
            if (m.get("frame_id_match_left") or 0) < (m.get("count") or 0):
                lines.append(f"- **WARNING：{tag} 有 "
                             f"{(m.get('count') or 0) - (m.get('frame_id_match_left') or 0)} 帧 "
                             f"frame_id 与左目不一致**")
            if (m.get("width_mismatch") or 0) > 0 or (m.get("height_mismatch") or 0) > 0:
                lines.append(f"- **WARNING：{tag} 输出几何与期望不符："
                             f"宽 {m.get('width_mismatch')} 帧、高 {m.get('height_mismatch')} 帧"
                             f"（期望 {m.get('expected_width')}x{m.get('expected_height')}）**")
            bad_order = [k for k in (m.get("order") or {}) if k != "0,1"]
            if bad_order:
                lines.append(f"- **WARNING：{tag} 出现非 0,1 的组合顺序：{bad_order}**")
            if (m.get("pub_duplicates") or 0) > 0 or (m.get("recv_duplicates") or 0) > 0:
                lines.append(f"- **WARNING：{tag} 关联键有重复（pub {m.get('pub_duplicates', 0)}、"
                             f"recv {m.get('recv_duplicates', 0)}），B-11 统计可能偏少**")
        lines.append("")

    dr = metrics.get("drop")
    if dr:
        lines.append("## 设备侧丢帧（frame_drop + queue_stat）")
        lines.append("")
        if not dr.get("queue_counters_available", False):
            lines.append("- **WARNING：没有 queue_stat 事件，队列丢弃计数不可用**")
        lines.append("| 流 | 收到帧 | 丢帧总数 | 丢帧率 | queue_recycle | empty_null | fq_full | fq_expired | getframe_fail | frame_id缺口(归因) |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for key, m in sorted(dr.get("streams", {}).items()):
            r = m.get("drops_by_reason", {})
            lines.append(
                f"| {key} | {m.get('frames_in', '-')} | {m.get('drops_total', '-')} "
                f"| {_dash(m.get('drop_rate'))} | {r.get('queue_recycle', '-')} "
                f"| {r.get('empty_null', '-')} | {r.get('fq_full', '-')} "
                f"| {r.get('fq_expired', '-')} | {r.get('getframe_fail', '-')} "
                f"| {_dash(m.get('frame_id_gaps'))} |")
        lines.append("")
        task = dr.get("task_drops", {})
        if task:
            lines.append("| 任务 | select_timeout | select_error |")
            lines.append("|---|---|---|")
            for name, counts in sorted(task.items()):
                lines.append(f"| {name} | {counts.get('select_timeout', 0)} | {counts.get('select_error', 0)} |")
            lines.append("")
        combine = dr.get("combine", {})
        if any(combine.get(k, 0) for k in ("queue_recycle", "empty_null")):
            lines.append(
                f"- combine 队列：recycle={combine.get('queue_recycle', 0)}，"
                f"empty_null={combine.get('empty_null', 0)}，depth={combine.get('depth', 0)}")
            lines.append("")

    rs = metrics.get("resolution")
    if rs:
        lines.append("## 分辨率准确性（frame_in + recorder 元数据）")
        lines.append("")
        if not rs.get("expected_available", False):
            lines.append("- **WARNING：未能从 meta.txt 解析期望分辨率，仅列出实际值**")
        if not rs.get("device_covered", False):
            lines.append("- **WARNING：设备侧没有带 width/height 的 frame_in，设备侧未覆盖**")
        if rs.get("missing_topics"):
            lines.append(f"- **WARNING：缺少期望 topic**：{', '.join(rs['missing_topics'])}")
        lines.append("| 面 | 流/topic | 期望 | 实际（唯一值） | mismatch |")
        lines.append("|---|---|---|---|---|")
        for key, m in sorted(rs.get("device", {}).items()):
            obs = ", ".join(f"{k}({v})" for k, v in m.get("observed", {}).items())
            lines.append(f"| device | {key} | {_dash(m.get('expected'))} | {obs} | "
                         f"{_dash(m.get('mismatch'))} |")
        for topic, m in sorted(rs.get("published", {}).items()):
            obs = ", ".join(f"{k}({v})" for k, v in m.get("observed", {}).items())
            lines.append(f"| published | {topic} | {_dash(m.get('expected'))} | {obs} | "
                         f"{_dash(m.get('mismatch'))} |")
        lines.append("")
        lines.append(f"- 设备侧 mismatch：{rs.get('device_mismatch', '-')}；"
                     f"发布侧 mismatch：{rs.get('published_mismatch', '-')}；"
                     f"全部一致：{rs.get('all_match', '-')}")
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
        lines.append("## 设备侧采集（frame_id/曝光；frame_id 缺口仅归因）")
        lines.append("")
        lines.append("| 流 | 采集帧数 | frame_id反推丢帧 | 丢帧率 | 曝光有效率 | 曝光均值(µs) |")
        lines.append("|---|---|---|---|---|---|")
        for key, m in sorted(fi.items()):
            lines.append(
                f"| {key} | {m['count']} | {_dash(m.get('dropped_by_frame_id'))} "
                f"| {_dash(m.get('drop_rate'))} | {_dash(m.get('exp_valid_ratio'))} "
                f"| {_dash(m.get('exp_us_mean'))} |")
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
        if (metrics.get("exposure") and
                not any(m.get("exp_midpoint") for m in lat.values())):
            lines.append("- **NOTE：exposure 事件存在，但本轮没有可信曝光量，"
                         "延迟全部按帧 ts 口径（t0 未做曝光中点修正）**")
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
