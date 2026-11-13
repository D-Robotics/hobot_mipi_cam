# eval/05-exposure：曝光/触发时间戳与 t0 口径

| 项 | 内容 |
|---|---|
| 分支轮次 | `eval/05-exposure` |
| 日期 | 2026-11-12 |
| 平台 | RDK X5 V1.0，kernel 6.1.83，TROS humble |
| 传感器 | sc132gs 双目（pipe0/pipe1），1088x1280@30 nv12 / `dual_combine=1` |
| 用例 | C04：sensor ts + lpwm=off<br>C08：realtime ts + lpwm=off<br>C07：sensor ts + lpwm=on |
| 结果 | **C07 触发可用**（exp 稳定，双目同一触发点）；**C04/C08 在 lpwm=off 时 `trig_valid` 假阳性**，曝光不可用；**C08 的 realtime stamp 被 int32 溢出污染 −2.147s** |
| 原始数据 | C04：`eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_125226/`<br>C08：`eval/logs/C08_dual-1088x1280-nv12-30fps-combine1-realtime_20000104_124315/`<br>C07：`eval/logs/C07_dual-1088x1280-nv12-30fps-combine1-lpwm_20000104_124655/` |
| 指标 JSON | `metrics.json`（C04/C08/C07 全量分析结果） |

> C04 首轮（`..._123938`）因用例 YAML 未启用 `exposure` 事件被丢弃，本报告使用补跑 `..._125226`。

## 1. 结论速览

| 用例 | `trig_valid` | trig 可用 | 曝光 `exp_us` | 发布 stamp | t0 口径 |
|---|---|---|---|---|---|
| C04（lpwm=off，sensor） | 100%（假阳性） | 0% | 158.1~331.1 **s**（随时间线性增长） | 正常（sensor 用 `info.timestamps`） | 退化为帧 ts |
| C08（lpwm=off，realtime） | 100%（假阳性） | 0% | 169658.9~169831.7 **s** | **被偏移 −2.147s** | 退化为帧 ts（自身已偏移） |
| C07（lpwm=on，sensor） | 100% | **100%** | **4740/5990 µs**（左右目） | 正常 | **trig**，间隔 33.334ms |

一句话：**X5 上 `trig_tv` 只在 LPWM 开启时才有意义；关闭后驱动不清零，残留最后一次触发值，使 `trig_valid`（仅判非零）在本机恒为真。**

## 2. C07：LPWM 触发路径可用

| 流 | 帧数 | trig_valid | trig 可用 | exp p50(us) | exp min~max(us) | t0 间隔 p50(ms) | t0 回退 |
|---|---|---|---|---|---|---|---|
| 0/main | 5189 | 1.0 | 1.0 | 4740 | 4740~10216 | 33.334 | 0 |
| 1/main | 5188 | 1.0 | 1.0 | 5990 | 5990~10216 | 33.334 | 0 |

- `trig_tv` 逐帧递增，周期 **33.334 ms**（5015/5188 帧精确 33.334ms，其余 33.287~33.300ms），与 30fps 目标一致。
- **双目同一触发点**：成对帧（5188 组）中两 pipe 的 `ts_trig_ns` 完全一致，0 处不一致，触发是硬件级同步的。
- 曝光：pipe0 稳定 **4740/4741 µs**，pipe1 稳定 **5990/5991 µs**，左右目相差 **1.25 ms（26%）**；`sync_ae` 默认 false 且本轮未启用，两目 AE 独立。
- 启动瞬态：前 2 帧 9859 µs、第 3~5 帧 10216 µs，之后收敛到稳态。
- 说明：`exp_us = ts_hw − ts_trig`，即“触发→帧可取”的硬件间隔，含曝光+读出/传输，是曝光时长的**上界**，不是纯曝光。

## 3. C04：lpwm=off 时 trig 假阳性（sensor 模式不影响发布）

- `trig_tv` 全程恒定 **244051766893000 ns（244051.767 s）**，只有 1 个唯一值；它正好等于 C07 最后一次记录的触发 `244051733559000` **+ 33.334 ms**（一个触发周期），即 LPWM 关闭前最后锁存的那一拍。
- 由于 `trig_tv` 非零，产品代码判 `trig_valid=1`，`exp_us` 随之是“当前 `tv` − 陈旧触发”的差值：从 158.08 s 线性增长到 331.08 s（每帧 +33.3 ms）。
- 但 `frame_ts_type=sensor` 分支使用的是 `info.timestamps`，因此**发布 stamp 与设备帧序完全正常**（设备侧间隔 33.333 ms，t0 退化后间隔 33.333 ms）。
- 影响面：`exp_us` 不可用、`sys_timestamps` 被错误回退（sensor 模式不使用该值，暂不影响发布）。

## 4. C08：realtime stamp 被 int32 溢出污染 −2.147s

- `trig_tv` 同样是陈旧值：全程恒定 **73999485154000 ns（73999.485 s）**。
- 产品代码按 `int32_t exposure_time = (tv.sec−trig.sec)*1e9 + (tv.usec−trig.usec)*1e3` 计算，真实差值约 **1.6966e14 ns**，远超 int32；实际回退量恒为 **2147483647 ns（INT32_MAX）**，即 int32 转换饱和。
- 证据：`ts_sys_raw_ns − ts_frame` 在全部 **10373/10373 帧**上恒为 **2147483647 ns（INT32_MAX）**，唯一值仅 1 个；发布 stamp 与 `ts_frame` 仅差 7~41 ns，排除 DDS/录制侧改写。
- 结果：`frame_ts_type=realtime` 时发布 stamp 恒定**早 2.147 s**；realtime 回退校验 **10373/10373 帧不符**（pipe0 5187、pipe1 5186）。
- 同时 `exp_us` 为 1.6966e11~1.6983e11 µs（随 `tv` 增长），t0 无法按触发计算，只能退化为帧 ts。
- 该偏移解释了历史“图像端到端偏移约 2.6s 且恒定”中的实时分支部分：realtime 路径被错误回退 2.147 s。sensor 路径另有 **~3.9 s** 的 stamp 落后，本轮无法区分它是“时钟域偏移”还是“真实帧龄/积压”，留 09-domain-audit 定性。

## 5. 时钟域：三套时钟并存

| 量 | 域 | 相对 `CLOCK_MONOTONIC`（打点 `ts_mono`） |
|---|---|---|
| `tv` / `trig_tv`（硬件时戳） | 硬件/LPWM 域 | p50 按 run/pipe 为 **+2202.9 / +2203.3 / +2206.5 / +2208.6 / +2211.7 / +2212.1 ms**；同 run 内极差 **1.5~9.3 ms**（主要来自启动瞬态，稳态 p1~p99 约 2.8 ms） |
| `info.timestamps`（sensor 发布 stamp） | 与 monotonic 同量级 | **−3.9 s**（C04：打点−stamp p50=3914.5ms，3912~3917ms） |
| `sys_timestamps` | realtime | 9.47e17 量级（本机 RTC 在 2000 年） |

- `tv − info.timestamps` 稳态为 **6117.0 ms**（C04/C07 一致；启动瞬态有 6.097~6.114 s 的低值），可作为两域的换算参考。
- 结论：**trig 口径的 t0 在硬件域，必须先减去约 2207±5 ms 才能与 `recv_mono` 对齐**；08 延迟分支应逐 run/pipe 标定该偏移，并按不确定度报告，否则延迟会差 2.2 s。

## 6. 代码与口径问题

| # | 问题 | 证据 | 本轮处置 |
|---|---|---|---|
| P1 | `trig_valid` 判据只判 `trig_tv` 非零，LPWM 关闭时残留旧值导致恒真 | C04/C08 均 100% trig_valid 但 exp 为数十秒 | 产品代码不改；分析链增加 `trig 可用` 闸门（trig 必须推进 + exp 上限随帧间隔放宽），不可用时 t0 退化为帧 ts 且告警 |
| P2 | `int32_t exposure_time` 溢出，realtime stamp 被回退 2.147 s | `sys_raw − ts_frame` 10373/10373 帧恒为 **2147483647 ns（INT32_MAX）** | 本轮只记录证据（提案已确认不修），留 08/修复分支 |
| P3 | `(tv_sec−trig_sec)*1e9` 用 double 计算后转 int32，属实现定义转换 | 同上 | 记录，待修复 |
| P4 | 无触发时没有可用的逐帧曝光时长 | 无 `trig_tv` 时 `exp` 不可得 | t0 明确降级为帧 ts，报告标注 |

## 7. 分析链

- `exposure_metrics`：按 `(pipe, stream)` 统计 `trig_valid/trig_usable` 覆盖率、`exp_us` 分位、`stamp_src`、`frame_domain`/`sys_raw_domain`、`tv−打点`/`hw−frame`/`sys_raw−frame` 差分、realtime 回退校验、`t0_basis` 与 t0 间隔自洽性。
- **trig 可信闸门**：`trig_valid` 之外再要求 `trig` 在推进（恒定即陈旧），且 `0 < exp ≤ max(1s, 1.5×帧间隔)`；闸门上限随帧间隔放宽，兼容低帧率长曝光。
- **t0 口径**：曝光可用才按触发算；不可用时退化为帧 ts，不拿垃圾曝光量修正。同一流内口径混用时输出 `t0_mixed` 告警。
- **exposure ↔ frame_in 互备**：延迟链优先用 exposure 的可用曝光，缺失时回退 frame_in 的 `exp_valid/exp_us`；realtime 域按方案 6.1 改为 `t0 = 帧ts + exp/2`（原先一律 `− exp/2`）。C07 因此得到曝光中点延迟，C04/C08 正确标注“无可信曝光”。
- **[EVAL] 行去重**：launch stdout 偶发把同一行写两次（ts_mono 完全相同），解析器对最近 8 行去重，去重条数写入 `meta.eval_duplicate_lines`（C04 本轮 1 条）。
- 报告新增「曝光/触发」章节与告警：无触发、trig 陈旧/假阳性、曝光部分可用、t0 口径混用、负曝光、realtime 回退不符。
- 单测 **34 项全过**（新增陈旧 trig、长曝光、口径混用、exposure→延迟、去重、realtime 回退不符、负曝光/报告告警）。

## 8. 复现

```bash
# 板端（192.168.127.10，NFS 仓库）
cd /home/sunrise/workspace/hobot_mipi_cam
./eval/run/run_case.sh C04 180
./eval/run/run_case.sh C08 180
./eval/run/run_case.sh C07 180

# 主机分析
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C07_dual-1088x1280-nv12-30fps-combine1-lpwm_20000104_124655/recorder.csv \
  --log eval/logs/C07_dual-1088x1280-nv12-30fps-combine1-lpwm_20000104_124655/mipi_cam.eval.log \
  --out-md /tmp/C07.md --out-json /tmp/C07.json
```

## 9. 局限与后续

- `exp_us` 是触发到取帧的硬件间隔，不是纯曝光；需用 `hbn_isp_get_exposure_attr`（或 ISP 日志）对照，才能把曝光与读出拆开。
- C07 仅覆盖 30fps、combine=1、无 sub_stream；未测其它帧率下的 LPWM 周期与占空。
- 未验证“从未开过 LPWM 的冷启动”下 `trig_tv` 是否为 0（决定假阳性是否必然发生）。
- 未测 realtime + lpwm 组合（真正可用的 realtime t0 需要同时具备触发与正确回退）。
- 未测 `sync_ae=true`；若要求双目曝光一致，需单独复测左右 `exp_us` 差。
- 未做重启/换板复现，P1/P2 的普适性需多板确认。
- 交接：08-device-latency 用 C07 的 trig t0（逐 run/pipe 标定约 −2207 ms 的域偏移，并带 ±5 ms 不确定度）；09-domain-audit 固化 `tv`/`info.timestamps`/realtime 三域映射；修复分支处理 P2/P3。
- `tv−打点` 偏移在启动后数秒内会变化（极差可达 9 ms），做延迟基线时需剔除启动窗口或按稳态标定。
